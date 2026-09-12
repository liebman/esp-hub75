//! HUB75 driver for `PARL_IO` peripherals (ESP32-C5 / ESP32-C6).
//!
//! The `TxEof` interrupt runs the BCM (Binary Code Modulation) refresh loop,
//! so the panel keeps scanning out the current framebuffer on its own.
//! Buffer swaps take effect at frame boundaries.
//!
//! ## Blocking Example
//!
//! ```rust,ignore
//! let hub75 = Hub75::new(
//!     peripherals.PARL_IO, pins, peripherals.DMA_CH0,
//!     tx_descriptors, Hub75Config::new(), &*fb,
//! ).expect("failed to create Hub75");
//!
//! // Display refreshes on its own; the main thread is free.
//! loop { core::hint::spin_loop(); }
//! ```
//!
//! ## Async Example
//!
//! ```rust,ignore
//! let hub75 = Hub75::new_async(
//!     peripherals.PARL_IO, pins, peripherals.DMA_CH0,
//!     tx_descriptors, Hub75Config::new(), &*fb0,
//! ).expect("failed to create Hub75");
//!
//! // Swap buffers; `wait()` spin-loops until the DMA no longer reads the
//! // old framebuffer. Use `wait_for_done().await` first to yield instead.
//! let old_fb = hub75.swap(fb1)?.wait().expect("DMA error");
//! ```

use esp_hal::parl_io::BitPackOrder;
use esp_hal::parl_io::ClkOutPin;
use esp_hal::parl_io::ConfigurePins;
use esp_hal::parl_io::ParlIo;
use esp_hal::parl_io::ParlIoDmaChannel;
use esp_hal::parl_io::ParlIoInterrupt;
use esp_hal::parl_io::SampleEdge;
use esp_hal::parl_io::TxConfig;
use esp_hal::parl_io::TxPins;
use esp_hal::peripherals::PARL_IO;

use crate::Hub75Backend;
use crate::Hub75Config;
use crate::Hub75DmaDescriptors;
use crate::Hub75Error;
use crate::Hub75Pins;
use crate::Hub75Pins8;
#[cfg(not(esp32c5))]
use crate::Hub75Pins16;
use crate::framebuffer::WordSize;
use crate::isr::BcmBuf;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

/// Selects the DMA channel as the TX EOF source on the ESP32-C5.
///
/// The ESP32-C5 `PARL_IO` can generate TX EOF from its bit-length counter or
/// from the GDMA channel's EOF signal; `tx_eof_gen_sel` chooses between them.
/// The HUB75 refresh loop is driven by EOF events that correspond to the end
/// of the descriptor chain (linear mode) or the armed boundary descriptor
/// (circular mode), so the DMA channel source is required for the ISR to fire.
///
/// # Panics
///
/// Panics if the `PARL_IO` peripheral cannot be accessed. This only happens
/// when the peripheral was taken by another driver instance.
// SAFETY: The `PARL_IO` peripheral is owned by the caller's `ParlIo` instance;
// this steals a second handle only to write the EOF-source select bit during
// construction, before the ISR is enabled and while no transfer is running, so
// there is no concurrent register access.
#[cfg(esp32c5)]
fn select_dma_eof_source() {
    unsafe {
        PARL_IO::steal()
            .register_block()
            .tx_genrl_cfg()
            .modify(|_, w| w.tx_eof_gen_sel().set_bit());
    }
}

/// Clears the `tx_eof` flag on the `PARL_IO` peripheral.
///
/// Called by the boundary ISR (circular mode) to drain a stale flag. The
/// ISR must **not** clear this flag on the handled boundary before
/// `wait()`: `wait()` polls `INT_RAW.tx_eof` — the very flag that fired the
/// ISR — and clears it itself on completion.
// SAFETY: The `PARL_IO` peripheral handle is owned by the driver; this
// steals a second handle only to write the interrupt-clear register from
// ISR context. The write is a single register store, and the driver never
// touches this register concurrently.
#[cfg_attr(not(feature = "circular-dma"), allow(dead_code))]
pub(crate) fn clear_frame_interrupt() {
    unsafe {
        PARL_IO::steal()
            .register_block()
            .int_clr()
            .write(|w| w.tx_eof().clear_bit_by_one());
    }
}

impl<FB, P, CH> Hub75Backend<FB, P, CH> for PARL_IO<'static>
where
    FB: crate::framebuffer::FrameBuffer + 'static,
    P: Hub75Pins<Word = FB::Word> + ParlIoPins<'static>,
    CH: ParlIoDmaChannel<'static>,
{
    fn construct<const N: usize>(
        self,
        hub75_pins: P,
        channel: CH,
        tx_descriptors: &'static mut Hub75DmaDescriptors<FB, N>,
        config: Hub75Config,
    ) -> Result<(), Hub75Error> {
        let (pins, clock_pin) = hub75_pins.convert_pins();

        let mut parl_io_dev = ParlIo::new(self, channel)?;

        // Bind the unified refresh ISR to the `PARL_IO` interrupt, and enable
        // the `TxEof` source for both refresh modes (before the TX
        // configuration consumes `parl_io_dev.tx`):
        //
        // - Linear mode: the `TxEof` source runs the BCM loop.
        // - Circular mode: the ring carries no `suc_eof`, so nothing fires until a swap
        //   arms the pass-boundary detector; the source stays enabled for the driver's
        //   lifetime.
        parl_io_dev.set_interrupt_handler(crate::isr::handler_with_priority(
            crate::isr::isr,
            config.interrupt_priority,
        ));

        // `listen` must precede `parl_io_dev.tx.with_config`, which partially
        // moves `parl_io_dev`.
        parl_io_dev.listen(ParlIoInterrupt::TxEof);

        #[cfg(feature = "invert-clock")]
        let sample_edge = SampleEdge::Normal;
        #[cfg(not(feature = "invert-clock"))]
        let sample_edge = SampleEdge::Invert;

        #[cfg(feature = "invert-blank")]
        let idle_value = 0x0000;
        #[cfg(not(feature = "invert-blank"))]
        let idle_value = 0x0100;

        let tx_config = TxConfig::default()
            .with_frequency(config.frequency)
            .with_idle_value(idle_value)
            .with_sample_edge(sample_edge)
            .with_bit_order(BitPackOrder::Msb);

        let clk_pin = ClkOutPin::new(clock_pin);
        let parl_io_tx = parl_io_dev.tx.with_config(pins, clk_pin, tx_config)?;

        // On the C5 the TX EOF signal comes from the DMA channel rather than
        // the peripheral's bit-length counter (`tx_eof_gen_sel` selects the
        // source). In linear mode the refresh-loop ISR never fires otherwise;
        // in circular mode, with a transfer length of 0 the frame ends at the
        // armed `suc_eof` descriptor regardless of its size.
        #[cfg(esp32c5)]
        select_dma_eof_source();

        let buf = BcmBuf::new(tx_descriptors.as_slice());
        crate::isr::init_state(parl_io_tx, buf);

        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Pin configurations
// ---------------------------------------------------------------------------

use esp_hal::gpio::AnyPin;
#[cfg(not(esp32c5))]
use esp_hal::gpio::NoPin;
use esp_hal::parl_io::TxEightBits;
#[cfg(not(esp32c5))]
use esp_hal::parl_io::TxSixteenBits;

#[cfg(not(esp32c5))]
impl crate::Hub75Pins for Hub75Pins16<'_> {
    type Word = u16;

    fn word_size(&self) -> WordSize {
        WordSize::Sixteen
    }
}

impl crate::Hub75Pins for Hub75Pins8<'_> {
    type Word = u8;

    fn word_size(&self) -> WordSize {
        WordSize::Eight
    }
}

/// Converts a HUB75 pin configuration into the `PARL_IO` pin format.
///
/// This trait is internal to the driver and is not part of the public API.
#[doc(hidden)]
pub trait ParlIoPins<'d> {
    /// The peripheral-specific pin format this configuration converts to
    /// (`TxEightBits` or `TxSixteenBits`).
    type Pins: TxPins + ConfigurePins + 'd;

    /// Converts the high-level pin definition into the peripheral-specific
    /// format, returning the converted pins and the clock pin.
    fn convert_pins(self) -> (Self::Pins, AnyPin<'d>);
}

#[cfg(not(esp32c5))]
impl<'d> ParlIoPins<'d> for Hub75Pins16<'d> {
    type Pins = TxSixteenBits<'d>;

    fn convert_pins(self) -> (TxSixteenBits<'d>, AnyPin<'d>) {
        let blank = self.blank.into_output_signal();
        #[cfg(feature = "invert-blank")]
        let blank = blank.with_output_inverter(true);

        let pins = TxSixteenBits::new(
            self.addr0, self.addr1, self.addr2, self.addr3, self.addr4, self.latch, NoPin, NoPin,
            blank, self.red1, self.grn1, self.blu1, self.red2, self.grn2, self.blu2, NoPin,
        );
        (pins, self.clock)
    }
}

impl<'d> ParlIoPins<'d> for Hub75Pins8<'d> {
    type Pins = TxEightBits<'d>;

    fn convert_pins(self) -> (TxEightBits<'d>, AnyPin<'d>) {
        let blank = self.blank.into_output_signal();
        #[cfg(feature = "invert-blank")]
        let blank = blank.with_output_inverter(true);

        let pins = TxEightBits::new(
            self.red1, self.grn1, self.blu1, self.red2, self.grn2, self.blu2, self.latch, blank,
        );
        (pins, self.clock)
    }
}
