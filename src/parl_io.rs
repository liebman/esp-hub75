//! HUB75 driver for PARL_IO peripherals (ESP32-C5 / ESP32-C6).
//!
//! This module provides an interrupt-driven display controller that
//! continuously refreshes a HUB75 panel from a framebuffer. The PARL_IO
//! `TxEof` interrupt drives the entire BCM (Binary Code Modulation) refresh
//! loop. Buffer swaps happen atomically at frame boundaries.
//!
//! # Blocking example
//!
//! ```rust,ignore
//! let hub75 = Hub75::new(
//!     peripherals.PARL_IO, pins, peripherals.DMA_CH0,
//!     tx_descriptors, Rate::from_mhz(20),
//! ).expect("failed to create Hub75");
//! hub75.start(&*fb).expect("failed to start Hub75");
//!
//! // Display refreshes automatically — main thread is free.
//! loop { core::hint::spin_loop(); }
//! ```
//!
//! # Async example
//!
//! ```rust,ignore
//! let hub75 = Hub75::new_async(
//!     peripherals.PARL_IO, pins, peripherals.DMA_CH0,
//!     tx_descriptors, Rate::from_mhz(20),
//! ).expect("failed to create Hub75");
//! hub75.start(&*fb0).expect("failed to start Hub75");
//!
//! // Swap buffers — yields to the executor, returns Err on DMA failure.
//! let old_ptr = hub75.swap(&*fb1).await.expect("DMA error");
//! ```

use esp_hal::dma::DmaChannelFor;
use esp_hal::dma::DmaDescriptor;
use esp_hal::parl_io::BitPackOrder;
use esp_hal::parl_io::ClkOutPin;
use esp_hal::parl_io::ConfigurePins;
use esp_hal::parl_io::ParlIo;
use esp_hal::parl_io::ParlIoInterrupt;
use esp_hal::parl_io::SampleEdge;
use esp_hal::parl_io::TxConfig;
use esp_hal::parl_io::TxPins;
use esp_hal::peripherals::PARL_IO;
use esp_hal::time::Rate;
use esp_hal::Blocking;

use crate::bcm_buf::BcmBuf;
pub use crate::isr::Hub75;
use crate::Hub75Error;
use crate::Hub75Pins;
#[cfg(not(feature = "esp32c5"))]
use crate::Hub75Pins16;
use crate::Hub75Pins8;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

impl<DM: esp_hal::DriverMode, FB: crate::framebuffer::FrameBuffer + 'static> Hub75<DM, FB> {
    fn new_internal<
        T: TxPins + ConfigurePins + 'static,
        P: Hub75Pins<'static, T, Word = FB::Word>,
    >(
        parl_io: PARL_IO<'static>,
        hub75_pins: P,
        channel: impl DmaChannelFor<PARL_IO<'static>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        let (pins, clock_pin) = hub75_pins.convert_pins();

        let mut parl_io_dev = ParlIo::new(parl_io, channel)?;

        parl_io_dev.set_interrupt_handler(crate::isr::hub75_isr);
        parl_io_dev.listen(ParlIoInterrupt::TxEof);

        #[cfg(feature = "invert-clock")]
        let sample_edge = SampleEdge::Normal;
        #[cfg(not(feature = "invert-clock"))]
        let sample_edge = SampleEdge::Invert;

        let config = TxConfig::default()
            .with_frequency(frequency)
            .with_idle_value(0)
            .with_sample_edge(sample_edge)
            .with_bit_order(BitPackOrder::Msb);

        let clk_pin = ClkOutPin::new(clock_pin);
        let parl_io_tx = parl_io_dev.tx.with_config(pins, clk_pin, config)?;

        // SAFETY: TODO: open issue or pr to esp-hal to set this when using GDMA and remove the C6 max length check
        #[cfg(feature = "esp32c5")]
        unsafe {
            use esp32c5 as pac;
            let pio = pac::PARL_IO::steal();
            pio.tx_genrl_cfg()
                .modify(|_, w| w.tx_eof_gen_sel().set_bit());
        }

        let buf = BcmBuf::new(tx_descriptors);
        crate::isr::init_isr_state(parl_io_tx, buf);
        crate::isr::start_internal(fb)?;

        Ok(Self::from_phantom())
    }
}

impl<FB: crate::framebuffer::FrameBuffer + 'static> Hub75<Blocking, FB> {
    /// Create a new blocking HUB75 driver.
    ///
    /// Configures the PARL_IO peripheral, applies pin assignments, and
    /// immediately starts DMA-driven display refresh with the provided
    /// framebuffer.
    ///
    /// The pin configuration's word type must match the framebuffer's word
    /// type — passing a 16-bit framebuffer with 8-bit pins (or vice versa)
    /// is a compile-time error.
    ///
    /// # Arguments
    /// * `parl_io` -- The PARL_IO peripheral instance
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `frequency` -- PARL_IO clock rate
    /// * `fb` -- Initial framebuffer to display
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new<T: TxPins + ConfigurePins + 'static, P: Hub75Pins<'static, T, Word = FB::Word>>(
        parl_io: PARL_IO<'static>,
        hub75_pins: P,
        channel: impl DmaChannelFor<PARL_IO<'static>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(parl_io, hub75_pins, channel, tx_descriptors, frequency, fb)
    }
}

impl<FB: crate::framebuffer::FrameBuffer + 'static> Hub75<esp_hal::Async, FB> {
    /// Create a new async HUB75 driver.
    ///
    /// Configures the PARL_IO peripheral, applies pin assignments, and
    /// immediately starts DMA-driven display refresh with the provided
    /// framebuffer.
    ///
    /// The pin configuration's word type must match the framebuffer's word
    /// type — passing a 16-bit framebuffer with 8-bit pins (or vice versa)
    /// is a compile-time error.
    ///
    /// # Arguments
    /// * `parl_io` -- The PARL_IO peripheral instance
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `frequency` -- PARL_IO clock rate
    /// * `fb` -- Initial framebuffer to display
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new_async<
        T: TxPins + ConfigurePins + 'static,
        P: Hub75Pins<'static, T, Word = FB::Word>,
    >(
        parl_io: PARL_IO<'static>,
        hub75_pins: P,
        channel: impl DmaChannelFor<PARL_IO<'static>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(parl_io, hub75_pins, channel, tx_descriptors, frequency, fb)
    }
}

// ---------------------------------------------------------------------------
// Pin configurations
// ---------------------------------------------------------------------------

use esp_hal::gpio::AnyPin;
#[cfg(not(feature = "esp32c5"))]
use esp_hal::gpio::NoPin;
use esp_hal::parl_io::TxEightBits;
#[cfg(not(feature = "esp32c5"))]
use esp_hal::parl_io::TxSixteenBits;

#[cfg(not(feature = "esp32c5"))]
impl<'d> crate::Hub75Pins<'d, TxSixteenBits<'d>> for Hub75Pins16<'d> {
    type Word = u16;

    fn convert_pins(self) -> (TxSixteenBits<'d>, AnyPin<'d>) {
        // SAFETY: We only use the output signal half. The original `AnyPin` is
        // consumed by the enclosing struct move, so there is no aliased access.
        let (_, blank) = unsafe { self.blank.split() };
        let pins = TxSixteenBits::new(
            self.addr0,
            self.addr1,
            self.addr2,
            self.addr3,
            self.addr4,
            self.latch,
            NoPin,
            NoPin,
            blank.with_output_inverter(true),
            self.red1,
            self.grn1,
            self.blu1,
            self.red2,
            self.grn2,
            self.blu2,
            NoPin,
        );
        (pins, self.clock)
    }
}

impl<'d> crate::Hub75Pins<'d, TxEightBits<'d>> for Hub75Pins8<'d> {
    type Word = u8;

    fn convert_pins(self) -> (TxEightBits<'d>, AnyPin<'d>) {
        // SAFETY: We only use the output signal half. The original `AnyPin` is
        // consumed by the enclosing struct move, so there is no aliased access.
        let (_, blank) = unsafe { self.blank.split() };
        let pins = TxEightBits::new(
            self.red1,
            self.grn1,
            self.blu1,
            self.red2,
            self.grn2,
            self.blu2,
            self.latch,
            #[cfg(feature = "invert-blank")]
            blank.with_output_inverter(true),
            #[cfg(not(feature = "invert-blank"))]
            blank,
        );
        (pins, self.clock)
    }
}
