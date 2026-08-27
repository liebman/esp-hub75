//! HUB75 driver for `PARL_IO` peripherals (ESP32-C5 / ESP32-C6).
//!
//! The `TxEof` interrupt runs the BCM (Binary Code Modulation) refresh loop,
//! so the panel keeps scanning out the current framebuffer on its own.
//! Buffer swaps take effect at frame boundaries.
//!
//! # Blocking example
//!
//! ```rust,ignore
//! let hub75 = Hub75::new(
//!     peripherals.PARL_IO, pins, peripherals.DMA_CH0,
//!     tx_descriptors, Rate::from_mhz(20), &*fb,
//! ).expect("failed to create Hub75");
//!
//! // Display refreshes on its own; the main thread is free.
//! loop { core::hint::spin_loop(); }
//! ```
//!
//! # Async example
//!
//! ```rust,ignore
//! let hub75 = Hub75::new_async(
//!     peripherals.PARL_IO, pins, peripherals.DMA_CH0,
//!     tx_descriptors, Rate::from_mhz(20), &*fb0,
//! ).expect("failed to create Hub75");
//!
//! // Swap buffers: yields to the executor, returns Err on DMA failure.
//! let old_fb = hub75.swap(fb1)?.wait().expect("DMA error");
//! ```

use esp_hal::Blocking;
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

use crate::Hub75Error;
use crate::Hub75Pins;
use crate::Hub75Pins8;
#[cfg(not(esp32c5))]
use crate::Hub75Pins16;
use crate::bcm::linear::BcmBuf;
pub use crate::isr::Hub75;

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
        crate::isr::claim_driver()?;

        let (pins, clock_pin) = hub75_pins.convert_pins();

        let mut parl_io_dev = ParlIo::new(parl_io, channel)?;

        parl_io_dev.set_interrupt_handler(crate::isr::hub75_isr);
        parl_io_dev.listen(ParlIoInterrupt::TxEof);

        #[cfg(feature = "invert-clock")]
        let sample_edge = SampleEdge::Normal;
        #[cfg(not(feature = "invert-clock"))]
        let sample_edge = SampleEdge::Invert;

        #[cfg(feature = "invert-blank")]
        let idle_value = 0x0000;
        #[cfg(not(feature = "invert-blank"))]
        let idle_value = 0x0100;

        let config = TxConfig::default()
            .with_frequency(frequency)
            .with_idle_value(idle_value)
            .with_sample_edge(sample_edge)
            .with_bit_order(BitPackOrder::Msb);

        let clk_pin = ClkOutPin::new(clock_pin);
        let parl_io_tx = parl_io_dev.tx.with_config(pins, clk_pin, config)?;

        // SAFETY: The driver above owns the PARL_IO peripheral. We steal it
        // only to set the `tx_eof_gen_sel` bit, so the DMA EOF
        // signal comes from the GDMA channel rather than the peripheral's
        // byte counter; esp-hal doesn't expose this register. The ISR isn't
        // active yet, so no data race.
        #[cfg(esp32c5)]
        unsafe {
            esp_hal::peripherals::PARL_IO::steal()
                .register_block()
                .tx_genrl_cfg()
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
    /// Configures the `PARL_IO` peripheral, applies pin assignments, and
    /// immediately starts DMA-driven display refresh with the provided
    /// framebuffer.
    ///
    /// The pin configuration's word type must match the framebuffer's word
    /// type; passing a 16-bit framebuffer with 8-bit pins (or vice versa)
    /// is a compile-time error.
    ///
    /// # Arguments
    /// * `parl_io` -- The `PARL_IO` peripheral instance
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `frequency` -- `PARL_IO` clock rate
    /// * `fb` -- Initial framebuffer to display
    /// # Errors
    ///
    /// Returns [`Hub75Error::AlreadyInitialised`] if a `Hub75` instance
    /// already exists. Returns [`Hub75Error::AlreadyRunning`] or
    /// [`Hub75Error::Dma`](crate::Hub75Error::Dma) /
    /// [`Hub75Error::ParlIo`](crate::Hub75Error::ParlIo) if the initial
    /// DMA transfer fails.
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
    /// Configures the `PARL_IO` peripheral, applies pin assignments, and
    /// immediately starts DMA-driven display refresh with the provided
    /// framebuffer.
    ///
    /// The pin configuration's word type must match the framebuffer's word
    /// type; passing a 16-bit framebuffer with 8-bit pins (or vice versa)
    /// is a compile-time error.
    ///
    /// # Arguments
    /// * `parl_io` -- The `PARL_IO` peripheral instance
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `frequency` -- `PARL_IO` clock rate
    /// * `fb` -- Initial framebuffer to display
    /// # Errors
    ///
    /// Returns [`Hub75Error::AlreadyInitialised`] if a `Hub75` instance
    /// already exists. Returns [`Hub75Error::AlreadyRunning`] or
    /// [`Hub75Error::Dma`](crate::Hub75Error::Dma) /
    /// [`Hub75Error::ParlIo`](crate::Hub75Error::ParlIo) if the initial
    /// DMA transfer fails.
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
#[cfg(not(esp32c5))]
use esp_hal::gpio::NoPin;
use esp_hal::parl_io::TxEightBits;
#[cfg(not(esp32c5))]
use esp_hal::parl_io::TxSixteenBits;

#[cfg(not(esp32c5))]
impl<'d> crate::Hub75Pins<'d, TxSixteenBits<'d>> for Hub75Pins16<'d> {
    type Word = u16;

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

impl<'d> crate::Hub75Pins<'d, TxEightBits<'d>> for Hub75Pins8<'d> {
    type Word = u8;

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
