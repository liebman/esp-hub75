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
//!     tx_descriptors, Hub75Config::new(Rate::from_mhz(20)), &*fb,
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
//!     tx_descriptors, Hub75Config::new(Rate::from_mhz(20)), &*fb0,
//! ).expect("failed to create Hub75");
//!
//! // Swap buffers: yields to the executor, returns Err on DMA failure.
//! let old_fb = hub75.swap(fb1)?.wait().expect("DMA error");
//! ```

use esp_hal::Blocking;
use esp_hal::parl_io::BitPackOrder;
use esp_hal::parl_io::ClkOutPin;
use esp_hal::parl_io::ConfigurePins;
use esp_hal::parl_io::ParlIo;
use esp_hal::parl_io::ParlIoDmaChannel;
#[cfg(not(feature = "circular-dma"))]
use esp_hal::parl_io::ParlIoInterrupt;
use esp_hal::parl_io::SampleEdge;
use esp_hal::parl_io::TxConfig;
#[cfg(esp32c5)]
use esp_hal::parl_io::TxEofSource;
use esp_hal::parl_io::TxPins;
use esp_hal::peripherals::PARL_IO;

use crate::Hub75Config;
use crate::Hub75DmaDescriptors;
use crate::Hub75Error;
use crate::Hub75Pins;
use crate::Hub75Pins8;
#[cfg(not(esp32c5))]
use crate::Hub75Pins16;
#[cfg(feature = "circular-dma")]
use crate::bcm::circular::CircularBcmBuf;
#[cfg(not(feature = "circular-dma"))]
use crate::bcm::linear::BcmBuf;
pub use crate::isr::Hub75;
#[cfg(feature = "circular-dma")]
use crate::isr::PARL_IO_DUMMY_TRANSFER_LEN;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

impl<DM: esp_hal::DriverMode, FB: crate::framebuffer::FrameBuffer + 'static> Hub75<DM, FB> {
    #[cfg(not(feature = "circular-dma"))]
    fn new_internal<
        T: TxPins + ConfigurePins + 'static,
        P: Hub75Pins<'static, T, Word = FB::Word>,
        const N: usize,
    >(
        parl_io: PARL_IO<'static>,
        hub75_pins: P,
        channel: impl ParlIoDmaChannel<'static>,
        tx_descriptors: &'static mut Hub75DmaDescriptors<FB, N>,
        config: Hub75Config,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        crate::isr::claim_driver()?;

        let (pins, clock_pin) = hub75_pins.convert_pins();

        let mut parl_io_dev = ParlIo::new(parl_io, channel)?;

        parl_io_dev.set_interrupt_handler(crate::isr::handler_with_priority(
            crate::isr::hub75_isr,
            config.interrupt_priority,
        ));
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

        // On the C5 the TX EOF must come from the DMA channel rather than the
        // peripheral's bit-length counter, or the refresh-loop ISR never fires.
        #[cfg(esp32c5)]
        let tx_config = tx_config.with_eof_source(TxEofSource::DmaEof);

        let clk_pin = ClkOutPin::new(clock_pin);
        let parl_io_tx = parl_io_dev.tx.with_config(pins, clk_pin, tx_config)?;

        let buf = BcmBuf::new(tx_descriptors.as_slice());
        crate::isr::init_isr_state(parl_io_tx, buf);
        crate::isr::start_internal(fb)?;

        Ok(Self::from_phantom())
    }

    /// Circular-DMA constructor (ESP32-C5 only).
    ///
    /// The DMA engine is started once with a looping, `suc_eof`-free
    /// descriptor chain and never stopped; in steady state no interrupts are
    /// enabled. A swap arms the boundary detector (`suc_eof` on the last
    /// descriptor + the `PARL_IO` `TxEof` interrupt); the consumed `suc_eof`
    /// halts the DMA channel, so the ISR restarts the transfer after
    /// applying the pending buffer delta (see
    /// [`crate::isr::hub75_boundary_isr`]).
    ///
    /// The ISR must be bound before the first transfer starts so it can
    /// never fire without state to service; `store_circular_state` enables
    /// the interrupt source only once the transfer is stored there.
    #[cfg(feature = "circular-dma")]
    fn new_internal<
        T: TxPins + ConfigurePins + 'static,
        P: Hub75Pins<'static, T, Word = FB::Word>,
        const N: usize,
    >(
        parl_io: PARL_IO<'static>,
        hub75_pins: P,
        channel: impl ParlIoDmaChannel<'static>,
        tx_descriptors: &'static mut Hub75DmaDescriptors<FB, N>,
        config: Hub75Config,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        crate::isr::claim_driver()?;
        crate::bcm::validate_fb_internal_ram(fb);

        let (pins, clock_pin) = hub75_pins.convert_pins();

        let mut parl_io_dev = ParlIo::new(parl_io, channel)?;

        // Bind the unified swap-boundary ISR to the `PARL_IO` interrupt.
        // Binding unlistens from and clears all `PARL_IO` interrupt sources,
        // so nothing fires until a swap arms the `TxEof` source.
        parl_io_dev.set_interrupt_handler(crate::isr::handler_with_priority(
            crate::isr::hub75_boundary_isr,
            config.interrupt_priority,
        ));

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

        // On the C5 the TX EOF must come from the DMA channel rather than the
        // peripheral's bit-length counter. With `DmaEof` and a transfer
        // length of 0 the frame ends at the armed `suc_eof` descriptor
        // regardless of its size.
        #[cfg(esp32c5)]
        let tx_config = tx_config.with_eof_source(TxEofSource::DmaEof);

        let clk_pin = ClkOutPin::new(clock_pin);
        let parl_io_tx = parl_io_dev.tx.with_config(pins, clk_pin, tx_config)?;

        let mut buf = CircularBcmBuf::new(tx_descriptors.as_slice(), fb);
        let desc_ptr = buf.descriptors_ptr();
        let desc_count = buf.desc_count();
        let fb_ptr = core::ptr::from_ref(fb).cast::<()>();

        let xfer = parl_io_tx
            .write(PARL_IO_DUMMY_TRANSFER_LEN, buf)
            .map_err(|(err, _tx, _buf)| Hub75Error::ParlIo(err))?;

        crate::isr::store_circular_state(xfer, desc_ptr, desc_count, fb_ptr);

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
    /// * `config` -- `PARL_IO` clock rate and options
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
    pub fn new<
        T: TxPins + ConfigurePins + 'static,
        P: Hub75Pins<'static, T, Word = FB::Word>,
        const N: usize,
    >(
        parl_io: PARL_IO<'static>,
        hub75_pins: P,
        channel: impl ParlIoDmaChannel<'static>,
        tx_descriptors: &'static mut Hub75DmaDescriptors<FB, N>,
        config: Hub75Config,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(parl_io, hub75_pins, channel, tx_descriptors, config, fb)
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
    /// * `config` -- `PARL_IO` clock rate and options
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
        const N: usize,
    >(
        parl_io: PARL_IO<'static>,
        hub75_pins: P,
        channel: impl ParlIoDmaChannel<'static>,
        tx_descriptors: &'static mut Hub75DmaDescriptors<FB, N>,
        config: Hub75Config,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(parl_io, hub75_pins, channel, tx_descriptors, config, fb)
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
