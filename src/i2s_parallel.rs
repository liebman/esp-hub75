//! HUB75 driver for I2S Parallel peripherals (ESP32).
//!
//! The I2S DMA `out_total_eof` interrupt runs the BCM loop . In
//! circular-DMA mode, the per-descriptor `out_eof` interrupt runs the
//! loop. The loop sends the framebuffer data to the panel again and
//! again. The panel shows the current framebuffer by itself. A buffer
//! swap starts at a frame boundary.
//!
//! # Blocking example
//!
//! ```rust,ignore
//! let hub75 = Hub75::new(
//!     peripherals.I2S0, pins, peripherals.DMA_I2S0,
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
//!     peripherals.I2S0, pins, peripherals.DMA_I2S0,
//!     tx_descriptors, Hub75Config::new(Rate::from_mhz(20)), &*fb0,
//! ).expect("failed to create Hub75");
//!
//! // Swap buffers: yields to the executor, returns Err on DMA failure.
//! let old_fb = hub75.swap(fb1)?.wait().expect("DMA error");
//! ```

use esp_hal::Blocking;
use esp_hal::gpio::AnyPin;
use esp_hal::gpio::NoPin;
use esp_hal::i2s::parallel::I2sParallel;
use esp_hal::i2s::parallel::I2sParallelDmaChannel;
use esp_hal::i2s::parallel::I2sParallelInterrupt;
use esp_hal::i2s::parallel::Instance;
use esp_hal::i2s::parallel::TxEightBits;
use esp_hal::i2s::parallel::TxPins;
use esp_hal::i2s::parallel::TxSixteenBits;

use crate::Hub75Config;
use crate::Hub75DmaDescriptors;
use crate::Hub75Error;
use crate::Hub75Pins;
use crate::Hub75Pins8;
use crate::Hub75Pins16;
#[cfg(feature = "circular-dma")]
use crate::bcm::circular::CircularBcmBuf;
#[cfg(not(feature = "circular-dma"))]
use crate::bcm::linear::BcmBuf;
pub use crate::isr::Hub75;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

#[cfg(not(feature = "circular-dma"))]
impl<DM: esp_hal::DriverMode, FB: crate::framebuffer::FrameBuffer + 'static> Hub75<DM, FB> {
    fn new_internal<
        T: TxPins<'static> + 'static,
        P: Hub75Pins<'static, T, Word = FB::Word>,
        I: Instance + 'static,
        const N: usize,
    >(
        i2s: I,
        hub75_pins: P,
        channel: impl I2sParallelDmaChannel<'static, I>,
        tx_descriptors: &'static mut Hub75DmaDescriptors<FB, N>,
        config: Hub75Config,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        crate::isr::claim_driver()?;

        let (pins, clock_pin) = hub75_pins.convert_pins();

        // By default data changes on the falling edge of CLK so it is stable
        // when the panel latches on the rising edge. The ESP32 I2S peripheral
        // shifts on the rising edge, so we invert the clock output unless the
        // user opted into the opposite polarity.
        #[cfg(not(feature = "invert-clock"))]
        let clock_pin = clock_pin.into_output_signal().with_output_inverter(true);

        let mut i2s_parallel = I2sParallel::new(i2s, channel, config.frequency, pins, clock_pin);

        // This connects `hub75_isr` to the interrupt and turns on the
        // `out_total_eof` source. The `out_total_eof` interrupt occurs when
        // the DMA finishes the full DMA descriptor chain.
        i2s_parallel.set_interrupt_handler(crate::isr::handler_with_priority(
            crate::isr::hub75_isr,
            config.interrupt_priority,
        ));
        i2s_parallel.listen(I2sParallelInterrupt::TotalEof);

        let buf = BcmBuf::new(tx_descriptors.as_slice());
        crate::isr::init_isr_state(i2s_parallel, buf);
        crate::isr::start_internal(fb)?;

        Ok(Self::from_phantom())
    }
}

#[cfg(feature = "circular-dma")]
impl<DM: esp_hal::DriverMode, FB: crate::framebuffer::FrameBuffer + 'static> Hub75<DM, FB> {
    fn new_internal<
        T: TxPins<'static> + 'static,
        P: Hub75Pins<'static, T, Word = FB::Word>,
        I: Instance + 'static,
        const N: usize,
    >(
        i2s: I,
        hub75_pins: P,
        channel: impl I2sParallelDmaChannel<'static, I>,
        tx_descriptors: &'static mut Hub75DmaDescriptors<FB, N>,
        config: Hub75Config,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        crate::isr::claim_driver()?;
        crate::bcm::validate_fb_internal_ram(fb);

        let (pins, clock_pin) = hub75_pins.convert_pins();

        // By default data changes on the falling edge of CLK so it is stable
        // when the panel latches on the rising edge. The ESP32 I2S peripheral
        // shifts on the rising edge, so we invert the clock output unless the
        // user opted into the opposite polarity.
        #[cfg(not(feature = "invert-clock"))]
        let clock_pin = clock_pin.into_output_signal().with_output_inverter(true);

        let mut i2s_parallel = I2sParallel::new(i2s, channel, config.frequency, pins, clock_pin);

        // This connects `hub75_boundary_isr` to the interrupt and turns
        // on the per-descriptor `out_eof` source. The `out_eof` interrupt
        // occurs each time the DMA finds a descriptor with `suc_eof=1`.
        // This is also true for a circular chain , where `next` points
        // back to the start of the chain. We checked this on the hardware
        // (ESP32 classic, I2S LCD mode). `out_total_eof` never occurs on a
        // circular chain, because the transfer never ends.
        i2s_parallel.set_interrupt_handler(crate::isr::handler_with_priority(
            crate::isr::hub75_boundary_isr,
            config.interrupt_priority,
        ));
        i2s_parallel.listen(I2sParallelInterrupt::Eof);

        let mut buf = CircularBcmBuf::new(tx_descriptors.as_slice(), fb);
        let desc_ptr = buf.descriptors_ptr();
        let desc_count = buf.desc_count();
        let fb_ptr = core::ptr::from_ref(fb).cast::<()>();

        let xfer = i2s_parallel
            .send(buf)
            .map_err(|(err, _tx, _buf)| Hub75Error::Dma(err))?;

        crate::isr::store_circular_state(xfer, desc_ptr, desc_count, fb_ptr);

        Ok(Self::from_phantom())
    }
}

impl<FB: crate::framebuffer::FrameBuffer + 'static> Hub75<Blocking, FB> {
    /// Create a new blocking HUB75 driver.
    ///
    /// Configures the I2S peripheral, applies pin assignments, and
    /// immediately starts DMA-driven display refresh with the provided
    /// framebuffer.
    ///
    /// The pin configuration's word type must match the framebuffer's word
    /// type; passing a 16-bit framebuffer with 8-bit pins (or vice versa)
    /// is a compile-time error.
    ///
    /// # Arguments
    /// * `i2s` -- The I2S peripheral instance (I2S0 or I2S1)
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel (`DMA_I2S0` or `DMA_I2S1`)
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `config` -- I2S clock rate
    /// * `fb` -- Initial framebuffer to display
    /// # Errors
    ///
    /// Returns [`Hub75Error::AlreadyInitialised`] if a `Hub75` instance
    /// already exists. Returns [`Hub75Error::AlreadyRunning`] or
    /// [`Hub75Error::Dma`] if the initial DMA transfer fails.
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new<
        T: TxPins<'static> + 'static,
        P: Hub75Pins<'static, T, Word = FB::Word>,
        I: Instance + 'static,
        const N: usize,
    >(
        i2s: I,
        hub75_pins: P,
        channel: impl I2sParallelDmaChannel<'static, I>,
        tx_descriptors: &'static mut Hub75DmaDescriptors<FB, N>,
        config: Hub75Config,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(i2s, hub75_pins, channel, tx_descriptors, config, fb)
    }
}

impl<FB: crate::framebuffer::FrameBuffer + 'static> Hub75<esp_hal::Async, FB> {
    /// Create a new async HUB75 driver.
    ///
    /// Configures the I2S peripheral, applies pin assignments, and
    /// immediately starts DMA-driven display refresh with the provided
    /// framebuffer.
    ///
    /// The pin configuration's word type must match the framebuffer's word
    /// type; passing a 16-bit framebuffer with 8-bit pins (or vice versa)
    /// is a compile-time error.
    ///
    /// # Arguments
    /// * `i2s` -- The I2S peripheral instance (I2S0 or I2S1)
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel (`DMA_I2S0` or `DMA_I2S1`)
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `config` -- I2S clock rate
    /// * `fb` -- Initial framebuffer to display
    /// # Errors
    ///
    /// Returns [`Hub75Error::AlreadyInitialised`] if a `Hub75` instance
    /// already exists. Returns [`Hub75Error::AlreadyRunning`] or
    /// [`Hub75Error::Dma`] if the initial DMA transfer fails.
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new_async<
        T: TxPins<'static> + 'static,
        P: Hub75Pins<'static, T, Word = FB::Word>,
        I: Instance + 'static,
        const N: usize,
    >(
        i2s: I,
        hub75_pins: P,
        channel: impl I2sParallelDmaChannel<'static, I>,
        tx_descriptors: &'static mut Hub75DmaDescriptors<FB, N>,
        config: Hub75Config,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(i2s, hub75_pins, channel, tx_descriptors, config, fb)
    }
}

// ---------------------------------------------------------------------------
// Pin configurations
// ---------------------------------------------------------------------------

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
