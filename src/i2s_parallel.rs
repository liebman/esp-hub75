//! HUB75 driver for I2S Parallel peripherals (ESP32).
//!
//! This module provides an interrupt-driven display controller that
//! continuously refreshes a HUB75 panel from a framebuffer. The I2S DMA
//! `out_total_eof` interrupt drives the entire BCM (Binary Code Modulation)
//! refresh loop. Buffer swaps happen atomically at frame boundaries.
//!
//! # Blocking example
//!
//! ```rust,ignore
//! let hub75 = Hub75::new(
//!     peripherals.I2S0, pins, peripherals.DMA_I2S0,
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
//!     peripherals.I2S0, pins, peripherals.DMA_I2S0,
//!     tx_descriptors, Rate::from_mhz(20),
//! ).expect("failed to create Hub75");
//! hub75.start(&*fb0).expect("failed to start Hub75");
//!
//! // Swap buffers — yields to the executor, returns Err on DMA failure.
//! let old_ptr = hub75.swap(&*fb1).await.expect("DMA error");
//! ```

use esp_hal::dma::DmaChannelFor;
use esp_hal::dma::DmaDescriptor;
use esp_hal::gpio::AnyPin;
use esp_hal::gpio::NoPin;
use esp_hal::i2s::parallel::I2sParallel;
use esp_hal::i2s::parallel::TxEightBits;
use esp_hal::i2s::parallel::TxPins;
use esp_hal::i2s::parallel::TxSixteenBits;
use esp_hal::i2s::AnyI2s;
use esp_hal::peripherals::Interrupt;
use esp_hal::time::Rate;
use esp_hal::Blocking;

use crate::bcm_buf::BcmBuf;
pub use crate::isr::Hub75;
use crate::Hub75Error;
use crate::Hub75Pins;
use crate::Hub75Pins16;
use crate::Hub75Pins8;

// ---------------------------------------------------------------------------
// I2S instance trait — maps concrete peripherals to their interrupt and
// register block for enabling `out_total_eof`.
// ---------------------------------------------------------------------------

/// Helper trait implemented for I2S0 and I2S1 to provide interrupt binding
/// and `out_total_eof` enable. Users don't need to interact with this trait
/// directly — just pass `peripherals.I2S0` or `peripherals.I2S1` to the
/// constructor.
pub trait I2sHub75Instance: esp_hal::i2s::parallel::Instance {
    #[doc(hidden)]
    fn bind_and_enable_isr();
}

impl I2sHub75Instance for esp_hal::peripherals::I2S0<'_> {
    fn bind_and_enable_isr() {
        unsafe {
            esp_hal::interrupt::bind_handler(Interrupt::I2S0, crate::isr::hub75_isr);
            let stolen = esp_hal::peripherals::I2S0::steal();
            stolen
                .register_block()
                .int_ena()
                .modify(|_, w| w.out_total_eof().set_bit());
        }
    }
}

impl I2sHub75Instance for esp_hal::peripherals::I2S1<'_> {
    fn bind_and_enable_isr() {
        unsafe {
            esp_hal::interrupt::bind_handler(Interrupt::I2S1, crate::isr::hub75_isr);
            let stolen = esp_hal::peripherals::I2S1::steal();
            stolen
                .register_block()
                .int_ena()
                .modify(|_, w| w.out_total_eof().set_bit());
        }
    }
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

impl<DM: esp_hal::DriverMode, FB: crate::framebuffer::FrameBuffer + 'static> Hub75<DM, FB> {
    fn new_internal<
        T: TxPins<'static> + 'static,
        P: Hub75Pins<'static, T, Word = FB::Word>,
        I: I2sHub75Instance + 'static,
    >(
        i2s: I,
        hub75_pins: P,
        channel: impl DmaChannelFor<AnyI2s<'static>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        let (pins, clock_pin) = hub75_pins.convert_pins();

        // By default data changes on the falling edge of CLK so it is stable
        // when the panel latches on the rising edge. The ESP32 I2S peripheral
        // shifts on the rising edge, so we invert the clock output unless the
        // user opted into the opposite polarity.
        #[cfg(not(feature = "invert-clock"))]
        let clock_pin = clock_pin.into_output_signal().with_output_inverter(true);

        let i2s_parallel = I2sParallel::new(i2s, channel, frequency, pins, clock_pin);

        // Bind our ISR and enable the out_total_eof interrupt so we get
        // notified on each DMA transfer completion.
        I::bind_and_enable_isr();

        let buf = BcmBuf::new(tx_descriptors);
        crate::isr::init_isr_state(i2s_parallel, buf);
        crate::isr::start_internal(fb)?;

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
    /// type — passing a 16-bit framebuffer with 8-bit pins (or vice versa)
    /// is a compile-time error.
    ///
    /// # Arguments
    /// * `i2s` -- The I2S peripheral instance (I2S0 or I2S1)
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel (DMA_I2S0 or DMA_I2S1)
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `frequency` -- I2S clock rate
    /// * `fb` -- Initial framebuffer to display
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new<
        T: TxPins<'static> + 'static,
        P: Hub75Pins<'static, T, Word = FB::Word>,
        I: I2sHub75Instance + 'static,
    >(
        i2s: I,
        hub75_pins: P,
        channel: impl DmaChannelFor<AnyI2s<'static>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(i2s, hub75_pins, channel, tx_descriptors, frequency, fb)
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
    /// type — passing a 16-bit framebuffer with 8-bit pins (or vice versa)
    /// is a compile-time error.
    ///
    /// # Arguments
    /// * `i2s` -- The I2S peripheral instance (I2S0 or I2S1)
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel (DMA_I2S0 or DMA_I2S1)
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `frequency` -- I2S clock rate
    /// * `fb` -- Initial framebuffer to display
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new_async<
        T: TxPins<'static> + 'static,
        P: Hub75Pins<'static, T, Word = FB::Word>,
        I: I2sHub75Instance + 'static,
    >(
        i2s: I,
        hub75_pins: P,
        channel: impl DmaChannelFor<AnyI2s<'static>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(i2s, hub75_pins, channel, tx_descriptors, frequency, fb)
    }
}

// ---------------------------------------------------------------------------
// Pin configurations
// ---------------------------------------------------------------------------

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
