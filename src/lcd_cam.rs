//! HUB75 driver for LCD_CAM peripherals (ESP32-S3).
//!
//! This module provides an interrupt-driven display controller that
//! continuously refreshes a HUB75 panel from a framebuffer. The LCD_CAM
//! `lcd_trans_done` interrupt drives the entire BCM (Binary Code Modulation)
//! refresh loop. Buffer swaps happen atomically at frame boundaries.
//!
//! # Blocking example
//!
//! ```rust,ignore
//! let hub75 = Hub75::new(
//!     peripherals.LCD_CAM, pins, peripherals.DMA_CH0,
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
//!     peripherals.LCD_CAM, pins, peripherals.DMA_CH0,
//!     tx_descriptors, Rate::from_mhz(20),
//! ).expect("failed to create Hub75");
//! hub75.start(&*fb0).expect("failed to start Hub75");
//!
//! // Swap buffers — yields to the executor, returns Err on DMA failure.
//! let old_ptr = hub75.swap(&*fb1).await.expect("DMA error");
//! ```

use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::TxChannelFor;
use esp_hal::gpio::NoPin;
use esp_hal::lcd_cam::lcd::i8080;
use esp_hal::lcd_cam::lcd::i8080::I8080;
#[cfg(feature = "invert-clock")]
use esp_hal::lcd_cam::lcd::ClockMode;
#[cfg(feature = "invert-clock")]
use esp_hal::lcd_cam::lcd::Phase;
#[cfg(feature = "invert-clock")]
use esp_hal::lcd_cam::lcd::Polarity;
use esp_hal::lcd_cam::LcdCam;
use esp_hal::peripherals::LCD_CAM;
use esp_hal::time::Rate;
use esp_hal::Blocking;

use crate::bcm_buf::BcmBuf;
use crate::framebuffer::WordSize;
pub use crate::isr::Hub75;
use crate::Hub75Error;
use crate::Hub75Pins;
use crate::Hub75Pins16;
use crate::Hub75Pins8;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

impl<DM: esp_hal::DriverMode> Hub75<DM> {
    fn new_internal(
        lcd_cam: LCD_CAM<'static>,
        hub75_pins: impl Hub75Pins<'static>,
        channel: impl TxChannelFor<LCD_CAM<'static>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
    ) -> Result<Self, Hub75Error> {
        let word_size = hub75_pins.word_size();

        let mut lcd_cam_dev = LcdCam::new(lcd_cam);
        lcd_cam_dev.set_interrupt_handler(crate::isr::hub75_isr);

        #[allow(unused_mut)]
        let mut config = i8080::Config::default().with_frequency(frequency);
        #[cfg(feature = "invert-clock")]
        let config = config.with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftHigh,
        });

        let i8080 = I8080::new(lcd_cam_dev.lcd, channel, config).map_err(Hub75Error::I8080)?;
        let i8080 = hub75_pins.apply(i8080);

        // Enable the LCD done interrupt at the peripheral level so the ISR
        // fires on every transfer completion.
        unsafe {
            let stolen = LCD_CAM::steal();
            stolen
                .register_block()
                .lc_dma_int_ena()
                .modify(|_, w| w.lcd_trans_done_int_ena().set_bit());
        }

        let buf = BcmBuf::new(tx_descriptors);
        crate::isr::init_isr_state(i8080, buf, word_size);

        Ok(Self::from_phantom())
    }
}

impl Hub75<Blocking> {
    /// Create a new blocking HUB75 driver.
    ///
    /// The driver is created in an idle state. Call [`Hub75::start`] with a
    /// framebuffer to begin display refresh.
    ///
    /// # Arguments
    /// * `lcd_cam` -- The LCD_CAM peripheral instance
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `frequency` -- LCD_CAM clock rate
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new(
        lcd_cam: LCD_CAM<'static>,
        hub75_pins: impl Hub75Pins<'static>,
        channel: impl TxChannelFor<LCD_CAM<'static>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(lcd_cam, hub75_pins, channel, tx_descriptors, frequency)
    }
}

impl Hub75<esp_hal::Async> {
    /// Create a new async HUB75 driver.
    ///
    /// The driver is created in an idle state. Call [`Hub75::start`] with a
    /// framebuffer to begin display refresh.
    ///
    /// # Arguments
    /// * `lcd_cam` -- The LCD_CAM peripheral instance
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `frequency` -- LCD_CAM clock rate
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new_async(
        lcd_cam: LCD_CAM<'static>,
        hub75_pins: impl Hub75Pins<'static>,
        channel: impl TxChannelFor<LCD_CAM<'static>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(lcd_cam, hub75_pins, channel, tx_descriptors, frequency)
    }
}

// ---------------------------------------------------------------------------
// Pin configurations
// ---------------------------------------------------------------------------

impl<'d> crate::Hub75Pins<'d> for Hub75Pins16<'d> {
    fn word_size(&self) -> WordSize {
        WordSize::Sixteen
    }

    fn apply<DM: esp_hal::DriverMode>(self, i8080: I8080<'d, DM>) -> I8080<'d, DM> {
        // SAFETY: We only use the output signal half. The original `AnyPin` is
        // consumed by the enclosing struct move, so there is no aliased access.
        let (_, blank) = unsafe { self.blank.split() };

        i8080
            .with_wrx(self.clock)
            .with_data0(self.addr0)
            .with_data1(self.addr1)
            .with_data2(self.addr2)
            .with_data3(self.addr3)
            .with_data4(self.addr4)
            .with_data5(self.latch)
            .with_data6(NoPin)
            .with_data7(NoPin)
            .with_data8(blank.with_output_inverter(true))
            .with_data9(self.red1)
            .with_data10(self.grn1)
            .with_data11(self.blu1)
            .with_data12(self.red2)
            .with_data13(self.grn2)
            .with_data14(self.blu2)
            .with_data15(NoPin)
    }
}

impl<'d> crate::Hub75Pins<'d> for Hub75Pins8<'d> {
    fn word_size(&self) -> WordSize {
        WordSize::Eight
    }

    fn apply<DM: esp_hal::DriverMode>(self, i8080: I8080<'d, DM>) -> I8080<'d, DM> {
        // SAFETY: We only use the output signal half of each pin. The original
        // `AnyPin` values are consumed by the enclosing struct move, so there
        // is no aliased access.
        let (_, blank) = unsafe { self.blank.split() };
        #[cfg(feature = "invert-blank")]
        let blank = blank.with_output_inverter(true);
        let (_, clock) = unsafe { self.clock.split() };
        #[cfg(feature = "invert-clock")]
        let clock = clock.with_output_inverter(true);
        i8080
            .with_wrx(clock)
            .with_data0(self.red1)
            .with_data1(self.grn1)
            .with_data2(self.blu1)
            .with_data3(self.red2)
            .with_data4(self.grn2)
            .with_data5(self.blu2)
            .with_data6(self.latch)
            .with_data7(blank)
    }
}
