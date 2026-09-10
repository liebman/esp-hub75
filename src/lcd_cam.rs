//! HUB75 driver for `LCD_CAM` peripherals (ESP32-S3).
//!
//! The `lcd_trans_done` interrupt runs the BCM (Binary Code Modulation)
//! refresh loop, so the panel keeps scanning out the current framebuffer on
//! its own. Buffer swaps take effect at frame boundaries.
//!
//! # Blocking example
//!
//! ```rust,ignore
//! let hub75 = Hub75::new(
//!     peripherals.LCD_CAM, pins, peripherals.DMA_CH0,
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
//!     peripherals.LCD_CAM, pins, peripherals.DMA_CH0,
//!     tx_descriptors, Hub75Config::new(Rate::from_mhz(20)), &*fb0,
//! ).expect("failed to create Hub75");
//!
//! // Swap buffers: yields to the executor, returns Err on DMA failure.
//! let old_fb = hub75.swap(fb1)?.wait().expect("DMA error");
//! ```

use esp_hal::Blocking;
use esp_hal::gpio::NoPin;
use esp_hal::lcd_cam::LcdCam;
use esp_hal::lcd_cam::LcdDmaTxChannel;
#[cfg(feature = "invert-clock")]
use esp_hal::lcd_cam::lcd::ClockMode;
#[cfg(feature = "invert-clock")]
use esp_hal::lcd_cam::lcd::Phase;
#[cfg(feature = "invert-clock")]
use esp_hal::lcd_cam::lcd::Polarity;
use esp_hal::lcd_cam::lcd::i8080;
#[cfg(feature = "circular-dma")]
use esp_hal::lcd_cam::lcd::i8080::Command;
use esp_hal::lcd_cam::lcd::i8080::I8080;
use esp_hal::peripherals::LCD_CAM;

use crate::Hub75Config;
use crate::Hub75DmaDescriptors;
use crate::Hub75Error;
use crate::Hub75Pins;
use crate::Hub75Pins8;
use crate::Hub75Pins16;
use crate::isr::BcmBuf;
use crate::framebuffer::WordSize;
pub use crate::isr::Hub75;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

/// Clears the `lcd_trans_done` flag on the `LCD_CAM` peripheral.
///
/// Called by the boundary ISR (circular mode) to drain the handled boundary
/// flag and any stale flag. `wait()` polls the peripheral's state registers
/// rather than this flag, so clearing it before `wait()` is safe.
// SAFETY: The `LCD_CAM` peripheral handle is owned by the driver; this steals
// a second handle only to write the interrupt-clear register from ISR
// context. The write is a single register store, and the driver never touches
// this register concurrently.
#[cfg_attr(not(feature = "circular-dma"), allow(dead_code))]
pub(crate) fn clear_frame_interrupt() {
    unsafe {
        LCD_CAM::steal()
            .register_block()
            .lc_dma_int_clr()
            .write(|w| w.lcd_trans_done_int_clr().set_bit());
    }
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

impl<DM: esp_hal::DriverMode, FB: crate::framebuffer::FrameBuffer + 'static> Hub75<DM, FB> {
    fn new_internal<P: Hub75Pins<'static, Word = FB::Word>, const N: usize>(
        lcd_cam: LCD_CAM<'static>,
        hub75_pins: P,
        channel: impl LcdDmaTxChannel<'static>,
        tx_descriptors: &'static mut Hub75DmaDescriptors<FB, N>,
        config: Hub75Config,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        crate::isr::claim_driver()?;
        crate::bcm::validate_fb_internal_ram(fb);

        let word_size = hub75_pins.word_size();

        let mut lcd_cam_dev = LcdCam::new(lcd_cam);

        // Bind the boundary/refresh ISR to the LCD_CAM peripheral interrupt
        // and enable the `lcd_trans_done` source (both DMA modes), before
        // the first transfer starts; the source stays enabled for the
        // driver's lifetime.
        //
        // - Linear: every chain ends with `suc_eof` + `NULL` next, so `lcd_trans_done`
        //   fires at every segment-group boundary and runs the BCM loop.
        // - Circular: the ring carries no `suc_eof`, so nothing fires in steady state.
        //   A swap relinks the second-to-last ring descriptor to the spare boundary
        //   descriptor (`suc_eof` + `NULL` next) — from the LCD_CAM's point of view
        //   that is a normal end-of-transfer (identical to linear mode), so
        //   `lcd_trans_done` fires at the armed pass boundary and the ISR consumes and
        //   restarts the transfer.
        lcd_cam_dev.set_interrupt_handler(crate::isr::handler_with_priority(
            crate::isr::isr,
            config.interrupt_priority,
        ));
        // SAFETY: The `LCD_CAM` peripheral is owned by `lcd_cam_dev`; this
        // steals a second handle only to enable the `lcd_trans_done` source
        // during construction, while no transfer is running, so there is no
        // concurrent register access.
        unsafe {
            LCD_CAM::steal()
                .register_block()
                .lc_dma_int_ena()
                .modify(|_, w| w.lcd_trans_done_int_ena().set_bit());
        }

        let lcd_config = {
            let c = i8080::Config::default().with_frequency(config.frequency);
            #[cfg(feature = "invert-clock")]
            let c = c.with_clock_mode(ClockMode {
                polarity: Polarity::IdleLow,
                phase: Phase::ShiftHigh,
            });
            c
        };

        let i8080 = I8080::new(lcd_cam_dev.lcd, channel, lcd_config).map_err(Hub75Error::I8080)?;
        let i8080 = hub75_pins.apply(i8080);

        cfg_select! {
            feature = "circular-dma" => {
                let mut buf = BcmBuf::new(tx_descriptors.as_slice(), fb);
                let descriptor_ptr = buf.descriptors_ptr();
                let descriptor_count = buf.descriptor_count();
                let fb_ptr = core::ptr::from_ref(fb).cast::<()>();

                let xfer = match word_size {
                    WordSize::Eight => i8080.send(Command::<u8>::None, 0, buf),
                    WordSize::Sixteen => i8080.send(Command::<u16>::None, 0, buf),
                }
                .map_err(|(err, _tx, _buf)| Hub75Error::Dma(err))?;

                crate::isr::init_state(xfer, descriptor_ptr, descriptor_count, fb_ptr, word_size);
            }
            _ => {
                let buf = BcmBuf::new(tx_descriptors.as_slice());
                crate::isr::init_state(i8080, buf, word_size);
                crate::isr::start_internal(fb)?;
            }
        }

        Ok(Self::from_phantom())
    }
}

impl<FB: crate::framebuffer::FrameBuffer + 'static> Hub75<Blocking, FB> {
    /// Create a new blocking HUB75 driver.
    ///
    /// Configures the `LCD_CAM` peripheral, applies pin assignments, and
    /// immediately starts DMA-driven display refresh with the provided
    /// framebuffer.
    ///
    /// The pin configuration's word type must match the framebuffer's word
    /// type; passing a 16-bit framebuffer with 8-bit pins (or vice versa)
    /// is a compile-time error.
    ///
    /// # Arguments
    /// * `lcd_cam` -- The `LCD_CAM` peripheral instance
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `config` -- `LCD_CAM` clock rate
    /// * `fb` -- Initial framebuffer to display
    /// # Errors
    ///
    /// Returns [`Hub75Error::AlreadyInitialised`] if a `Hub75` instance
    /// already exists. Returns [`Hub75Error::AlreadyRunning`],
    /// [`Hub75Error::Dma`], or [`Hub75Error::I8080`](crate::Hub75Error::I8080)
    /// if the initial DMA transfer fails.
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new<P: Hub75Pins<'static, Word = FB::Word>, const N: usize>(
        lcd_cam: LCD_CAM<'static>,
        hub75_pins: P,
        channel: impl LcdDmaTxChannel<'static>,
        tx_descriptors: &'static mut Hub75DmaDescriptors<FB, N>,
        config: Hub75Config,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(lcd_cam, hub75_pins, channel, tx_descriptors, config, fb)
    }
}

impl<FB: crate::framebuffer::FrameBuffer + 'static> Hub75<esp_hal::Async, FB> {
    /// Create a new async HUB75 driver.
    ///
    /// Configures the `LCD_CAM` peripheral, applies pin assignments, and
    /// immediately starts DMA-driven display refresh with the provided
    /// framebuffer.
    ///
    /// The pin configuration's word type must match the framebuffer's word
    /// type; passing a 16-bit framebuffer with 8-bit pins (or vice versa)
    /// is a compile-time error.
    ///
    /// # Arguments
    /// * `lcd_cam` -- The `LCD_CAM` peripheral instance
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `config` -- `LCD_CAM` clock rate
    /// * `fb` -- Initial framebuffer to display
    /// # Errors
    ///
    /// Returns [`Hub75Error::AlreadyInitialised`] if a `Hub75` instance
    /// already exists. Returns [`Hub75Error::AlreadyRunning`],
    /// [`Hub75Error::Dma`], or [`Hub75Error::I8080`](crate::Hub75Error::I8080)
    /// if the initial DMA transfer fails.
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new_async<P: Hub75Pins<'static, Word = FB::Word>, const N: usize>(
        lcd_cam: LCD_CAM<'static>,
        hub75_pins: P,
        channel: impl LcdDmaTxChannel<'static>,
        tx_descriptors: &'static mut Hub75DmaDescriptors<FB, N>,
        config: Hub75Config,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(lcd_cam, hub75_pins, channel, tx_descriptors, config, fb)
    }
}

// ---------------------------------------------------------------------------
// Pin configurations
// ---------------------------------------------------------------------------

impl<'d> crate::Hub75Pins<'d> for Hub75Pins16<'d> {
    type Word = u16;

    fn word_size(&self) -> WordSize {
        WordSize::Sixteen
    }

    fn apply<DM: esp_hal::DriverMode>(self, i8080: I8080<'d, DM>) -> I8080<'d, DM> {
        let blank = self.blank.into_output_signal();
        #[cfg(feature = "invert-blank")]
        let blank = blank.with_output_inverter(true);

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
            .with_data8(blank)
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
    type Word = u8;

    fn word_size(&self) -> WordSize {
        WordSize::Eight
    }

    fn apply<DM: esp_hal::DriverMode>(self, i8080: I8080<'d, DM>) -> I8080<'d, DM> {
        let blank = self.blank.into_output_signal();
        #[cfg(feature = "invert-blank")]
        let blank = blank.with_output_inverter(true);
        // SAFETY: same split as `blank` above; output half only.
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
