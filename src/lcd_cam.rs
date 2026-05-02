use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaError;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::dma::TxChannelFor;
use esp_hal::gpio::NoPin;
use esp_hal::lcd_cam::lcd::i8080;
use esp_hal::lcd_cam::lcd::i8080::Command;
use esp_hal::lcd_cam::lcd::i8080::I8080Transfer;
use esp_hal::lcd_cam::lcd::i8080::I8080;
#[cfg(feature = "invert-clock")]
use esp_hal::lcd_cam::lcd::ClockMode;
#[cfg(feature = "invert-clock")]
use esp_hal::lcd_cam::lcd::Phase;
#[cfg(feature = "invert-clock")]
use esp_hal::lcd_cam::lcd::Polarity;
use esp_hal::lcd_cam::LcdCam;
use esp_hal::peripherals::LCD_CAM;
#[cfg(feature = "iram")]
use esp_hal::ram;
use esp_hal::time::Rate;

use crate::framebuffer::FrameBuffer;
use crate::framebuffer::WordSize;
use crate::Hub75Error;
use crate::Hub75Pins;
use crate::Hub75Pins16;
use crate::Hub75Pins8;

/// HUB75 LED matrix display driver using LCD/CAM peripheral
///
/// This driver uses the ESP32's LCD/CAM peripheral in I8080 mode to drive HUB75
/// LED matrix displays. It supports both 8-bit and 16-bit configurations and
/// can operate in either blocking or async mode.
pub struct Hub75<'d, DM: esp_hal::DriverMode> {
    i8080: I8080<'d, DM>,
    tx_descriptors: &'static mut [DmaDescriptor],
}

impl<'d> Hub75<'d, esp_hal::Blocking> {
    /// Creates a new blocking HUB75 driver instance
    ///
    /// # Arguments
    /// * `lcd_cam` - The LCD/CAM peripheral instance
    /// * `hub75_pins` - The HUB75 pin configuration
    /// * `channel` - The DMA channel to use for transfers
    /// * `tx_descriptors` - DMA descriptors for the transfer buffer
    /// * `frequency` - The clock frequency for the display
    ///
    /// # Returns
    /// A new `Hub75` instance configured for blocking operation
    ///
    /// # Errors
    /// Returns an error if the peripheral cannot be configured
    pub fn new(
        lcd_cam: LCD_CAM<'d>,
        hub75_pins: impl Hub75Pins<'d>,
        channel: impl TxChannelFor<LCD_CAM<'d>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
    ) -> Result<Self, Hub75Error> {
        let lcd_cam = LcdCam::new(lcd_cam);
        Self::new_internal(lcd_cam, hub75_pins, channel, tx_descriptors, frequency)
    }
}

impl<'d> Hub75<'d, esp_hal::Async> {
    /// Creates a new async HUB75 driver instance
    ///
    /// # Arguments
    /// * `lcd_cam` - The LCD/CAM peripheral instance
    /// * `hub75_pins` - The HUB75 pin configuration
    /// * `channel` - The DMA channel to use for transfers
    /// * `tx_descriptors` - DMA descriptors for the transfer buffer
    /// * `frequency` - The clock frequency for the display
    ///
    /// # Returns
    /// A new `Hub75` instance configured for async operation
    ///
    /// # Errors
    /// Returns an error if the peripheral cannot be configured
    pub fn new_async(
        lcd_cam: LCD_CAM<'d>,
        hub75_pins: impl Hub75Pins<'d>,
        channel: impl TxChannelFor<LCD_CAM<'d>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
    ) -> Result<Self, Hub75Error> {
        let lcd_cam = LcdCam::new(lcd_cam).into_async();
        Self::new_internal(lcd_cam, hub75_pins, channel, tx_descriptors, frequency)
    }
}

impl<'d, DM: esp_hal::DriverMode> Hub75<'d, DM> {
    fn new_internal(
        lcd_cam: LcdCam<'d, DM>,
        hub75_pins: impl Hub75Pins<'d>,
        channel: impl TxChannelFor<LCD_CAM<'d>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
    ) -> Result<Self, Hub75Error> {
        #[allow(unused_mut)]
        let mut config = i8080::Config::default().with_frequency(frequency);
        #[cfg(feature = "invert-clock")]
        let config = config.with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftHigh,
        });
        let i8080 = I8080::new(lcd_cam.lcd, channel, config).map_err(Hub75Error::I8080)?;
        let i8080 = hub75_pins.apply(i8080);
        Ok(Self {
            i8080,
            tx_descriptors,
        })
    }

    /// Renders a frame buffer to the display.
    ///
    /// This method handles both contiguous (single-plane) and bitplane
    /// (multi-plane BCM-weighted) framebuffers automatically. The DMA
    /// descriptor chain is built internally — callers never need to manage
    /// DMA buffer types.
    ///
    /// Calling render consumes the `Hub75` instance and returns a
    /// `Hub75Transfer` instance that can be used to wait for the transfer
    /// to complete.  After the transfer is complete, the `Hub75` will be
    /// returned from the `wait()` method on the `Hub75Transfer` instance.
    ///
    /// # Arguments
    /// * `fb` - The frame buffer to render
    ///
    /// # Returns
    /// A `Hub75Transfer` instance that can be used to wait for the transfer to
    /// complete
    ///
    /// # Errors
    /// Returns a tuple of `Hub75Error` and the `Hub75` instance if the transfer
    /// cannot be started
    #[cfg_attr(feature = "iram", ram)]
    pub fn render(
        self,
        fb: &impl FrameBuffer,
    ) -> Result<Hub75Transfer<'d, crate::bcm_dma_buf::BcmTxDmaBuf, DM>, (Hub75Error, Self)> {
        let i8080 = self.i8080;
        let tx_descriptors = self.tx_descriptors;
        let tx_buf = crate::bcm_dma_buf::BcmTxDmaBuf::new(tx_descriptors, fb);
        let word_size = fb.get_word_size();

        let result = match word_size {
            WordSize::Eight => i8080.send(Command::<u8>::None, 0, tx_buf),
            WordSize::Sixteen => i8080.send(Command::<u16>::None, 0, tx_buf),
        };
        let xfer = result.map_err(|(e, i8080, buf)| {
            let tx_descriptors = buf.split();
            (
                e.into(),
                Self {
                    i8080,
                    tx_descriptors,
                },
            )
        })?;
        Ok(Hub75Transfer {
            xfer,
            tx_descriptors: None,
        })
    }

    /// Renders using a caller-provided DMA transmit buffer.
    ///
    /// This is an escape hatch for custom [`DmaTxBuffer`] implementations
    /// where the caller manages the descriptor chain directly.
    ///
    /// # Arguments
    /// * `word_size` - Lane width to use for LCD-CAM transfer formatting.
    /// * `buf` - DMA buffer implementing [`DmaTxBuffer`].
    ///
    /// # Returns
    /// A [`Hub75Transfer`] which can be awaited and then consumed via
    /// [`Hub75Transfer::wait_with_buf`] to recover both the finalized buffer
    /// and the driver instance.
    ///
    /// # Errors
    /// Returns a tuple of `Hub75Error`, the recovered `Hub75` instance, and the
    /// original buffer if the transfer cannot be started.
    #[cfg_attr(feature = "iram", ram)]
    pub fn render_buf<BUF: DmaTxBuffer>(
        self,
        word_size: WordSize,
        buf: BUF,
    ) -> Result<Hub75Transfer<'d, BUF, DM>, (Hub75Error, Self, BUF)> {
        let i8080 = self.i8080;
        let tx_descriptors = self.tx_descriptors;
        let result = match word_size {
            WordSize::Eight => i8080.send(Command::<u8>::None, 0, buf),
            WordSize::Sixteen => i8080.send(Command::<u16>::None, 0, buf),
        };
        match result {
            Ok(xfer) => Ok(Hub75Transfer {
                xfer,
                tx_descriptors: Some(tx_descriptors),
            }),
            Err((e, i8080, buf)) => Err((
                e.into(),
                Self {
                    i8080,
                    tx_descriptors,
                },
                buf,
            )),
        }
    }
}

/// Represents an in-progress transfer to the HUB75 display
///
/// This struct is returned by `Hub75::render` and can be used to wait for the
/// transfer to complete. It provides both blocking and async methods for
/// waiting.
pub struct Hub75Transfer<'d, BUF: DmaTxBuffer, DM: esp_hal::DriverMode> {
    xfer: I8080Transfer<'d, BUF, DM>,
    tx_descriptors: Option<&'static mut [DmaDescriptor]>,
}

impl<'d, BUF: DmaTxBuffer, DM: esp_hal::DriverMode> Hub75Transfer<'d, BUF, DM> {
    /// Checks if the transfer is complete
    ///
    /// # Returns
    /// `true` if the transfer is complete and `wait()` will not block
    #[cfg_attr(feature = "iram", ram)]
    pub fn is_done(&self) -> bool {
        self.xfer.is_done()
    }

    /// Waits for transfer completion and returns the finalized buffer plus hub.
    #[cfg_attr(feature = "iram", ram)]
    pub fn wait_with_buf(self) -> (Result<(), DmaError>, BUF::Final, Hub75<'d, DM>) {
        let Hub75Transfer {
            xfer,
            tx_descriptors,
        } = self;
        let (result, i8080, final_buf) = xfer.wait();
        let tx_descriptors = tx_descriptors
            .expect("wait_with_buf is only supported for render_buf()-started transfers");
        (
            result,
            final_buf,
            Hub75 {
                i8080,
                tx_descriptors,
            },
        )
    }
}

impl<'d, DM: esp_hal::DriverMode> Hub75Transfer<'d, crate::bcm_dma_buf::BcmTxDmaBuf, DM> {
    /// Waits for the transfer to complete and recovers the `Hub75` instance.
    ///
    /// # Returns
    /// A tuple containing:
    /// 1. The result of the transfer
    /// 2. The `Hub75` instance for reuse
    #[cfg_attr(feature = "iram", ram)]
    pub fn wait(self) -> (Result<(), DmaError>, Hub75<'d, DM>) {
        let (result, i8080, bcm_buf) = self.xfer.wait();
        let tx_descriptors = bcm_buf.split();
        (
            result,
            Hub75 {
                i8080,
                tx_descriptors,
            },
        )
    }
}

impl<BUF: DmaTxBuffer> Hub75Transfer<'_, BUF, esp_hal::Async> {
    /// Asynchronously waits for the transfer to complete
    ///
    /// # Returns
    /// A `Result` indicating whether the transfer completed successfully
    ///
    /// # Note
    /// This method does not return the `Hub75` instance. Use `wait()` after
    /// `wait_for_done` returns to get the `Hub75` instance, it won't block at
    /// that point.
    #[cfg_attr(feature = "iram", ram)]
    pub async fn wait_for_done(&mut self) -> Result<(), DmaError> {
        self.xfer.wait_for_done().await;
        Ok(())
    }
}

impl<'d> crate::Hub75Pins<'d> for Hub75Pins16<'d> {
    fn apply<DM: esp_hal::DriverMode>(self, i8080: I8080<'d, DM>) -> I8080<'d, DM> {
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
    // fn convert_pins(self) -> (TxSixteenBits<'d>, AnyPin<'d>) {
    //     let (_, blank) = unsafe { self.blank.split() };
    //     let pins = TxSixteenBits::new(
    //         self.addr0,
    //         self.addr1,
    //         self.addr2,
    //         self.addr3,
    //         self.addr4,
    //         self.latch,
    //         NoPin,
    //         NoPin,
    //         blank.with_output_inverter(true),
    //         self.red1,
    //         self.grn1,
    //         self.blu1,
    //         self.red2,
    //         self.grn2,
    //         self.blu2,
    //         NoPin,
    //     );
    //     (pins, self.clock)
    // }
}

impl<'d> crate::Hub75Pins<'d> for Hub75Pins8<'d> {
    fn apply<DM: esp_hal::DriverMode>(self, i8080: I8080<'d, DM>) -> I8080<'d, DM> {
        let (_, blank) = unsafe { self.blank.split() };
        #[cfg(feature = "invert-blank")]
        let blank = blank.with_output_inverter(true);
        i8080
            .with_wrx(self.clock)
            .with_data0(self.red1)
            .with_data1(self.grn1)
            .with_data2(self.blu1)
            .with_data3(self.red2)
            .with_data4(self.grn2)
            .with_data5(self.blu2)
            .with_data6(self.latch)
            .with_data7(blank)
    }
    // fn convert_pins(self) -> (TxEightBits<'d>, AnyPin<'d>) {
    //     let (_, blank) = unsafe { self.blank.split() };
    //     let pins = TxEightBits::new(
    //         self.red1,
    //         self.grn1,
    //         self.blu1,
    //         self.red2,
    //         self.grn2,
    //         self.blu2,
    //         self.latch,
    //         #[cfg(feature = "invert-blank")]
    //         blank.with_output_inverter(true),
    //         #[cfg(not(feature = "invert-blank"))]
    //         blank,
    //     );
    //     (pins, self.clock)
    // }
}
