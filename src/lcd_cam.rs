use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaError;
use esp_hal::dma::DmaTxBuf;
use esp_hal::dma::TxChannelFor;
use esp_hal::gpio::NoPin;
use esp_hal::lcd_cam::lcd::i8080;
use esp_hal::lcd_cam::lcd::i8080::Command;
use esp_hal::lcd_cam::lcd::i8080::I8080Transfer;
use esp_hal::lcd_cam::lcd::i8080::I8080;
use esp_hal::lcd_cam::LcdCam;
use esp_hal::peripherals::LCD_CAM;
use esp_hal::time::Rate;
#[cfg(feature = "iram")]
use esp_hal::ram;

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
        let i8080 = I8080::new(
            lcd_cam.lcd,
            channel,
            i8080::Config::default().with_frequency(frequency),
        )
        .map_err(Hub75Error::I8080)?;
        let i8080 = hub75_pins.apply(i8080);
        Ok(Self {
            i8080,
            tx_descriptors,
        })
    }

    /// Renders a frame buffer to the display.
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
    pub fn render<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    >(
        self,
        fb: &impl FrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>,
    ) -> Result<Hub75Transfer<'d, DM>, (Hub75Error, Self)> {
        let i8080 = self.i8080;
        let tx_descriptors = self.tx_descriptors;
        let tx_buffer = unsafe {
            let (ptr, len) = fb.read_buffer();
            // SAFETY: tx_buffer is only used until the tx_buf.split below!
            core::slice::from_raw_parts_mut(ptr as *mut u8, len)
        };
        // TODO: can't recover from this because tx_descriptors is consumed!
        let tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).expect("failed to create DmaTxBuf!");
        let result = match fb.get_word_size() {
            WordSize::Eight => i8080.send(Command::<u8>::None, 0, tx_buf),
            WordSize::Sixteen => i8080.send(Command::<u16>::None, 0, tx_buf),
        };
        let xfer = result.map_err(|(e, i8080, buf)| {
            let (tx_descriptors, _) = buf.split();
            (
                e.into(),
                Self {
                    i8080,
                    tx_descriptors,
                },
            )
        })?;
        Ok(Hub75Transfer { xfer })
    }
}

/// Represents an in-progress transfer to the HUB75 display
///
/// This struct is returned by `Hub75::render` and can be used to wait for the
/// transfer to complete. It provides both blocking and async methods for
/// waiting.
pub struct Hub75Transfer<'d, DM: esp_hal::DriverMode> {
    xfer: I8080Transfer<'d, DmaTxBuf, DM>,
}

impl<'d, DM: esp_hal::DriverMode> Hub75Transfer<'d, DM> {
    /// Checks if the transfer is complete
    ///
    /// # Returns
    /// `true` if the transfer is complete and `wait()` will not block
    #[cfg_attr(feature = "iram", ram)]
    pub fn is_done(&self) -> bool {
        self.xfer.is_done()
    }

    /// Waits for the transfer to complete
    ///
    /// # Returns
    /// A tuple containing:
    /// 1. The result of the transfer
    /// 2. The `Hub75` instance for reuse
    ///
    /// # Note
    /// This method clears the transfer interrupt flag
    #[cfg_attr(feature = "iram", ram)]
    pub fn wait(self) -> (Result<(), DmaError>, Hub75<'d, DM>) {
        let (result, i8080, tx_buf) = self.xfer.wait();
        let (tx_descriptors, _) = tx_buf.split();
        match result {
            Ok(()) => (
                Ok(()),
                Hub75 {
                    i8080,
                    tx_descriptors,
                },
            ),
            Err(e) => (
                Err(e),
                Hub75 {
                    i8080,
                    tx_descriptors,
                },
            ),
        }
    }
}

impl Hub75Transfer<'_, esp_hal::Async> {
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
