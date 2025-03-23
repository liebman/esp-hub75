use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaError;
use esp_hal::dma::DmaTxBuf;
use esp_hal::dma::TxChannelFor;
use esp_hal::gpio::AnyPin;
use esp_hal::gpio::NoPin;
use esp_hal::lcd_cam::lcd::i8080;
use esp_hal::lcd_cam::lcd::i8080::Command;
use esp_hal::lcd_cam::lcd::i8080::I8080Transfer;
use esp_hal::lcd_cam::lcd::i8080::TxEightBits;
use esp_hal::lcd_cam::lcd::i8080::I8080;
use esp_hal::lcd_cam::LcdCam;
use esp_hal::peripheral::Peripheral;
use esp_hal::peripherals::LCD_CAM;
use esp_hal::time::Rate;

use crate::framebuffer::latched::DmaFrameBuffer;
use crate::Hub75Error;

pub struct Hub75Pins {
    pub red1: AnyPin,
    pub grn1: AnyPin,
    pub blu1: AnyPin,
    pub red2: AnyPin,
    pub grn2: AnyPin,
    pub blu2: AnyPin,
    pub blank: AnyPin,
    pub clock: AnyPin,
    pub latch: AnyPin,
}

pub struct Hub75<'d, DM: esp_hal::DriverMode> {
    i8080: I8080<'d, DM>,
    tx_descriptors: &'static mut [DmaDescriptor],
}

impl<'d> Hub75<'d, esp_hal::Blocking> {
    pub fn new<CH>(
        lcd_cam: LCD_CAM,
        hub75_pins: Hub75Pins,
        channel: impl Peripheral<P = CH> + 'd,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
    ) -> Result<Self, Hub75Error>
    where
        CH: TxChannelFor<LCD_CAM>,
    {
        let lcd_cam = LcdCam::new(lcd_cam);
        Self::new_internal(lcd_cam, hub75_pins, channel, tx_descriptors, frequency)
    }
}

impl<'d> Hub75<'d, esp_hal::Async> {
    pub fn new_async<CH>(
        lcd_cam: LCD_CAM,
        hub75_pins: Hub75Pins,
        channel: impl Peripheral<P = CH> + 'd,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
    ) -> Result<Self, Hub75Error>
    where
        CH: TxChannelFor<LCD_CAM>,
    {
        let lcd_cam = LcdCam::new(lcd_cam).into_async();
        Self::new_internal(lcd_cam, hub75_pins, channel, tx_descriptors, frequency)
    }
}

impl<'d, DM: esp_hal::DriverMode> Hub75<'d, DM> {
    fn new_internal<CH>(
        lcd_cam: LcdCam<'d, DM>,
        hub75_pins: Hub75Pins,
        channel: impl Peripheral<P = CH> + 'd,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
    ) -> Result<Self, Hub75Error>
    where
        CH: TxChannelFor<LCD_CAM>,
    {
        let pins = TxEightBits::new(
            hub75_pins.red1,
            hub75_pins.grn1,
            hub75_pins.blu1,
            hub75_pins.red2,
            hub75_pins.grn2,
            hub75_pins.blu2,
            hub75_pins.latch,
            #[cfg(feature = "invert-blank")]
            hub75_pins.blank,
            #[cfg(not(feature = "invert-blank"))]
            hub75_pins.blank,
        );

        let i8080 = I8080::new(
            lcd_cam.lcd,
            channel,
            pins,
            i8080::Config::default().with_frequency(frequency),
        )
        .map_err(Hub75Error::I8080)?
        .with_ctrl_pins(NoPin, hub75_pins.clock);
        Ok(Self {
            i8080,
            tx_descriptors,
        })
    }

    pub fn render<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    >(
        self,
        fb: &mut DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>,
    ) -> Result<Hub75Transfer<'d, DM>, (Hub75Error, Self)> {
        let i8080 = self.i8080;
        let tx_descriptors = self.tx_descriptors;
        let tx_buffer = unsafe {
            use esp_hal::dma::ReadBuffer;
            let (ptr, len) = fb.read_buffer();
            // SAFETY: tx_buffer is only used until the tx_buf.split below!
            core::slice::from_raw_parts_mut(ptr as *mut u8, len)
        };
        // TODO: can't recover from this because tx_descriptors is consumed!
        let tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).expect("failed to create DmaTxBuf!");
        let xfer = i8080
            .send(Command::<u8>::None, 0, tx_buf)
            .map_err(|(e, i8080, buf)| {
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

pub struct Hub75Transfer<'d, DM: esp_hal::DriverMode> {
    pub xfer: I8080Transfer<'d, DmaTxBuf, DM>,
}

impl<'d, DM: esp_hal::DriverMode> Hub75Transfer<'d, DM> {
    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        self.xfer.is_done()
    }

    /// Waits for the transfer to finish and returns the peripheral and buffer.
    ///
    /// Note: This also clears the transfer interrupt so it can be used in
    /// interrupt handlers to "handle" the interrupt.
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
    pub async fn wait_for_done(&mut self) -> Result<(), DmaError> {
        self.xfer.wait_for_done().await;
        Ok(())
    }
}
