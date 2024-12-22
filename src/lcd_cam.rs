use esp_hal::dma::Channel;
use esp_hal::dma::DmaChannelConvert;
use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaEligible;
use esp_hal::dma::DmaError;
use esp_hal::dma::DmaTxBuf;
use esp_hal::gpio::NoPin;
use esp_hal::lcd_cam::lcd::i8080;
use esp_hal::lcd_cam::lcd::i8080::Command;
use esp_hal::lcd_cam::lcd::i8080::I8080Transfer;
use esp_hal::lcd_cam::lcd::i8080::TxSixteenBits;
use esp_hal::lcd_cam::lcd::i8080::I8080;
use esp_hal::lcd_cam::LcdCam;
use esp_hal::peripherals::LCD_CAM;

use crate::framebuffer::DmaFrameBuffer;
use crate::HertzU32;
use crate::Hub75Error;
use crate::Hub75Pins;

pub struct Hub75<'d, DM: esp_hal::Mode> {
    i8080: I8080<'d, DM>,
    tx_descriptors: &'static mut [DmaDescriptor],
}

impl<'d> Hub75<'d, esp_hal::Blocking> {
    pub fn new<CH>(
        lcd_cam: LCD_CAM,
        hub75_pins: Hub75Pins,
        channel: Channel<'d, esp_hal::Blocking, CH>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: HertzU32,
    ) -> Result<Self, Hub75Error>
    where
        CH: DmaChannelConvert<<LCD_CAM as DmaEligible>::Dma>,
    {
        let lcd_cam = LcdCam::new(lcd_cam);
        Self::new_internal(lcd_cam, hub75_pins, channel, tx_descriptors, frequency)
    }
}

impl<'d> Hub75<'d, esp_hal::Async> {
    pub fn new_async<CH>(
        lcd_cam: LCD_CAM,
        hub75_pins: Hub75Pins,
        channel: Channel<'d, esp_hal::Blocking, CH>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: HertzU32,
    ) -> Result<Self, Hub75Error>
    where
        CH: DmaChannelConvert<<LCD_CAM as DmaEligible>::Dma>,
    {
        let lcd_cam = LcdCam::new(lcd_cam).into_async();
        Self::new_internal(lcd_cam, hub75_pins, channel, tx_descriptors, frequency)
    }
}

impl<'d, DM: esp_hal::Mode> Hub75<'d, DM> {
    fn new_internal<CH>(
        lcd_cam: LcdCam<'d, DM>,
        hub75_pins: Hub75Pins,
        channel: Channel<'d, esp_hal::Blocking, CH>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: HertzU32,
    ) -> Result<Self, Hub75Error>
    where
        CH: DmaChannelConvert<<LCD_CAM as DmaEligible>::Dma>,
    {
        let (_, blank) = hub75_pins.blank.split();
        let pins = TxSixteenBits::new(
            hub75_pins.addr0,
            hub75_pins.addr1,
            hub75_pins.addr2,
            hub75_pins.addr3,
            hub75_pins.addr4,
            hub75_pins.latch,
            NoPin,
            NoPin,
            blank.inverted(),
            hub75_pins.red1,
            hub75_pins.grn1,
            hub75_pins.blu1,
            hub75_pins.red2,
            hub75_pins.grn2,
            hub75_pins.blu2,
            NoPin,
        );

        let i8080 = I8080::new(
            lcd_cam.lcd,
            channel.tx,
            pins,
            frequency,
            i8080::Config::default(),
        )
        .with_ctrl_pins(NoPin, hub75_pins.clock);
        Ok(Self {
            i8080,
            tx_descriptors,
        })
    }

    pub fn render<const ROWS: usize, const COLS: usize, const BITS: u8, const SIZE: usize>(
        self,
        fb: &mut DmaFrameBuffer<ROWS, COLS, BITS, SIZE>,
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
            .send(Command::<u16>::None, 0, tx_buf)
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

pub struct Hub75Transfer<'d, DM: esp_hal::Mode> {
    pub xfer: I8080Transfer<'d, DmaTxBuf, DM>,
}

impl<'d, DM: esp_hal::Mode> Hub75Transfer<'d, DM> {
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

impl<'d> Hub75Transfer<'d, esp_hal::Async> {
    pub async fn wait_for_done(&mut self) -> Result<(), DmaError> {
        Ok(self.xfer.wait_for_done().await)
    }
}
