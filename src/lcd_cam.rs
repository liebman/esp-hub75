use core::cell::Cell;

use esp_hal::dma::Channel;
use esp_hal::dma::DmaChannelConvert;
use esp_hal::dma::DmaEligible;
use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaTxBuf;
use esp_hal::gpio::NoPin;
use esp_hal::lcd_cam::lcd::i8080;
use esp_hal::lcd_cam::lcd::i8080::Command;
use esp_hal::lcd_cam::lcd::i8080::TxSixteenBits;
use esp_hal::lcd_cam::lcd::i8080::I8080;
use esp_hal::lcd_cam::LcdCam;
use esp_hal::peripherals::LCD_CAM;

use crate::framebuffer::DmaFrameBuffer;
use crate::HertzU32;
use crate::Hub75Pins;

pub struct Hub75<'d, DM: esp_hal::Mode> {
    i8080: Cell<Option<I8080<'d, DM>>>,
    tx_descriptors: Cell<Option<&'static mut [DmaDescriptor]>>,
}

impl<'d> Hub75<'d, esp_hal::Async> {
    pub fn new_async<CH>(
        lcd_cam: LCD_CAM,
        hub75_pins: Hub75Pins,
        channel: Channel<'d, CH, esp_hal::Async>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: HertzU32,
    ) -> Self
    where
        CH: DmaChannelConvert<<LCD_CAM as DmaEligible>::Dma>,
    {
        let lcd_cam = LcdCam::new_async(lcd_cam);
        Self::new_internal(lcd_cam, hub75_pins, channel, tx_descriptors, frequency)
    }

    pub async fn render_async<
        const ROWS: usize,
        const COLS: usize,
        const BITS: u8,
        const SIZE: usize,
    >(
        &mut self,
        fb: &mut DmaFrameBuffer<ROWS, COLS, BITS, SIZE>,
    ) {
        let i8080 = self.i8080.take().expect("i8080 is None!");
        let tx_descriptors = self.tx_descriptors.take().expect("tx_descriptors is None!");
        let tx_buffer = unsafe {
            use esp_hal::dma::ReadBuffer;
            let (ptr, len) = fb.read_buffer();
            // SAFETY: tx_buffer is only used until the tx_buf.split below!
            core::slice::from_raw_parts_mut(ptr as *mut u8, len)
        };
        let tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).expect("DmaTxBuf::new failed");
        let mut xfer = i8080.send(Command::<u16>::None, 0, tx_buf).expect("send failed");
        xfer.wait_for_done().await;
        let (result, i8080, tx_buf) = xfer.wait();
        result.expect("transfer failed");
        let (tx_descriptors, _) = tx_buf.split();
        self.i8080.set(Some(i8080));
        self.tx_descriptors.set(Some(tx_descriptors));
    }
}

impl<'d, DM: esp_hal::Mode> Hub75<'d, DM> {
    fn new_internal<CH>(
        lcd_cam: LcdCam<'d, DM>,
        hub75_pins: Hub75Pins,
        channel: Channel<'d, CH, DM>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: HertzU32,
    ) -> Self
    where
        CH: DmaChannelConvert<<LCD_CAM as DmaEligible>::Dma>,
    {
        let pins = TxSixteenBits::new(
            hub75_pins.addr0,
            hub75_pins.addr1,
            hub75_pins.addr2,
            hub75_pins.addr3,
            hub75_pins.addr4,
            hub75_pins.latch,
            NoPin,
            NoPin,
            hub75_pins.blank.into_peripheral_output().inverted(),
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
        let i8080 = Cell::new(Some(i8080));
        let tx_descriptors = Cell::new(Some(tx_descriptors));
        Self { i8080, tx_descriptors }
    }
}
