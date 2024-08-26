use esp_hal::clock::Clocks;
use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaPriority;
use esp_hal::gpio::AnyPin;
use esp_hal::gpio::DummyPin;
use esp_hal::lcd_cam::lcd::i8080;
use esp_hal::lcd_cam::lcd::i8080::Command;
use esp_hal::lcd_cam::lcd::i8080::TxSixteenBits;
use esp_hal::lcd_cam::lcd::i8080::I8080;
use esp_hal::lcd_cam::LcdCam;
use esp_hal::peripherals::LCD_CAM;
use esp_hal::prelude::*;

use crate::framebuffer::DmaFrameBuffer;

pub trait GetBuffer {
    fn get_buffer(&self) -> &'static [u8];
}

pub struct Hub75Pins<'d> {
    pub red1: AnyPin<'d>,
    pub grn1: AnyPin<'d>,
    pub blu1: AnyPin<'d>,
    pub red2: AnyPin<'d>,
    pub grn2: AnyPin<'d>,
    pub blu2: AnyPin<'d>,
    pub addr0: AnyPin<'d>,
    pub addr1: AnyPin<'d>,
    pub addr2: AnyPin<'d>,
    pub addr3: AnyPin<'d>,
    pub addr4: AnyPin<'d>,
    pub blank: AnyPin<'d>,
    pub clock: AnyPin<'d>,
    pub latch: AnyPin<'d>,
}

type Hub75TxSixteenBits<'d> = TxSixteenBits<
    'd,
    AnyPin<'d>,
    AnyPin<'d>,
    AnyPin<'d>,
    AnyPin<'d>,
    AnyPin<'d>,
    AnyPin<'d>,
    AnyPin<'d>,
    DummyPin,
    DummyPin,
    DummyPin,
    AnyPin<'d>,
    AnyPin<'d>,
    AnyPin<'d>,
    AnyPin<'d>,
    AnyPin<'d>,
    AnyPin<'d>,
>;

// TODO: make DMA channel a type parameter
use esp_hal::dma::DmaChannel0;
pub struct Hub75<'d, DM: esp_hal::Mode> {
    i8080: I8080<'d, DmaChannel0, Hub75TxSixteenBits<'d>, DM>,
}

impl<'d> Hub75<'d, esp_hal::Blocking> {
    pub fn new(
        lcd_cam: LCD_CAM,
        hub75_pins: Hub75Pins<'d>,
        channel: esp_hal::dma::ChannelCreator<0>,
        clocks: &'d Clocks,
        tx_descriptors: &'static mut [DmaDescriptor],
    ) -> Self {
        let lcd_cam = LcdCam::new(lcd_cam);
        Self::new_internal(lcd_cam, hub75_pins, channel, clocks, tx_descriptors)
    }
}

#[cfg(feature = "async")]
impl<'d> Hub75<'d, esp_hal::Async> {
    pub fn new_async(
        lcd_cam: LCD_CAM,
        hub75_pins: Hub75Pins<'d>,
        channel: esp_hal::dma::ChannelCreator<0>,
        clocks: &'d Clocks,
        tx_descriptors: &'static mut [DmaDescriptor],
    ) -> Self {
        let lcd_cam = LcdCam::new_async(lcd_cam);
        Self::new_internal(lcd_cam, hub75_pins, channel, clocks, tx_descriptors)
    }

    pub async fn render_async<
        const ROWS: usize,
        const COLS: usize,
        const BITS: u8,
        const SIZE: usize,
    >(
        &mut self,
        fb: &DmaFrameBuffer<ROWS, COLS, BITS, SIZE>,
    ) {
        self.i8080
            .send_dma_async(Command::<u16>::None, 0, fb)
            .await
            .expect("write_framebuffer failed");
    }
}

impl<'d, DM: esp_hal::Mode> Hub75<'d, DM> {
    fn new_internal(
        lcd_cam: LcdCam<'d, DM>,
        hub75_pins: Hub75Pins<'d>,
        channel: esp_hal::dma::ChannelCreator<0>,
        clocks: &'d Clocks,
        tx_descriptors: &'static mut [DmaDescriptor],
    ) -> Self {
        let channel = channel.configure(false, DmaPriority::Priority0);
        let pins = TxSixteenBits::new(
            hub75_pins.addr0,
            hub75_pins.addr1,
            hub75_pins.addr2,
            hub75_pins.addr3,
            hub75_pins.addr4,
            hub75_pins.latch,
            hub75_pins.blank,
            DummyPin::new(),
            DummyPin::new(),
            DummyPin::new(),
            hub75_pins.red1,
            hub75_pins.grn1,
            hub75_pins.blu1,
            hub75_pins.red2,
            hub75_pins.grn2,
            hub75_pins.blu2,
        );

        let i8080 = I8080::new(
            lcd_cam.lcd,
            channel.tx,
            tx_descriptors,
            pins,
            20.MHz(),
            i8080::Config::default(),
            clocks,
        )
        .with_ctrl_pins(DummyPin::new(), hub75_pins.clock);

        Self { i8080 }
    }
}
