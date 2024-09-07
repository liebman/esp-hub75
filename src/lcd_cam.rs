use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaPriority;
use esp_hal::gpio::AnyPin;
use esp_hal::gpio::ErasedPin;
use esp_hal::gpio::DummyPin;
use esp_hal::lcd_cam::lcd::i8080;
use esp_hal::lcd_cam::lcd::i8080::Command;
use esp_hal::lcd_cam::lcd::i8080::TxSixteenBits;
use esp_hal::lcd_cam::lcd::i8080::I8080;
use esp_hal::lcd_cam::LcdCam;
use esp_hal::peripherals::LCD_CAM;
use esp_hal::prelude::*;

use crate::framebuffer::DmaFrameBuffer;

pub struct Hub75Pins {
    pub red1: ErasedPin,
    pub grn1: ErasedPin,
    pub blu1: ErasedPin,
    pub red2: ErasedPin,
    pub grn2: ErasedPin,
    pub blu2: ErasedPin,
    pub addr0: ErasedPin,
    pub addr1: ErasedPin,
    pub addr2: ErasedPin,
    pub addr3: ErasedPin,
    pub addr4: ErasedPin,
    pub blank: ErasedPin,
    pub clock: ErasedPin,
    pub latch: ErasedPin,
}

type Hub75TxSixteenBits<'d> = TxSixteenBits<
    'd,
    ErasedPin,
    ErasedPin,
    ErasedPin,
    ErasedPin,
    ErasedPin,
    ErasedPin,
    AnyPin<'d>,
    DummyPin,
    DummyPin,
    DummyPin,
    ErasedPin,
    ErasedPin,
    ErasedPin,
    ErasedPin,
    ErasedPin,
    ErasedPin,
>;

// TODO: make DMA channel a type parameter
use esp_hal::dma::DmaChannel0;
pub struct Hub75<'d, DM: esp_hal::Mode> {
    i8080: I8080<'d, DmaChannel0, Hub75TxSixteenBits<'d>, DM>,
}

impl<'d> Hub75<'d, esp_hal::Blocking> {
    pub fn new(
        lcd_cam: LCD_CAM,
        hub75_pins: Hub75Pins,
        channel: esp_hal::dma::ChannelCreator<0>,
        tx_descriptors: &'static mut [DmaDescriptor],
    ) -> Self {
        let lcd_cam = LcdCam::new(lcd_cam);
        Self::new_internal(lcd_cam, hub75_pins, channel, tx_descriptors)
    }
}

impl<'d> Hub75<'d, esp_hal::Async> {
    pub fn new_async(
        lcd_cam: LCD_CAM,
        hub75_pins: Hub75Pins,
        channel: esp_hal::dma::ChannelCreator<0>,
        tx_descriptors: &'static mut [DmaDescriptor],
    ) -> Self {
        let lcd_cam = LcdCam::new_async(lcd_cam);
        Self::new_internal(lcd_cam, hub75_pins, channel, tx_descriptors)
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
        hub75_pins: Hub75Pins,
        channel: esp_hal::dma::ChannelCreator<0>,
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
            AnyPin::new_inverted(hub75_pins.blank),
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
        )
        .with_ctrl_pins(DummyPin::new(), hub75_pins.clock);

        Self { i8080 }
    }
}
