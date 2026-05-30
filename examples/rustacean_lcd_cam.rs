//! Example rendering a Rustacean PNG image on a HUB75 LED matrix using LCD_CAM
//!
//! The image is pre-converted to raw RGB888 bytes and drawn via
//! `embedded_graphics::image::ImageRaw`.

#![no_std]
#![no_main]
#![allow(clippy::uninlined_format_args)]

#[cfg(feature = "defmt")]
use defmt_rtt as _;
use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::ascii::FONT_5X7;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::prelude::RgbColor;
use embedded_graphics::text::Alignment;
use embedded_graphics::text::Text;
use embedded_graphics::Drawable;
use embedded_sprites::image::Image;
use embedded_sprites::include_image;
use embedded_sprites::sprite::Sprite;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::Pin;
use esp_hal::main;
use esp_hal::time::Rate;
use esp_hub75::framebuffer::bitplane::plain::DmaFrameBuffer;
use esp_hub75::framebuffer::compute_rows;
use esp_hub75::Color;
use esp_hub75::Hub75;
use esp_hub75::Hub75Pins16;

esp_bootloader_esp_idf::esp_app_desc!();

const ROWS: usize = 64;
const COLS: usize = 64;
const NROWS: usize = compute_rows(ROWS);
const PLANES: usize = 7;

type FBType = DmaFrameBuffer<NROWS, COLS, PLANES>;

#[include_image]
const GRASS_DATA: Image<hub75_framebuffer::Color> = "./images/rustacean-flat-happy-64x64.png";

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    let tx_descriptors = esp_hub75::hub75_dma_descriptors!(FBType);

    let pins = Hub75Pins16 {
        red1: peripherals.GPIO38.degrade(),
        grn1: peripherals.GPIO42.degrade(),
        blu1: peripherals.GPIO48.degrade(),
        red2: peripherals.GPIO47.degrade(),
        grn2: peripherals.GPIO2.degrade(),
        blu2: peripherals.GPIO21.degrade(),
        addr0: peripherals.GPIO14.degrade(),
        addr1: peripherals.GPIO46.degrade(),
        addr2: peripherals.GPIO13.degrade(),
        addr3: peripherals.GPIO9.degrade(),
        addr4: peripherals.GPIO3.degrade(),
        blank: peripherals.GPIO11.degrade(),
        clock: peripherals.GPIO12.degrade(),
        latch: peripherals.GPIO10.degrade(),
    };

    let mut hub75 = Hub75::new_async(
        peripherals.LCD_CAM,
        pins,
        peripherals.DMA_CH0,
        tx_descriptors,
        Rate::from_mhz(20),
    )
    .expect("failed to create Hub75!");

    let mut fb = FBType::new();

    let rustacean = Sprite::new(Point::new(0, 0), &GRASS_DATA);
    rustacean.draw(&mut fb).expect("failed to draw image");

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(Color::WHITE)
        .background_color(Color::BLACK)
        .build();
    Text::with_alignment(
        "Hello, Hub75",
        Point::new(31, 55),
        text_style,
        Alignment::Center,
    )
    .draw(&mut fb)
    .expect("failed to draw text");

    loop {
        let xfer = hub75
            .render(&fb)
            .map_err(|(e, _hub75)| e)
            .expect("failed to start render!");
        let (result, new_hub75) = xfer.wait();
        hub75 = new_hub75;
        result.expect("transfer failed");
    }
}
