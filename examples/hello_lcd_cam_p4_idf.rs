//! Example for using the LCD/CAM I80 driver with a HUB75 LED matrix display
//! via ESP-IDF (std / FreeRTOS) on ESP32-P4.
//!
//! Unlike bare-metal examples there is no `#![no_std]`, no `#![no_main]`,
//! no DMA descriptor array, and no DMA channel argument. DMA is managed
//! automatically by the ESP-IDF LCD driver.
//!
//! # Build
//!
//! ```sh
//! cargo build --example hello_lcd_cam_p4_idf \
//!     --features esp32p4-idf \
//!     --target riscv32imafc-esp-espidf
//! ```
//!
//! You will also need a `.cargo/config.toml` in your project (or workspace)
//! that sets the target and ESP-IDF toolchain paths for the IDF std build,
//! e.g.:
//!
//! ```toml
//! [build]
//! target = "riscv32imafc-esp-espidf"
//!
//! [env]
//! ESP_IDF_VERSION = "v5.3"
//! ```
//!
//! # Pin mapping
//!
//! The GPIO numbers below are illustrative — adjust to suit your hardware.

use core::mem::MaybeUninit;

use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::ascii::FONT_5X7;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::prelude::RgbColor;
use embedded_graphics::text::Alignment;
use embedded_graphics::text::Text;
use embedded_graphics::Drawable;
use esp_hub75::framebuffer::compute_frame_count;
use esp_hub75::framebuffer::compute_rows;
use esp_hub75::framebuffer::plain::DmaFrameBuffer;
use esp_hub75::Color;
use esp_hub75::Hub75;
use esp_hub75::Hub75IdfPins16;

const ROWS: usize = 32;
const COLS: usize = 64;
const BITS: u8 = 4;
const NROWS: usize = compute_rows(ROWS);
const FRAME_COUNT: usize = compute_frame_count(BITS);

type FBType = DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>;

// Allocate the frame buffer as a static to avoid overflowing the stack.
static mut FB: MaybeUninit<FBType> = MaybeUninit::zeroed();

fn main() {
    // Required to initialise the ESP-IDF logger and link IDF patches
    esp_idf_sys::link_patches();

    let pins = Hub75IdfPins16 {
        red1: 38,
        grn1: 42,
        blu1: 48,
        red2: 47,
        grn2: 2,
        blu2: 21,
        addr0: 14,
        addr1: 46,
        addr2: 13,
        addr3: 9,
        addr4: 3,
        blank: 11,
        clock: 12,
        latch: 10,
        // The IDF I80 driver requires a D/C GPIO; HUB75 has no such signal so
        // any spare GPIO works. The driver drives it HIGH during transfers and
        // the panel ignores it. Change this to a free GPIO on your board.
        dc: 45,
    };

    let mut hub75 = Hub75::new(pins, 20_000_000, core::mem::size_of::<FBType>())
        .expect("failed to create Hub75");

    #[allow(static_mut_refs)]
    let fb: &mut FBType = unsafe { FB.assume_init_mut() };
    // Initialize control signals (output_enable/BLANK timing, latch, row address).
    // MaybeUninit::zeroed() skips DmaFrameBuffer::new(), so format() must be
    // called explicitly to set up the scan timing before first render.
    fb.format();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(Color::YELLOW)
        .background_color(Color::BLACK)
        .build();
    Text::with_alignment(
        "Hello, World!",
        Point::new(32, 31),
        text_style,
        Alignment::Center,
    )
    .draw(&mut fb)
    .expect("failed to draw text");

    loop {
        let xfer = hub75
            .render(&fb)
            .map_err(|(e, _hub75)| e)
            .expect("failed to start render");
        let (result, new_hub75) = xfer.wait();
        hub75 = new_hub75;
        result.expect("transfer failed");
    }
}
