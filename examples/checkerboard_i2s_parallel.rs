//! Diagnostic checkerboard example for HUB75 panels (I2S-parallel).
//!
//! This example renders a filled checkerboard pattern to help diagnose column
//! ordering, bit alignment, and clock-edge sampling issues.
//!
//! It is especially useful for panels that appear to work with animations but
//! show distortion on static vertical or horizontal features.
//!
//! ## Running the example
//!
//! ```bash
//! cargo +esp run --release \
//!   --example checkerboard_i2s_parallel \
//!   --features esp32,log,esp-hal-unstable,invert-clock
//! ```
//! 
//! Disable `invert-clock` if your panel does not require inverted CLK sampling.

#![no_std]
#![no_main]
#![allow(clippy::uninlined_format_args)]

#[cfg(feature = "defmt")]
use defmt_rtt as _;
use embedded_graphics::geometry::Point;
use embedded_graphics::geometry::Size;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::prelude::{Primitive, RgbColor};
use embedded_graphics::Drawable;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::Pin;
use esp_hal::main;
use esp_hal::time::Rate;
use esp_hub75::framebuffer::compute_frame_count;
use esp_hub75::framebuffer::compute_rows;
use esp_hub75::framebuffer::plain::DmaFrameBuffer;
use esp_hub75::Color;
use esp_hub75::Hub75;
use esp_hub75::Hub75Pins16;

esp_bootloader_esp_idf::esp_app_desc!();

// Panel geometry
const ROWS: usize = 48;
const COLS: usize = 96;

// Color depth
const BITS: u8 = 1;

// Diagnostic checkerboard parameters
//
// We draw a checkerboard of filled squares (not lines) so row 0 and col 0 are
// part of a cell rather than being a "border line".
const GRID_SIZE: i32 = 8;

// Set to `-(GRID_SIZE / 2)` to start half a cell "before" (0, 0) so the first
// boundary falls at GRID_SIZE/2. This can help reveal sampling-edge issues.
const GRID_ORIGIN: i32 = 0;

const NROWS: usize = compute_rows(ROWS);
const FRAME_COUNT: usize = compute_frame_count(BITS);

type FBType = DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>;

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    let (_, tx_descriptors) = esp_hal::dma_descriptors!(0, FBType::dma_buffer_size_bytes());

    // TIP:
    // If you see column misalignment or horizontal distortion on some panels,
    // try building with the `invert-clock` feature enabled.
    let pins = Hub75Pins16 {
        red1: peripherals.GPIO25.degrade(),
        grn1: peripherals.GPIO26.degrade(),
        blu1: peripherals.GPIO27.degrade(),
        red2: peripherals.GPIO14.degrade(),
        grn2: peripherals.GPIO12.degrade(),
        blu2: peripherals.GPIO13.degrade(),
        addr0: peripherals.GPIO23.degrade(),
        addr1: peripherals.GPIO19.degrade(),
        addr2: peripherals.GPIO5.degrade(),
        addr3: peripherals.GPIO17.degrade(),
        addr4: peripherals.GPIO32.degrade(),
        blank: peripherals.GPIO15.degrade(),
        clock: peripherals.GPIO16.degrade(),
        latch: peripherals.GPIO4.degrade(),
    };

    let mut hub75 = Hub75::new(
        peripherals.I2S0.into(),
        pins,
        peripherals.DMA_I2S0,
        tx_descriptors,
        Rate::from_mhz(5),
    )
    .expect("failed to create Hub75!")
    .into_async();

    let mut fb = FBType::new();

    // Fill background
    let bg = PrimitiveStyleBuilder::new()
        .fill_color(Color::BLACK)
        .build();
    Rectangle::new(Point::new(0, 0), Size::new(COLS as u32, ROWS as u32))
        .into_styled(bg)
        .draw(&mut fb)
        .expect("failed to clear framebuffer");

    // Draw a checkerboard of GRID_SIZE x GRID_SIZE filled squares (not lines)
    // to diagnose bit/byte ordering and sampling-edge issues.
    let cell_on = PrimitiveStyleBuilder::new()
        .fill_color(Color::WHITE)
        .build();

    // Iterate cells starting at GRID_ORIGIN so the first boundary is at GRID_SIZE/2.
    // Negative coordinates are OK; drawing will be clipped to the framebuffer.
    let mut y = GRID_ORIGIN;
    let mut y_idx: i32 = 0;
    while y < ROWS as i32 {
        let mut x = GRID_ORIGIN;
        let mut x_idx: i32 = 0;
        while x < COLS as i32 {
            let is_white = ((x_idx + y_idx) & 1) == 0;
            if is_white {
                Rectangle::new(
                    Point::new(x, y),
                    Size::new(GRID_SIZE as u32, GRID_SIZE as u32),
                )
                .into_styled(cell_on)
                .draw(&mut fb)
                .expect("failed to draw checkerboard cell");
            }
            x += GRID_SIZE;
            x_idx += 1;
        }
        y += GRID_SIZE;
        y_idx += 1;
    }
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
