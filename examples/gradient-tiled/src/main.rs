//! Blocking (non-embassy) HUB75 demo driving four tiled 64×32 panels (2×2 grid)
//! as a single 128×64 virtual canvas with a 16-bit bitplane framebuffer (no
//! latch circuit). See the sibling `gradient` example for the ordinary
//! single-panel variant.
//!
//! Select the target board with exactly one of the `esp32`, `esp32s3`,
//! `esp32c6`, or `esp32-trinity` features (`esp32-trinity` implies `esp32`,
//! sharing all ESP32-common setup). The framebuffer is the 16-bit
//! `plain` (plane-major `frame`) layout by default; enable the `row` feature
//! to use the row-major `plain::row` layout instead.
//!
//! The `ChainTopRightDown` remapper (via `RemappedFrameBuffer`) presents the
//! four physical panels as one 128×64 virtual canvas. Panels are chained
//! starting at the top right, running left across each row, then wrapping
//! down to the next row (every second row is mounted upside-down). The
//! underlying framebuffer is laid out as 32 rows × 256 columns: 16 addresses ×
//! 2 channels, with four 64-column panels per channel.
//!
//! The ISR runs the BCM refresh loop; the blocking `swap()` + `wait()` method
//! exchanges framebuffers without an async runtime. Everything runs in a
//! single `#[main]` loop.
//!
//! This example draws a simple gradient across the full virtual canvas, draws
//! quadrant boundary lines and labels to confirm the tiling, and shows the
//! refresh rate, render rate and a simple counter.
//!
//! Note that you most likely need level converters 3.3v to 5v for all HUB75
//! signals.

#![no_std]
#![no_main]
#![allow(clippy::uninlined_format_args)]

use core::fmt;

#[cfg(feature = "defmt")]
use defmt::info;
#[cfg(feature = "defmt")]
use defmt_rtt as _;
use embedded_graphics::Drawable;
use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::mono_font::ascii::FONT_5X7;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::text::Alignment;
use embedded_graphics::text::Text;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::Pin;
use esp_hal::main;
use esp_hal::time::Duration;
use esp_hal::time::Instant;
use esp_hal::time::Rate;
use esp_hub75::Color;
use esp_hub75::Hub75;
use esp_hub75::Hub75Pins16;
use esp_hub75::framebuffer::FrameBufferOperations;
#[cfg(not(feature = "row"))]
use esp_hub75::framebuffer::bitplane::plain::DmaFrameBuffer;
#[cfg(feature = "row")]
use esp_hub75::framebuffer::bitplane::plain::row::DmaFrameBuffer;
use esp_hub75::framebuffer::tiling::ChainTopRightDown;
use esp_hub75::framebuffer::tiling::PixelRemapper;
use esp_hub75::framebuffer::tiling::RemappedFrameBuffer;
use heapless::String;
#[cfg(feature = "log")]
use log::info;

esp_bootloader_esp_idf::esp_app_desc!();

// ---- board selection check ----
// `esp32-trinity` implies `esp32`, so the chip features alone determine
// exclusivity.
#[cfg(not(any(feature = "esp32", feature = "esp32s3", feature = "esp32c6")))]
compile_error!(
    "no board selected; enable exactly one of: `esp32`, `esp32s3`, `esp32c6`, `esp32-trinity`"
);
#[cfg(any(
    all(feature = "esp32", feature = "esp32s3"),
    all(feature = "esp32", feature = "esp32c6"),
    all(feature = "esp32s3", feature = "esp32c6")
))]
compile_error!(
    "multiple board features enabled; enable exactly one of: `esp32`, `esp32s3`, `esp32c6`, \
     `esp32-trinity`"
);

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

// --- Panel geometry ---
// Each physical panel is 64×32; four of them are tiled in a 2×2 grid.
const PANEL_ROWS: usize = 32;
const PANEL_COLS: usize = 64;
const TILE_ROWS: usize = 2;
const TILE_COLS: usize = 2;

// Physical framebuffer geometry for a 2×2 tiled arrangement:
// - 16 row addresses => NROWS = PANEL_ROWS / 2 (the inner framebuffer holds
//   NROWS * 2 rows: one per channel)
// - four 64-column panels per channel => FB_COLS = PANEL_COLS * TILE_ROWS *
//   TILE_COLS
const ROWS: usize = PANEL_ROWS;
const NROWS: usize = ROWS / 2;
const FB_COLS: usize = PANEL_COLS * TILE_ROWS * TILE_COLS;

// Virtual canvas size (what we draw to).
const VIRT_ROWS: usize = PANEL_ROWS * TILE_ROWS;
const VIRT_COLS: usize = PANEL_COLS * TILE_COLS;

const PLANES: usize = 4;

const LINE1: i32 = VIRT_ROWS as i32 - 1 - 14;
const LINE2: i32 = VIRT_ROWS as i32 - 1 - 7;
const LINE3: i32 = VIRT_ROWS as i32 - 1;
const NBARS: i32 = VIRT_ROWS as i32 / 8;

// --- Type aliases ---
// Raw inner framebuffer (used with hub75_dma_descriptors! macro)
type InnerFB = DmaFrameBuffer<NROWS, FB_COLS, PLANES>;

// Pixel remapper for the 2×2 tiling with ChainTopRightDown
type Remapper = ChainTopRightDown<ROWS, PANEL_COLS, TILE_ROWS, TILE_COLS>;

// Remapped framebuffer presenting a 128×64 virtual canvas.
// This is what we draw to and pass to Hub75.
type DisplayFB = RemappedFrameBuffer<InnerFB, Remapper>;

// The remapper's framebuffer geometry must match the inner framebuffer.
const _: () = {
    assert!(Remapper::FB_ROWS == NROWS * 2);
    assert!(Remapper::FB_COLS == FB_COLS);
    assert!(Remapper::VIRT_ROWS == VIRT_ROWS);
    assert!(Remapper::VIRT_COLS == VIRT_COLS);
};

unsafe extern "C" {
    static _stack_end_cpu0: u32;
    static _stack_start_cpu0: u32;
}

#[main]
fn main() -> ! {
    #[cfg(feature = "log")]
    esp_println::logger::init_logger(log::LevelFilter::Info);
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    info!("Main starting (tiled 2x2 128x64)!");
    info!("main: stack size:  {}", unsafe {
        core::ptr::addr_of!(_stack_start_cpu0).offset_from(core::ptr::addr_of!(_stack_end_cpu0))
    });
    info!("Panel: {}x{}", PANEL_ROWS, PANEL_COLS);
    info!("Tile:  {}x{}", TILE_ROWS, TILE_COLS);
    info!("Virtual: {}x{}", VIRT_ROWS, VIRT_COLS);
    info!("FB dims: {}x{}", NROWS * 2, FB_COLS);
    info!("PLANES: {}", PLANES);
    info!("FB (InnerFB) size: {}", core::mem::size_of::<InnerFB>());
    info!("FB (DisplayFB) size: {}", core::mem::size_of::<DisplayFB>());

    info!("init framebuffers (tiled)");
    let fb0 = mk_static!(DisplayFB, DisplayFB::new());
    let fb1 = mk_static!(DisplayFB, DisplayFB::new());

    info!("fb0: {:?}", fb0);
    info!("fb1: {:?}", fb1);

    // DMA descriptors are sized for the inner (physical) framebuffer
    let tx_descriptors = esp_hub75::hub75_dma_descriptors!(InnerFB);
    info!(
        "DMA descriptors: {} ({} bytes)",
        tx_descriptors.len(),
        core::mem::size_of_val(tx_descriptors)
    );

    #[cfg(feature = "esp32c6")]
    let pins = Hub75Pins16 {
        red1: peripherals.GPIO19.degrade(),
        grn1: peripherals.GPIO20.degrade(),
        blu1: peripherals.GPIO21.degrade(),
        red2: peripherals.GPIO22.degrade(),
        grn2: peripherals.GPIO23.degrade(),
        blu2: peripherals.GPIO15.degrade(),
        addr0: peripherals.GPIO10.degrade(),
        addr1: peripherals.GPIO8.degrade(),
        addr2: peripherals.GPIO1.degrade(),
        addr3: peripherals.GPIO0.degrade(),
        addr4: peripherals.GPIO11.degrade(),
        blank: peripherals.GPIO5.degrade(),
        clock: peripherals.GPIO7.degrade(),
        latch: peripherals.GPIO6.degrade(),
    };

    #[cfg(feature = "esp32s3")]
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

    #[cfg(all(feature = "esp32", not(feature = "esp32-trinity")))]
    let pins = Hub75Pins16 {
        red1: peripherals.GPIO16.degrade(),
        grn1: peripherals.GPIO4.degrade(),
        blu1: peripherals.GPIO17.degrade(),
        red2: peripherals.GPIO18.degrade(),
        grn2: peripherals.GPIO5.degrade(),
        blu2: peripherals.GPIO19.degrade(),
        addr0: peripherals.GPIO15.degrade(),
        addr1: peripherals.GPIO13.degrade(),
        addr2: peripherals.GPIO12.degrade(),
        addr3: peripherals.GPIO14.degrade(),
        addr4: peripherals.GPIO2.degrade(),
        blank: peripherals.GPIO25.degrade(),
        clock: peripherals.GPIO27.degrade(),
        latch: peripherals.GPIO26.degrade(),
    };

    #[cfg(feature = "esp32-trinity")]
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
        addr4: peripherals.GPIO18.degrade(),
        blank: peripherals.GPIO15.degrade(),
        clock: peripherals.GPIO16.degrade(),
        latch: peripherals.GPIO4.degrade(),
    };

    #[cfg(all(feature = "20mhz", not(feature = "esp32")))]
    let rate = Rate::from_mhz(20);
    #[cfg(all(feature = "20mhz", feature = "esp32"))]
    let rate = Rate::from_mhz(19);
    #[cfg(not(feature = "20mhz"))]
    let rate = Rate::from_mhz(10);

    #[cfg(feature = "esp32c6")]
    let hub75 = Hub75::new(
        peripherals.PARL_IO,
        pins,
        peripherals.DMA_CH0,
        tx_descriptors,
        rate,
        &*fb0,
    )
    .expect("failed to create Hub75");

    #[cfg(feature = "esp32s3")]
    let hub75 = Hub75::new(
        peripherals.LCD_CAM,
        pins,
        peripherals.DMA_CH0,
        tx_descriptors,
        rate,
        &*fb0,
    )
    .expect("failed to create Hub75");

    // `esp32-trinity` implies `esp32`, so both ESP32 boards share the I2S0
    // peripheral path.
    #[cfg(feature = "esp32")]
    let hub75 = Hub75::new(
        peripherals.I2S0,
        pins,
        peripherals.DMA_I2S0,
        tx_descriptors,
        rate,
        &*fb0,
    )
    .expect("failed to create Hub75");

    // fb0 is now displayed by the ISR; fb1 is our idle drawing buffer.
    let mut fb = fb1;

    let fps_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(Color::YELLOW)
        .background_color(Color::BLACK)
        .build();

    let label_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(Color::CYAN)
        .background_color(Color::BLACK)
        .build();

    let mut render_count = 0u32;
    let mut refresh_count_start = hub75.frame_count();
    let mut start = Instant::now();
    let mut refresh_rate = 0u32;
    let mut render_rate = 0u32;

    let mut simple_counter = 0u32;
    let mut counter_start = Instant::now();

    loop {
        fb.erase();

        // Draw a horizontal RGB gradient across the full 128×64 virtual canvas.
        // Three horizontal bands split evenly (~8 rows each).
        const STEP: u8 = (256 / VIRT_COLS) as u8;
        for x in 0..VIRT_COLS {
            let brightness = (x as u8).saturating_mul(STEP);
            for y in 0..NBARS {
                fb.set_pixel(Point::new(x as i32, y), Color::new(brightness, 0, 0));
                fb.set_pixel(
                    Point::new(x as i32, y + NBARS),
                    Color::new(0, brightness, 0),
                );
                fb.set_pixel(
                    Point::new(x as i32, y + 2 * NBARS),
                    Color::new(0, 0, brightness),
                );
            }
        }

        // Draw thin white lines at the quadrant boundaries so we can visually
        // confirm the tiling is correct.

        // Vertical boundary at the midpoint
        for y in 0..VIRT_ROWS {
            fb.set_pixel(Point::new((PANEL_COLS - 1) as i32, y as i32), Color::WHITE);
        }
        // Horizontal boundary at the midpoint
        for x in 0..VIRT_COLS {
            fb.set_pixel(Point::new(x as i32, (PANEL_ROWS - 1) as i32), Color::WHITE);
        }

        // Quadrant labels
        Text::with_alignment("TL", Point::new(2, 6), label_style, Alignment::Left)
            .draw(fb)
            .unwrap();
        Text::with_alignment(
            "TR",
            Point::new((PANEL_COLS + 2) as i32, 6),
            label_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();
        Text::with_alignment(
            "BL",
            Point::new(2, (PANEL_ROWS + 6) as i32),
            label_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();
        Text::with_alignment(
            "BR",
            Point::new((PANEL_COLS + 2) as i32, (PANEL_ROWS + 6) as i32),
            label_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();

        // Status lines: label left-justified, value right-justified at the
        // right edge of the virtual canvas.
        let right = VIRT_COLS as i32 - 1;

        let mut buffer: String<16> = String::new();

        // Refresh
        fmt::write(&mut buffer, format_args!("Refresh:")).unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(0, LINE3),
            fps_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();
        buffer.clear();
        fmt::write(&mut buffer, format_args!("{:4}", refresh_rate)).unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(right, LINE3),
            fps_style,
            Alignment::Right,
        )
        .draw(fb)
        .unwrap();
        buffer.clear();

        // Render
        fmt::write(&mut buffer, format_args!("Render:")).unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(0, LINE2),
            fps_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();
        buffer.clear();
        fmt::write(&mut buffer, format_args!("{:5}", render_rate)).unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(right, LINE2),
            fps_style,
            Alignment::Right,
        )
        .draw(fb)
        .unwrap();
        buffer.clear();

        // Simple
        fmt::write(&mut buffer, format_args!("Simple:")).unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(0, LINE1),
            fps_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();
        buffer.clear();
        fmt::write(&mut buffer, format_args!("{}", simple_counter)).unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(right, LINE1),
            fps_style,
            Alignment::Right,
        )
        .draw(fb)
        .unwrap();

        fb = hub75
            .swap(fb)
            .expect("swap already in flight")
            .wait()
            .expect("DMA transfer failed");

        render_count += 1;
        if start.elapsed() > Duration::from_secs(1) {
            render_rate = render_count;
            let current_frame_count = hub75.frame_count();
            refresh_rate = current_frame_count.wrapping_sub(refresh_count_start);
            refresh_count_start = current_frame_count;
            render_count = 0;
            start = Instant::now();
        }

        // Increment the simple counter once every ~100 ms.
        if counter_start.elapsed() >= Duration::from_millis(100) {
            if simple_counter >= 99999 {
                simple_counter = 0;
            } else {
                simple_counter += 1;
            }
            counter_start = Instant::now();
        }
    }
}
