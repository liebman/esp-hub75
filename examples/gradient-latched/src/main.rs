//! Blocking (non-embassy) HUB75 demo driving a 64×64 panel with an 8-bit
//! bitplane framebuffer and a SmartLEDShield-style latch circuit. See the
//! sibling `gradient` example for the 16-bit direct-drive (non-latched)
//! variant.
//!
//! Select the target board with exactly one of the `esp32`, `esp32s3`,
//! `esp32c6`, or `esp32c5` features. The framebuffer is the 8-bit `latched`
//! layout by default; enable the `row` feature to use the row-major
//! `latched::row` layout instead.
//!
//! The ISR runs the BCM refresh loop; the blocking `swap()` + `wait()` method
//! exchanges framebuffers without an async runtime. Everything runs in a
//! single `#[main]` loop.
//!
//! This example draws a simple gradient on the display and shows the refresh
//! rate, render rate and a simple counter.
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
use esp_hal::gpio::Level;
use esp_hal::gpio::Output;
use esp_hal::gpio::OutputConfig;
use esp_hal::gpio::Pin;
use esp_hal::main;
use esp_hal::time::Duration;
use esp_hal::time::Instant;
use esp_hal::time::Rate;
use esp_hub75::Color;
use esp_hub75::Hub75;
use esp_hub75::Hub75Config;
use esp_hub75::Hub75Pins8;
#[cfg(not(feature = "row"))]
use esp_hub75::framebuffer::bitplane::latched::DmaFrameBuffer;
#[cfg(feature = "row")]
use esp_hub75::framebuffer::bitplane::latched::row::DmaFrameBuffer;
use esp_hub75::framebuffer::compute_rows;
use heapless::String;
#[cfg(feature = "log")]
use log::info;

esp_bootloader_esp_idf::esp_app_desc!();

// ---- board selection check ----
#[cfg(not(any(
    feature = "esp32",
    feature = "esp32s3",
    feature = "esp32c6",
    feature = "esp32c5"
)))]
compile_error!(
    "no board selected; enable exactly one of: `esp32`, `esp32s3`, `esp32c6`, `esp32c5`"
);
#[cfg(any(
    all(feature = "esp32", feature = "esp32s3"),
    all(feature = "esp32", feature = "esp32c6"),
    all(feature = "esp32", feature = "esp32c5"),
    all(feature = "esp32s3", feature = "esp32c6"),
    all(feature = "esp32s3", feature = "esp32c5"),
    all(feature = "esp32c6", feature = "esp32c5")
))]
compile_error!(
    "multiple board features enabled; enable exactly one of: `esp32`, `esp32s3`, `esp32c6`, \
     `esp32c5`"
);

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

const ROWS: usize = 64;
const COLS: usize = 64;
const NROWS: usize = compute_rows(ROWS);
const PLANES: usize = 6;

const LINE1: i32 = ROWS as i32 - 1 - 14;
const LINE2: i32 = ROWS as i32 - 1 - 7;
const LINE3: i32 = ROWS as i32 - 1;
const NBARS: i32 = ROWS as i32 / 8;

type FBType = DmaFrameBuffer<NROWS, COLS, PLANES>;

unsafe extern "C" {
    static _stack_end_cpu0: u32;
    static _stack_start_cpu0: u32;
}

#[main]
fn main() -> ! {
    #[cfg(feature = "log")]
    esp_println::logger::init_logger(log::LevelFilter::Info);
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    info!("Main starting!");
    info!("main: stack size:  {}", unsafe {
        core::ptr::addr_of!(_stack_start_cpu0).offset_from(core::ptr::addr_of!(_stack_end_cpu0))
    });
    info!("ROWS: {}", ROWS);
    info!("COLS: {}", COLS);
    info!("PLANES: {}", PLANES);
    info!("FB size: {}", core::mem::size_of::<FBType>());

    info!("init framebuffers");
    let fb0 = mk_static!(FBType, FBType::new());
    let fb1 = mk_static!(FBType, FBType::new());

    info!("fb0: {:?}", fb0);
    info!("fb1: {:?}", fb1);

    let tx_descriptors = esp_hub75::hub75_dma_descriptors!(FBType);
    info!(
        "DMA descriptors: {} ({} bytes)",
        tx_descriptors.len(),
        core::mem::size_of_val(tx_descriptors)
    );

    #[cfg(feature = "esp32")]
    let pins = Hub75Pins8 {
        red1: peripherals.GPIO16.degrade(),
        grn1: peripherals.GPIO4.degrade(),
        blu1: peripherals.GPIO17.degrade(),
        red2: peripherals.GPIO18.degrade(),
        grn2: peripherals.GPIO5.degrade(),
        blu2: peripherals.GPIO19.degrade(),
        blank: peripherals.GPIO26.degrade(),
        clock: peripherals.GPIO25.degrade(),
        latch: peripherals.GPIO2.degrade(),
    };

    #[cfg(feature = "esp32s3")]
    let pins = Hub75Pins8 {
        red1: peripherals.GPIO10.degrade(),
        grn1: peripherals.GPIO11.degrade(),
        blu1: peripherals.GPIO12.degrade(),
        red2: peripherals.GPIO13.degrade(),
        grn2: peripherals.GPIO9.degrade(),
        blu2: peripherals.GPIO14.degrade(),
        blank: peripherals.GPIO45.degrade(),
        clock: peripherals.GPIO47.degrade(),
        latch: peripherals.GPIO21.degrade(),
    };

    #[cfg(feature = "esp32c6")]
    let pins = Hub75Pins8 {
        red1: peripherals.GPIO10.degrade(),
        grn1: peripherals.GPIO8.degrade(),
        blu1: peripherals.GPIO1.degrade(),
        red2: peripherals.GPIO0.degrade(),
        grn2: peripherals.GPIO11.degrade(),
        blu2: peripherals.GPIO7.degrade(),
        blank: peripherals.GPIO21.degrade(),
        clock: peripherals.GPIO19.degrade(),
        latch: peripherals.GPIO18.degrade(),
    };

    #[cfg(feature = "esp32c5")]
    let pins = Hub75Pins8 {
        red1: peripherals.GPIO9.degrade(),
        grn1: peripherals.GPIO8.degrade(),
        blu1: peripherals.GPIO7.degrade(),
        red2: peripherals.GPIO6.degrade(),
        grn2: peripherals.GPIO10.degrade(),
        blu2: peripherals.GPIO1.degrade(),
        blank: peripherals.GPIO27.degrade(),
        clock: peripherals.GPIO5.degrade(),
        latch: peripherals.GPIO26.degrade(),
    };

    #[cfg(feature = "esp32")]
    let _oe_pwm = Output::new(peripherals.GPIO27, Level::High, OutputConfig::default());
    #[cfg(feature = "esp32s3")]
    let _oe_pwm = Output::new(peripherals.GPIO48, Level::High, OutputConfig::default());
    #[cfg(feature = "esp32c6")]
    let _oe_pwm = Output::new(peripherals.GPIO20, Level::High, OutputConfig::default());
    #[cfg(feature = "esp32c5")]
    let _oe_pwm = Output::new(peripherals.GPIO4, Level::High, OutputConfig::default());

    #[cfg(all(feature = "20mhz", not(feature = "esp32")))]
    let rate = Rate::from_mhz(20);
    #[cfg(all(feature = "20mhz", feature = "esp32"))]
    let rate = Rate::from_mhz(19);
    #[cfg(not(feature = "20mhz"))]
    let rate = Rate::from_mhz(10);

    // NOTE: the ESP32's I2S0 does not support true 8-bit parallel output (it
    // drops every odd byte), so the 8-bit latched framebuffer must use I2S1.
    #[cfg(feature = "esp32")]
    let hub75 = Hub75::new(
        peripherals.I2S1,
        pins,
        peripherals.DMA_I2S1,
        tx_descriptors,
        Hub75Config::new(rate),
        &*fb0,
    )
    .expect("failed to create Hub75");

    #[cfg(feature = "esp32s3")]
    let hub75 = Hub75::new(
        peripherals.LCD_CAM,
        pins,
        peripherals.DMA_CH0,
        tx_descriptors,
        Hub75Config::new(rate),
        &*fb0,
    )
    .expect("failed to create Hub75");

    #[cfg(any(feature = "esp32c6", feature = "esp32c5"))]
    let hub75 = Hub75::new(
        peripherals.PARL_IO,
        pins,
        peripherals.DMA_CH0,
        tx_descriptors,
        Hub75Config::new(rate),
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

    let mut render_count = 0u32;
    let mut refresh_count_start = hub75.frame_count();
    let mut start = Instant::now();
    let mut refresh_rate = 0u32;
    let mut render_rate = 0u32;

    let mut simple_counter = 0u32;
    let mut counter_start = Instant::now();

    loop {
        fb.erase();

        const STEP: u8 = (256 / COLS) as u8;
        for x in 0..COLS {
            let brightness = (x as u8) * STEP;
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

        let mut buffer: String<64> = String::new();

        fmt::write(&mut buffer, format_args!("Refresh: {:4}", refresh_rate)).unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(0, LINE3),
            fps_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();

        buffer.clear();
        fmt::write(&mut buffer, format_args!("Render: {:5}", render_rate)).unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(0, LINE2),
            fps_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();

        buffer.clear();
        fmt::write(&mut buffer, format_args!("Simple: {:5}", simple_counter)).unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(0, LINE1),
            fps_style,
            Alignment::Left,
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
