//! Embassy "async" example driving 4 tiled 64×32 HUB75 panels (2×2 grid) on an
//! ESP32-Trinity board using the I2S peripheral with a bitplane framebuffer.
//!
//! Uses `RemappedFrameBuffer` with `ChainTopRightDown` to present the four
//! physical panels as a single 128×64 virtual canvas.  The ISR handles the
//! entire BCM refresh loop — the async `swap()` method lets the display task
//! exchange framebuffers without blocking.
//!
//! This example draws color bars that fill the full virtual canvas and shows
//! the refresh rate and render rate plus a simple counter.
//!
//! Following pins are used: (ESP-Trinity board)
//! - R1  => GPIO25
//! - G1  => GPIO26
//! - B1  => GPIO27
//! - R2  => GPIO14
//! - G2  => GPIO12
//! - B2  => GPIO13
//! - A   => GPIO23
//! - B   => GPIO19
//! - C   => GPIO5
//! - D   => GPIO17
//! - E   => GPIO18
//! - OE  => GPIO15
//! - CLK => GPIO16
//! - LAT => GPIO4
//!
//! Note that you most likely need level converters 3.3v to 5v for all HUB75
//! signals
#![no_std]
#![no_main]
#![allow(clippy::uninlined_format_args)]

use core::fmt;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;

#[cfg(feature = "defmt")]
use defmt::info;
#[cfg(feature = "defmt")]
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_executor::task;
use embassy_time::Duration;
use embassy_time::Instant;
use embassy_time::Timer;
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
use esp_hal::interrupt::Priority;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hub75::Color;
use esp_hub75::Hub75;
use esp_hub75::Hub75Pins16;
use esp_hub75::framebuffer::FrameBufferOperations;
use esp_hub75::framebuffer::bitplane::plain::DmaFrameBuffer;
use esp_hub75::framebuffer::tiling::ChainTopRightDown;
use esp_hub75::framebuffer::tiling::RemappedFrameBuffer;
use esp_rtos::embassy::InterruptExecutor;
use heapless::String;
#[cfg(feature = "log")]
use log::info;

esp_bootloader_esp_idf::esp_app_desc!();

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

static RENDER_RATE: AtomicU32 = AtomicU32::new(0);
static SIMPLE_COUNTER: AtomicU32 = AtomicU32::new(0);

// --- Panel geometry ---
const PANEL_ROWS: usize = 32;
const PANEL_COLS: usize = 64;
const TILE_ROWS: usize = 2;
const TILE_COLS: usize = 2;

// Physical framebuffer dimensions (matching ChainTopRightDown's layout)
// FB_COLS = PANEL_COLS * TILE_ROWS * TILE_COLS = 64 * 2 * 2 = 256
const ROWS: usize = PANEL_ROWS;
const FB_COLS: usize = PANEL_COLS * TILE_ROWS * TILE_COLS;
const NROWS: usize = ROWS / 2;
const PLANES: usize = 4;

// Virtual canvas size from ChainTopRightDown
const VIRT_ROWS: usize = PANEL_ROWS * TILE_ROWS; // 64
const VIRT_COLS: usize = PANEL_COLS * TILE_COLS; // 128

const LINE1: i32 = VIRT_ROWS as i32 - 1 - 14;
const LINE2: i32 = VIRT_ROWS as i32 - 1 - 7;
const LINE3: i32 = VIRT_ROWS as i32 - 1;

// --- Type aliases ---
// Raw inner framebuffer (used with hub75_dma_descriptors! macro)
type InnerFB = DmaFrameBuffer<NROWS, FB_COLS, PLANES>;

// Pixel remapper for 2×2 tiling with ChainTopRightDown
type Remapper = ChainTopRightDown<ROWS, PANEL_COLS, TILE_ROWS, TILE_COLS>;

// Remapped (tiled) framebuffer presenting a 128×64 virtual canvas
// This is what we draw to and pass to Hub75
type DisplayFB = RemappedFrameBuffer<InnerFB, Remapper>;

#[task]
async fn display_task(hub75: Hub75<esp_hal::Async, DisplayFB>, mut fb: &'static mut DisplayFB) {
    info!("display_task: starting (tiled 4-panel)!");
    let fps_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(Color::YELLOW)
        .background_color(Color::BLACK)
        .build();
    let mut render_count = 0u32;
    let mut refresh_count_start = hub75.frame_count();
    let mut start = Instant::now();
    let mut refresh_rate = 0u32;

    loop {
        fb.erase();

        // Draw color bars across the full 128×64 virtual canvas.
        // Three horizontal bands split evenly (~21 rows each).
        let step = (256 / VIRT_COLS) as u8;
        let nbars = VIRT_ROWS as i32 / 8;
        for x in 0..VIRT_COLS {
            let brightness = (x as u8).saturating_mul(step);
            for y in 0..nbars {
                fb.set_pixel(Point::new(x as i32, y), Color::new(brightness, 0, 0));
                fb.set_pixel(
                    Point::new(x as i32, y + nbars),
                    Color::new(0, brightness, 0),
                );
                fb.set_pixel(
                    Point::new(x as i32, y + 2 * nbars),
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
        let label_style = MonoTextStyleBuilder::new()
            .font(&FONT_5X7)
            .text_color(Color::CYAN)
            .background_color(Color::BLACK)
            .build();
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

        let mut label_buf: String<16> = String::new();
        let mut value_buf: String<8> = String::new();

        // Refresh
        fmt::write(&mut label_buf, format_args!("Refresh:")).unwrap();
        fmt::write(&mut value_buf, format_args!("{:4}", refresh_rate)).unwrap();
        Text::with_alignment(
            label_buf.as_str(),
            Point::new(0, LINE3),
            fps_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();
        Text::with_alignment(
            value_buf.as_str(),
            Point::new(right, LINE3),
            fps_style,
            Alignment::Right,
        )
        .draw(fb)
        .unwrap();

        label_buf.clear();
        value_buf.clear();

        // Render
        fmt::write(&mut label_buf, format_args!("Render:")).unwrap();
        fmt::write(
            &mut value_buf,
            format_args!("{:5}", RENDER_RATE.load(Ordering::Relaxed)),
        )
        .unwrap();
        Text::with_alignment(
            label_buf.as_str(),
            Point::new(0, LINE2),
            fps_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();
        Text::with_alignment(
            value_buf.as_str(),
            Point::new(right, LINE2),
            fps_style,
            Alignment::Right,
        )
        .draw(fb)
        .unwrap();

        label_buf.clear();
        value_buf.clear();

        // Simple
        fmt::write(&mut label_buf, format_args!("Simple:")).unwrap();
        fmt::write(
            &mut value_buf,
            format_args!("{}", SIMPLE_COUNTER.load(Ordering::Relaxed)),
        )
        .unwrap();
        Text::with_alignment(
            label_buf.as_str(),
            Point::new(0, LINE1),
            fps_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();
        Text::with_alignment(
            value_buf.as_str(),
            Point::new(right, LINE1),
            fps_style,
            Alignment::Right,
        )
        .draw(fb)
        .unwrap();

        let mut xfer = hub75.swap(fb).expect("swap already in flight");
        xfer.wait_for_done().await;
        fb = xfer.wait().expect("DMA transfer failed");

        render_count += 1;
        const FPS_INTERVAL: Duration = Duration::from_secs(1);
        if start.elapsed() > FPS_INTERVAL {
            RENDER_RATE.store(render_count, Ordering::Relaxed);
            let current_frame_count = hub75.frame_count();
            refresh_rate = current_frame_count.wrapping_sub(refresh_count_start);
            refresh_count_start = current_frame_count;
            render_count = 0;
            start = Instant::now();
        }
    }
}

unsafe extern "C" {
    static _stack_end_cpu0: u32;
    static _stack_start_cpu0: u32;
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    #[cfg(feature = "log")]
    esp_println::logger::init_logger(log::LevelFilter::Info);
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    info!("Main starting (tiled 4-panel)!");
    info!("Panel: {}x{}", PANEL_ROWS, PANEL_COLS);
    info!("Tile:  {}x{}", TILE_ROWS, TILE_COLS);
    info!("Virtual: {}x{}", VIRT_ROWS, VIRT_COLS);
    info!("FB dims: {}x{}", NROWS * 2, FB_COLS);
    info!("PLANES: {}", PLANES);
    info!("FB (InnerFB) size: {}", core::mem::size_of::<InnerFB>());
    info!("FB (DisplayFB) size: {}", core::mem::size_of::<DisplayFB>());
    info!("main: stack size:  {}", unsafe {
        core::ptr::addr_of!(_stack_start_cpu0).offset_from(core::ptr::addr_of!(_stack_end_cpu0))
    });
    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let software_interrupt = sw_ints.software_interrupt2;

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    info!("init embassy");
    esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

    info!("init framebuffers (tiled)");
    // const RVO-compatible constructors place the 41 KiB framebuffers
    // directly in static memory — no stack overflow, no unsafe pointer casts.
    let fb0 = mk_static!(DisplayFB, DisplayFB::new());
    let fb1 = mk_static!(DisplayFB, DisplayFB::new());
    info!("fb0: {:?}", fb0);
    info!("fb1: {:?}", fb1);

    // DMA descriptors sized for the inner framebuffer
    let tx_descriptors = esp_hub75::hub75_dma_descriptors!(InnerFB);
    info!(
        "DMA descriptors: {} ({} bytes)",
        tx_descriptors.len(),
        core::mem::size_of_val(tx_descriptors)
    );

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

    let hub75 = Hub75::new_async(
        peripherals.I2S0,
        pins,
        peripherals.DMA_I2S0,
        tx_descriptors,
        Rate::from_mhz(10),
        &*fb0, // Pass a shared reference to the DisplayFB (implements ReadBuffer etc.)
    )
    .expect("failed to create Hub75");

    let cpu1_fnctn = {
        move || {
            let hp_executor = mk_static!(
                InterruptExecutor<2>,
                InterruptExecutor::new(software_interrupt)
            );
            let high_pri_spawner = hp_executor.start(Priority::Priority3);
            high_pri_spawner.spawn(display_task(hub75, fb1).unwrap());
        }
    };

    use esp_hal::system::Stack;
    const DISPLAY_STACK_SIZE: usize = 8192;
    let app_core_stack = mk_static!(Stack<DISPLAY_STACK_SIZE>, Stack::new());

    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        sw_ints.software_interrupt1,
        app_core_stack,
        cpu1_fnctn,
    );

    loop {
        SIMPLE_COUNTER.fetch_add(1, Ordering::Relaxed);
        Timer::after(Duration::from_millis(100)).await;
    }
}
