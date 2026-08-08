//! Embassy "async" example driving a 64x64 quarter-scan (1/16-scan) HUB75
//! panel using the I2S peripheral of an `esp32` with 16-bit bitplane
//! framebuffer (no latch circuit).
//!
//! A 1/16-scan 64x64 panel has only 16 row addresses (A-D) and lights four
//! rows at a time (rows n, n+16, n+32 and n+48 for address n). The
//! `QuarterScan` remapper (via `RemappedFrameBuffer`) presents the panel as
//! an ordinary 64x64 canvas while the underlying framebuffer is laid out as
//! 32 rows x 128 columns: 16 addresses x 2 channels with two 64-column
//! sections per channel.
//!
//! Panels with a different quarter-scan wiring can select another group
//! wiring variant via the remapper's third type parameter, e.g.
//! `QuarterScan<64, 64, quarter_scan::Linear>` — see
//! `esp_hub75::framebuffer::tiling::quarter_scan`.
//!
//! The ISR handles the entire BCM refresh loop — the async `swap()` method lets
//! the display task exchange framebuffers without blocking. The display task
//! runs on the second core via `InterruptExecutor` (kept on core 0's executor
//! for now due to esp-hal issue #2369).
//!
//! This example draws a simple gradient on the display and shows the refresh
//! rate and render rate plus a simple counter. Thin white lines at rows 15,
//! 31 and 47 mark the quarter-scan group boundaries to help visually verify
//! the panel's interleave mapping.
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
//! - E   => GPIO18 (unused: 1/16-scan panels have only 16 row addresses)
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
use esp_hub75::framebuffer::bitplane::plain::row::DmaFrameBuffer;
use esp_hub75::framebuffer::tiling::PixelRemapper;
use esp_hub75::framebuffer::tiling::QuarterScan;
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
// Virtual canvas size (what we draw to)
const ROWS: usize = 64;
const COLS: usize = 64;

// Physical framebuffer geometry for a quarter-scan (1/16-scan) panel:
// - 16 row addresses => NROWS = ROWS / 4 (the inner framebuffer holds NROWS * 2
//   rows: one per channel)
// - two 64-column sections per channel => FB_COLS = COLS * 2
const NROWS: usize = ROWS / 4;
const FB_COLS: usize = COLS * 2;
const PLANES: usize = 6;

const LINE1: i32 = ROWS as i32 - 1 - 14;
const LINE2: i32 = ROWS as i32 - 1 - 7;
const LINE3: i32 = ROWS as i32 - 1;
const NBARS: i32 = ROWS as i32 / 8;

// --- Type aliases ---
// Raw inner framebuffer (used with hub75_dma_descriptors! macro)
type InnerFB = DmaFrameBuffer<NROWS, FB_COLS, PLANES>;

// Pixel remapper for the quarter-scan (1/16-scan) panel layout (the default is SectionsSwapped if not specified)
type Remapper = QuarterScan<ROWS, COLS>;
// type Remapper = QuarterScan<ROWS, COLS, esp_hub75::framebuffer::tiling::quarter_scan::Linear>;
// type Remapper = QuarterScan<ROWS, COLS, esp_hub75::framebuffer::tiling::quarter_scan::HalvesSwapped>;
// type Remapper = QuarterScan<ROWS, COLS, esp_hub75::framebuffer::tiling::quarter_scan::Alternating>;

// Remapped framebuffer presenting a plain 64x64 virtual canvas.
// This is what we draw to and pass to Hub75.
type DisplayFB = RemappedFrameBuffer<InnerFB, Remapper>;

// The remapper's framebuffer geometry must match the inner framebuffer.
const _: () = {
    assert!(Remapper::FB_ROWS == NROWS * 2);
    assert!(Remapper::FB_COLS == FB_COLS);
    assert!(Remapper::VIRT_ROWS == ROWS);
    assert!(Remapper::VIRT_COLS == COLS);
};

#[task]
async fn display_task(hub75: Hub75<esp_hal::Async, DisplayFB>, mut fb: &'static mut DisplayFB) {
    info!("display_task: starting (quarter-scan)!");
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

        // Draw thin white lines at the quarter-scan group boundaries so we
        // can visually confirm the remapping: rows 0..16, 16..32, 32..48 and
        // 48..64 each land in a different section of the shift register.
        // for x in 0..COLS {
        //     fb.set_pixel(Point::new(x as i32, NBARS * 2 - 1), Color::WHITE);
        //     fb.set_pixel(Point::new(x as i32, NBARS * 4 - 1), Color::WHITE);
        //     fb.set_pixel(Point::new(x as i32, NBARS * 6 - 1), Color::WHITE);
        // }

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
        fmt::write(
            &mut buffer,
            format_args!("Render: {:5}", RENDER_RATE.load(Ordering::Relaxed)),
        )
        .unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(0, LINE2),
            fps_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();

        buffer.clear();
        fmt::write(
            &mut buffer,
            format_args!("Simple: {:5}", SIMPLE_COUNTER.load(Ordering::Relaxed)),
        )
        .unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(0, LINE1),
            fps_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();

        let mut xfer = hub75.swap(fb);
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

    info!("Main starting (quarter-scan 64x64)!");
    info!("Virtual: {}x{}", ROWS, COLS);
    info!(
        "FB dims: {}x{} ({} row addresses)",
        NROWS * 2,
        FB_COLS,
        NROWS
    );
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

    info!("init framebuffers (quarter-scan)");
    let fb0 = mk_static!(DisplayFB, DisplayFB::new());
    fb0.erase();
    let fb1 = mk_static!(DisplayFB, DisplayFB::new());
    fb1.erase();

    info!("fb0: {:?}", fb0);
    info!("fb1: {:?}", fb1);

    // DMA descriptors are sized for the inner (physical) framebuffer
    let tx_descriptors = esp_hub75::hub75_dma_descriptors!(InnerFB);
    info!(
        "DMA descriptors: {} ({} bytes)",
        tx_descriptors.len(),
        tx_descriptors.len() * core::mem::size_of::<esp_hal::dma::DmaDescriptor>()
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
        if SIMPLE_COUNTER.fetch_add(1, Ordering::Relaxed) >= 99999 {
            SIMPLE_COUNTER.store(0, Ordering::Relaxed);
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
