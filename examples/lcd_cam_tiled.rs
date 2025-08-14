//! Embassy "async" example of an ESP32-S3 driving 9 64x32 HUB75 displays
//! in a 3x3 arrangement using the LCD_CAM peripheral.
//! Please note that you should probably use the latched framebuffer instead to
//! reach any sort of usable color depth while still fitting 2 frame buffers into RAM
//!
//! This example draws a simple gradient on the displays and shows the refresh
//! rate and render rate plus a simple counter.
//!
//! Folowing pins are used:
//! - R1  => GPIO38
//! - G1  => GPIO42
//! - B1  => GPIO48
//! - R2  => GPIO47
//! - G2  => GPIO2
//! - B2  => GPIO21
//! - A   => GPIO14
//! - B   => GPIO46
//! - C   => GPIO13
//! - D   => GPIO9
//! - E   => GPIO3
//! - OE  => GPIO11
//! - CLK => GPIO12
//! - LAT => GPIO10
//!
//! Note that you most likely need level converters 3.3v to 5v for all HUB75
//! signals

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

#[cfg(feature = "defmt")]
use defmt::info;
#[cfg(feature = "defmt")]
use defmt_rtt as _;
use alloc::fmt;
use embassy_executor::{task, Spawner};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embassy_time::{Instant, Ticker};
use embedded_graphics::mono_font::ascii::FONT_5X7;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::interrupt::Priority;
use esp_hal::psram::{FlashFreq, PsramConfig, SpiRamFreq, SpiTimingConfigCoreClock};
use esp_hal::timer::systimer::SystemTimer;
use esp_hal_embassy::InterruptExecutor;
use heapless::String;
use hub75_framebuffer::FrameBufferOperations;
#[cfg(feature = "log")]
use log::info;

use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::prelude::RgbColor;
use embedded_graphics::text::Alignment;
use embedded_graphics::text::Text;
use embedded_graphics::Drawable;
use esp_hal::gpio::Pin;
use esp_hal::peripherals::LCD_CAM;
use esp_hal::time::Rate;
use esp_hub75::framebuffer::compute_frame_count;
use esp_hub75::framebuffer::compute_rows;
use esp_hub75::framebuffer::plain::DmaFrameBuffer;
use esp_hub75::Color;
use esp_hub75::Hub75;
use esp_hub75::Hub75Pins16;
use hub75_framebuffer::tiling::{compute_tiled_cols, ChainTopRightDown, TiledFrameBuffer};

use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use static_cell::make_static;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// These are some values that can be tweaked to experiment with the display

// Whether to allocate the framebuffers to PSRAM
const ALLOCATE_TO_PSRAM: bool = false;

// This is the minimal refresh rate for the display is frames per second. Anything below 60 probably flickers
const MIN_FRAMERATE: u32 = 80;

// Specify color depth. 1 is very low but more does not fit into RAM.
// When using less panels this could be increased while still fitting into RAM.
// If the frame buffer is stored in PSRAM this
// can also be increased but it will hurt performance even more.
// When using the latched frame buffer, this can also be increased to 2.
const BITS: u8 = if ALLOCATE_TO_PSRAM { 2 } else { 1 };

// When allocating to PSRAM this need to be decreased to somewhere around 1Mhz
const TRANSFER_SPEED: Rate = if ALLOCATE_TO_PSRAM {
    Rate::from_khz(900)
} else {
    Rate::from_mhz(20)
};

// Panel layout settings
const TILED_COLS: usize = 3;
const TILED_ROWS: usize = 3;
const ROWS: usize = 32;
const PANEL_COLS: usize = 64;
const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);
const NROWS: usize = compute_rows(ROWS);
const FRAME_COUNT: usize = compute_frame_count(BITS);

static REFRESH_RATE: AtomicU32 = AtomicU32::new(0);
static RENDER_RATE: AtomicU32 = AtomicU32::new(0);
static BRIGHTNESS: AtomicU8 = AtomicU8::new(128);

type FBType = DmaFrameBuffer<ROWS, FB_COLS, NROWS, BITS, FRAME_COUNT>;
type TiledFBType = TiledFrameBuffer<
    FBType,
    ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
    ROWS,
    PANEL_COLS,
    NROWS,
    BITS,
    FRAME_COUNT,
    TILED_ROWS,
    TILED_COLS,
    FB_COLS,
>;

type FrameBufferExchange = Signal<CriticalSectionRawMutex, &'static mut TiledFBType>;

pub struct Hub75Peripherals<'d> {
    pub lcd_cam: LCD_CAM<'d>,
    pub dma_channel: esp_hal::peripherals::DMA_CH0<'d>,
    pub pins: Hub75Pins16<'d>,
}

#[task]
async fn hub75_task(
    peripherals: Hub75Peripherals<'static>,
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    fb: &'static mut TiledFBType,
) {
    info!("hub75_task: starting!");

    let (_, tx_descriptors) = esp_hal::dma_descriptors!(0, FBType::dma_buffer_size_bytes());

    let mut hub75 = Hub75::new_async(
        peripherals.lcd_cam,
        peripherals.pins,
        peripherals.dma_channel,
        tx_descriptors,
        TRANSFER_SPEED,
    )
    .expect("failed to create Hub75!");

    let mut count = 0u32;
    let mut start = Instant::now();

    let mut fb = fb;

    const MAX_DELAY: u64 = (1000_000f32 / MIN_FRAMERATE as f32) as u64;
    const MULTIPLIER: f32 = -(MAX_DELAY as f32 / 255f32);

    loop {
        // Delay control value (0-255)
        // 0 = never render (infinite delay)
        // 1 = maximum delay (MAX_DELAY microseconds)
        // 255 = no delay (0 microseconds)
        let delay_control = BRIGHTNESS.load(Ordering::Relaxed); // Use global BRIGHTNESS value

        // Calculate delay based on control value
        let delay_micros = if delay_control == 0 {
            // Never render - use a very long delay
            u64::MAX
        } else {
            // This gives: 1 -> MAX_DELAY, 255 -> 0
            (MULTIPLIER * delay_control as f32 + MAX_DELAY as f32) as u64
        };
        let mut ticker = Ticker::every(Duration::from_micros(delay_micros));

        // if there is a new buffer available, get it and send the old one
        if rx.signaled() {
            let new_fb = rx.wait().await;
            tx.signal(fb);
            fb = new_fb;
        }

        let mut xfer = hub75
            .render(fb)
            .map_err(|(e, _hub75)| e)
            .expect("failed to start render!");
        xfer.wait_for_done()
            .await
            .expect("rendering wait_for_done failed!");
        let (result, new_hub75) = xfer.wait();
        hub75 = new_hub75;
        result.expect("transfer failed");

        // Apply the calculated delay
        ticker.next().await;

        count += 1;
        const FPS_INTERVAL: Duration = Duration::from_secs(1);
        if start.elapsed() > FPS_INTERVAL {
            REFRESH_RATE.store(count, Ordering::Relaxed);
            count = 0;
            start = Instant::now();
        }
    }
}

#[task]
async fn display_task(
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    mut fb: &'static mut TiledFBType,
) {
    info!("display_task: starting!");
    let fps_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(Color::YELLOW)
        .background_color(Color::BLACK)
        .build();
    let mut count = 0u32;

    let mut start = Instant::now();

    loop {
        fb.erase();

        const STEP: u8 = (256 / (PANEL_COLS * TILED_COLS)) as u8;
        for x in 0..(PANEL_COLS * TILED_COLS) {
            let brightness = (x as u8) * STEP;
            for y in 0..8 {
                fb.set_pixel(Point::new(x as i32, y), Color::new(brightness, 0, 0));
            }
            for y in 8..16 {
                fb.set_pixel(Point::new(x as i32, y), Color::new(0, brightness, 0));
            }
            for y in 16..24 {
                fb.set_pixel(Point::new(x as i32, y), Color::new(0, 0, brightness));
            }
        }

        let mut buffer: String<64> = String::new();

        fmt::write(
            &mut buffer,
            format_args!("Refresh: {:4}", REFRESH_RATE.load(Ordering::Relaxed)),
        )
        .unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(0, 10),
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
            Point::new(192 / 2, 63 - 8),
            fps_style,
            Alignment::Center,
        )
        .draw(fb)
        .unwrap();

        buffer.clear();
        fmt::write(
            &mut buffer,
            format_args!("Bright: {:5}", BRIGHTNESS.load(Ordering::Relaxed)),
        )
        .unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(192, 95),
            fps_style,
            Alignment::Right,
        )
        .draw(fb)
        .unwrap();
        // send the frame buffer to be rendered
        tx.signal(fb);

        // get the next frame buffer
        fb = rx.wait().await;

        // count up the rate we are rendering full buffer
        count += 1;
        const FPS_INTERVAL: Duration = Duration::from_secs(1);
        if start.elapsed() > FPS_INTERVAL {
            RENDER_RATE.store(count, Ordering::Relaxed);
            count = 0;
            start = Instant::now();
        }
    }
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    #[cfg(feature = "log")]
    esp_println::logger::init_logger_from_env();

    let psram_config = PsramConfig {
        flash_frequency: FlashFreq::FlashFreq120m,
        ram_frequency: SpiRamFreq::Freq120m,
        core_clock: SpiTimingConfigCoreClock::SpiTimingConfigCoreClock240m,
        ..Default::default()
    };
    let config = esp_hal::Config::default()
        .with_cpu_clock(CpuClock::max())
        .with_psram(psram_config);
    let peripherals = esp_hal::init(config);
    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let software_interrupt = sw_ints.software_interrupt2;

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    esp_alloc::heap_allocator!(size: 32*1024);

    info!("Embassy initialized!");

    // LED Panel init
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

    let hub75_per = Hub75Peripherals {
        dma_channel: peripherals.DMA_CH0,
        lcd_cam: peripherals.LCD_CAM,
        pins,
    };

    // If the framebuffer is too large to fit in ram, we can allocate it on the
    // heap in PSRAM instead.
    // Allocate the framebuffer to PSRAM without ever putting it on the stack first
    let (fb0, fb1) = if ALLOCATE_TO_PSRAM {
        esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
        use alloc::alloc::alloc;
        use alloc::boxed::Box;
        use core::alloc::Layout;

        let layout = Layout::new::<TiledFBType>();

        let fb0 = unsafe {
            let ptr = alloc(layout) as *mut TiledFBType;
            Box::from_raw(ptr)
        };
        let fb1 = unsafe {
            let ptr = alloc(layout) as *mut TiledFBType;
            Box::from_raw(ptr)
        };
        (Box::leak(fb0), Box::leak(fb1))
    } else {
        // Allocate the framebuffers in static memory. This assumes that they fit into ram.
        (
            make_static!(TiledFrameBuffer::new()),
            make_static!(TiledFrameBuffer::new()),
        )
    };

    info!("init framebuffer exchange");
    static TX: FrameBufferExchange = FrameBufferExchange::new();
    static RX: FrameBufferExchange = FrameBufferExchange::new();

    let cpu1_fnctn = {
        move || {
            let hp_executor = make_static!(InterruptExecutor::new(software_interrupt));
            let high_pri_spawner = hp_executor.start(Priority::Priority3);

            // hub75 runs as high priority task
            high_pri_spawner
                .spawn(hub75_task(hub75_per, &RX, &TX, fb1))
                .ok();

            let lp_executor = make_static!(Executor::new());
            // display task runs as low priority task
            lp_executor.run(|spawner| {
                spawner.spawn(display_task(&TX, &RX, fb0)).ok();
            });
        }
    };

    use esp_hal::system::CpuControl;
    use esp_hal::system::Stack;
    use esp_hal_embassy::Executor;
    let cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    const DISPLAY_STACK_SIZE: usize = 8192;
    let app_core_stack: &mut Stack<DISPLAY_STACK_SIZE> = make_static!(Stack::new());
    let mut _cpu_control = cpu_control;

    #[allow(static_mut_refs)]
    let _guard = _cpu_control
        .start_app_core(app_core_stack, cpu1_fnctn)
        .unwrap();

    let mut direction = 1i8;

    loop {
        // Cycle BRIGHTNESS from 1 to 255 and back
        let current_brightness = BRIGHTNESS.load(Ordering::Relaxed);
        let new_brightness = if current_brightness == 1 {
            direction = 1;
            2
        } else if current_brightness == 255 {
            direction = -1;
            254
        } else {
            if direction == 1 {
                current_brightness + 1
            } else {
                current_brightness - 1
            }
        };

        // // Update BRIGHTNESS
        BRIGHTNESS.store(new_brightness, Ordering::Relaxed);

        Timer::after(Duration::from_millis(100)).await;
    }
}
