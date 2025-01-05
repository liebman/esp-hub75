//! Embassy "async" example driving a 64x64 HUB75 display using
//! the I2S peripheral of an `esp32` with a SmartLEDShield_ESP32_V0 style
//! circuit.
//!
//!
//! This example draws a simple gradient on the display and shows the refresh
//! rate and render rate plus a simple counter.
//!
//! Folowing pins are used:
//!   SIG     LAT  PIN
//! - R1      A  => GPIO4
//! - G1      B  => GPIO21
//! - B1      C  => GPIO22
//! - R2      D  => GPIO2
//! - G2      E  => GPIO25
//! - B2         => GPIO0
//! - OE_DMA     => GPIO32
//! - OE_PWM     => GPIO33
//! - CLK        => GPIO26
//! - LAT        => GPIO27
//!
//! Note that you most likeliy need level converters 3.3v to 5v for all HUB75
//! signals
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::fmt;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;

#[cfg(feature = "defmt")]
use defmt::info;
#[cfg(feature = "defmt")]
use defmt_rtt as _;
use embassy_executor::task;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Duration;
use embassy_time::Instant;
use embassy_time::Timer;
use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::ascii::FONT_5X7;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::text::Alignment;
use embedded_graphics::text::Text;
use embedded_graphics::Drawable;
use esp_backtrace as _;
use esp_hal::dma::Dma;
use esp_hal::dma::DmaPriority;
use esp_hal::dma::I2s1DmaChannelCreator;
use esp_hal::gpio::AnyPin;
use esp_hal::i2s::parallel::AnyI2s;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::interrupt::Priority;
use esp_hal::prelude::*;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_embassy::InterruptExecutor;
use esp_hub75::fb_latched::compute_frame_count;
use esp_hub75::fb_latched::compute_rows;
use esp_hub75::fb_latched::DmaFrameBuffer;
use esp_hub75::i2s_parallel_latch::Hub75;
use esp_hub75::i2s_parallel_latch::Hub75Pins;
use esp_hub75::Color;
use heapless::String;
#[cfg(feature = "log")]
use log::info;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

static REFRESH_RATE: AtomicU32 = AtomicU32::new(0);
static RENDER_RATE: AtomicU32 = AtomicU32::new(0);
static SIMPLE_COUNTER: AtomicU32 = AtomicU32::new(0);

const ROWS: usize = 32;
const COLS: usize = 64;
const BITS: u8 = 4;
const NROWS: usize = compute_rows(ROWS);
const FRAME_COUNT: usize = compute_frame_count(BITS);

type Hub75Type<'d> = Hub75<'d, esp_hal::Async>;
type FBType = DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>;
type FrameBufferExchange = Signal<CriticalSectionRawMutex, &'static mut FBType>;

pub struct Hub75Peripherals {
    pub i2s: AnyI2s,
    pub dma_channel: I2s1DmaChannelCreator,
    pub red1: AnyPin,
    pub grn1: AnyPin,
    pub blu1: AnyPin,
    pub red2: AnyPin,
    pub grn2: AnyPin,
    pub blu2: AnyPin,
    pub blank: AnyPin,
    pub clock: AnyPin,
    pub latch: AnyPin,
}

#[task]
async fn display_task(
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    mut fb: &'static mut FBType,
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
        fb.clear();

        const STEP: u8 = (256 / COLS) as u8;
        for x in 0..COLS {
            let brightness = (x as u8) * STEP;
            for y in 0..4 {
                fb.set_pixel(Point::new(x as i32, y), Color::new(brightness, 0, 0));
            }
            for y in 4..8 {
                fb.set_pixel(Point::new(x as i32, y), Color::new(0, brightness, 0));
            }
            for y in 8..12 {
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
            Point::new(0, 31),
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
            Point::new(0, 31 - 7),
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
            Point::new(0, 31 - 14),
            fps_style,
            Alignment::Left,
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

#[task]
async fn hub75_task(
    peripherals: Hub75Peripherals,
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    fb: &'static mut FBType,
) {
    info!("hub75_task: starting!");
    let channel = peripherals
        .dma_channel
        .configure(false, DmaPriority::Priority0);
    let (_, tx_descriptors) = esp_hal::dma_descriptors!(0, size_of::<FBType>());

    let pins = Hub75Pins {
        red1: peripherals.red1,
        grn1: peripherals.grn1,
        blu1: peripherals.blu1,
        red2: peripherals.red2,
        grn2: peripherals.grn2,
        blu2: peripherals.blu2,
        blank: peripherals.blank,
        clock: peripherals.clock,
        latch: peripherals.latch,
    };

    let mut hub75 = Hub75Type::new_async(peripherals.i2s, pins, channel, tx_descriptors, 20.MHz())
        .expect("failed to create Hub75!");

    let mut count = 0u32;
    let mut start = Instant::now();

    let mut fb = fb;

    loop {
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

        count += 1;
        const FPS_INTERVAL: Duration = Duration::from_secs(1);
        if start.elapsed() > FPS_INTERVAL {
            REFRESH_RATE.store(count, Ordering::Relaxed);
            count = 0;
            start = Instant::now();
        }
    }
}

extern "C" {
    static _stack_end_cpu0: u32;
    static _stack_start_cpu0: u32;
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    #[cfg(feature = "log")]
    esp_println::logger::init_logger(log::LevelFilter::Info);
    info!("Main starting!");
    info!("main: stack size:  {}", unsafe {
        core::ptr::addr_of!(_stack_start_cpu0).offset_from(core::ptr::addr_of!(_stack_end_cpu0))
    });
    info!("ROWS: {}", ROWS);
    info!("COLS: {}", COLS);
    info!("BITS: {}", BITS);
    info!("FRAMES: {}", FRAME_COUNT);
    info!("FB size: {}", size_of::<FBType>());
    let mut config = esp_hal::Config::default();
    config.cpu_clock = CpuClock::max();
    let peripherals = esp_hal::init(config);
    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let software_interrupt = sw_ints.software_interrupt2;
    let dma = Dma::new(peripherals.DMA);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    info!("init embassy");
    esp_hal_embassy::init(timg0.timer0);

    info!("init framebuffer exchange");
    static TX: FrameBufferExchange = FrameBufferExchange::new();
    static RX: FrameBufferExchange = FrameBufferExchange::new();

    info!("init framebuffers");
    let fb0 = mk_static!(FBType, FBType::new());
    let fb1 = mk_static!(FBType, FBType::new());
    fb0.clear();
    fb1.clear();

    info!("fb0: {:?}", fb0);
    info!("fb1: {:?}", fb1);

    let hub75_peripherals = Hub75Peripherals {
        i2s: peripherals.I2S1.into(),
        dma_channel: dma.i2s1channel,
        red1: peripherals.GPIO4.degrade(),
        grn1: peripherals.GPIO21.degrade(),
        blu1: peripherals.GPIO22.degrade(),
        red2: peripherals.GPIO2.degrade(),
        grn2: peripherals.GPIO25.degrade(),
        blu2: peripherals.GPIO0.degrade(),
        blank: peripherals.GPIO32.degrade(),
        clock: peripherals.GPIO26.degrade(),
        latch: peripherals.GPIO27.degrade(),
    };

    let hp_executor = mk_static!(
        InterruptExecutor<2>,
        InterruptExecutor::new(software_interrupt)
    );
    let high_pri_spawner = hp_executor.start(Priority::Priority3);

    // hub75 runs as high priority task
    high_pri_spawner
        .spawn(hub75_task(hub75_peripherals, &RX, &TX, fb1))
        .ok();
    spawner.spawn(display_task(&TX, &RX, fb0)).ok();

    // // run hub75 and display on second core
    // let cpu1_fnctn = {
    //     move || {
    //         let hp_executor = mk_static!(
    //             InterruptExecutor<2>,
    //             InterruptExecutor::new(software_interrupt)
    //         );
    //         let high_pri_spawner = hp_executor.start(Priority::Priority3);

    //         // hub75 runs as high priority task
    //         high_pri_spawner
    //             .spawn(hub75_task(hub75_peripherals, &RX, &TX, fb1))
    //             .ok();

    //         let lp_executor = mk_static!(Executor, Executor::new());
    //         // display task runs as low priority task
    //         lp_executor.run(|spawner| {
    //             spawner.spawn(display_task(&TX, &RX, fb0)).ok();
    //         });
    //     }
    // };

    // use esp_hal::cpu_control::CpuControl;
    // use esp_hal::cpu_control::Stack;
    // use esp_hal_embassy::Executor;
    // let cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    // const DISPLAY_STACK_SIZE: usize = 8192;
    // let app_core_stack = mk_static!(Stack<DISPLAY_STACK_SIZE>, Stack::new());
    // let mut _cpu_control = cpu_control;

    // #[allow(static_mut_refs)]
    // let _guard = _cpu_control
    //     .start_app_core(app_core_stack, cpu1_fnctn)
    //     .unwrap();

    loop {
        if SIMPLE_COUNTER.fetch_add(1, Ordering::Relaxed) >= 99999 {
            SIMPLE_COUNTER.store(0, Ordering::Relaxed);
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
