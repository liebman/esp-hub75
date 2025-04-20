//! Embassy "async" example driving a 64x32 HUB75 display using
//! the I2S peripheral of an `esp32` with a SmartLEDShield_ESP32_V0 style
//! circuit.  This example should also work for 64x64 displays if you change
//! the ROWS constant to 64.
//!
//! This example draws a simple gradient on the display and shows the refresh
//! rate and render rate plus a simple counter.
//!
//! Folowing pins are used:
//!
//!   SIG     LAT   PIN
//! - R1      A  => GPIO16
//! - G1      B  => GPIO4
//! - B1      C  => GPIO17
//! - R2      D  => GPIO18
//! - G2      E  => GPIO5
//! - B2         => GPIO19
//! - OE_DMA     => GPIO26
//! - OE_PWM     => GPIO27
//! - CLK        => GPIO25
//! - LAT        => GPIO2
//!
//! NOTE1: these are not the default pins for the SmartLEDShield_ESP32_V0
//!
//! NOTE2: you most likeliy need level converters 3.3v to 5v for all HUB75
//! signals
#![no_std]
#![no_main]

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
use esp_hal::clock::CpuClock;
use esp_hal::gpio::AnyPin;
use esp_hal::gpio::Level;
use esp_hal::gpio::Output;
use esp_hal::gpio::OutputConfig;
use esp_hal::gpio::Pin;
use esp_hal::i2s::AnyI2s;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::interrupt::Priority;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_embassy::InterruptExecutor;
use esp_hub75::framebuffer::compute_frame_count;
use esp_hub75::framebuffer::compute_rows;
use esp_hub75::framebuffer::latched::DmaFrameBuffer;
use esp_hub75::Color;
use esp_hub75::Hub75;
use esp_hub75::Hub75Pins8;
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

const ROWS: usize = 64;
const COLS: usize = 64;
const BITS: u8 = 4;
const NROWS: usize = compute_rows(ROWS);
const FRAME_COUNT: usize = compute_frame_count(BITS);

const LINE1: i32 = ROWS as i32 - 1 - 14;
const LINE2: i32 = ROWS as i32 - 1 - 7;
const LINE3: i32 = ROWS as i32 - 1;
const NBARS: i32 = ROWS as i32 / 8;

type FBType = DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>;
type FrameBufferExchange = Signal<CriticalSectionRawMutex, &'static mut FBType>;

pub struct Hub75Peripherals<'d> {
    pub i2s: AnyI2s<'d>,
    pub dma_channel: esp_hal::peripherals::DMA_I2S1<'d>,
    pub red1: AnyPin<'d>,
    pub grn1: AnyPin<'d>,
    pub blu1: AnyPin<'d>,
    pub red2: AnyPin<'d>,
    pub grn2: AnyPin<'d>,
    pub blu2: AnyPin<'d>,
    pub blank: AnyPin<'d>,
    pub clock: AnyPin<'d>,
    pub latch: AnyPin<'d>,
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

        fmt::write(
            &mut buffer,
            format_args!("Refresh: {:4}", REFRESH_RATE.load(Ordering::Relaxed)),
        )
        .unwrap();
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
    peripherals: Hub75Peripherals<'static>,
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    fb: &'static mut FBType,
) {
    info!("hub75_task: starting!");
    let channel = peripherals.dma_channel;
    let (_, tx_descriptors) = esp_hal::dma_descriptors!(0, FBType::dma_buffer_size_bytes());

    let pins = Hub75Pins8 {
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

    let mut hub75 = Hub75::new(
        peripherals.i2s,
        pins,
        channel,
        tx_descriptors,
        Rate::from_mhz(20),
    )
    .expect("failed to create Hub75!")
    .into_async();

    let mut count = 0u32;
    let mut start = Instant::now();

    let mut fb = fb;

    // wait for the first fb update
    let new_fb = rx.wait().await;
    info!("hub75_task: got first fb!");
    tx.signal(fb);
    info!("hub75_task: sent back first old fb!");
    fb = new_fb;

    loop {
        // if there is a new buffer available, get it and send the old one
        if rx.signaled() {
            let new_fb = rx.wait().await;
            tx.signal(fb);
            fb = new_fb;
        }

        // info!("hub75_task: starting transfer!");
        let mut xfer = hub75
            .render(fb)
            .map_err(|(e, _hub75)| e)
            .expect("failed to start render!");
        // DANIEL: Adding either of these two lines will make it work!
        // Timer::after_micros(100).await;
        // info!("hub75_task: waiting for transfer to complete!");
        xfer.wait_for_done()
            .await
            .expect("rendering wait_for_done failed!");
        // info!("hub75_task:reconstituting hub75!");
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
async fn main(_spawner: Spawner) {
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
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let software_interrupt = sw_ints.software_interrupt2;

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
        dma_channel: peripherals.DMA_I2S1,
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

    let _pwm_pin = Output::new(peripherals.GPIO27, Level::High, OutputConfig::default());

    // let hp_executor = mk_static!(
    //     InterruptExecutor<2>,
    //     InterruptExecutor::new(software_interrupt)
    // );
    // let high_pri_spawner = hp_executor.start(Priority::Priority3);

    // // hub75 runs as high priority task
    // high_pri_spawner
    //     .spawn(hub75_task(hub75_peripherals, &RX, &TX, fb1))
    //     .ok();
    // _spawner.spawn(display_task(&TX, &RX, fb0)).ok();

    // run hub75 and display on second core
    let cpu1_fnctn = {
        move || {
            let hp_executor = mk_static!(
                InterruptExecutor<2>,
                InterruptExecutor::new(software_interrupt)
            );
            let high_pri_spawner = hp_executor.start(Priority::Priority3);

            // hub75 runs as high priority task
            high_pri_spawner
                .spawn(hub75_task(hub75_peripherals, &RX, &TX, fb1))
                .ok();

            let lp_executor = mk_static!(Executor, Executor::new());
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
    let app_core_stack = mk_static!(Stack<DISPLAY_STACK_SIZE>, Stack::new());
    let mut _cpu_control = cpu_control;

    #[allow(static_mut_refs)]
    let _guard = _cpu_control
        .start_app_core(app_core_stack, cpu1_fnctn)
        .unwrap();

    loop {
        if SIMPLE_COUNTER.fetch_add(1, Ordering::Relaxed) >= 99999 {
            SIMPLE_COUNTER.store(0, Ordering::Relaxed);
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
