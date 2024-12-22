//! Embassy "async" example of an ESP32-C6 driving a 64x64 HUB75 display using
//! the PARL_IO peripheral.
//!
//! This example draws a simple gradient on the display and shows the refresh
//! rate and render rate plus a simple counter.
//!
//! Folowing pins are used:
//! - R1  => GPIO19
//! - G1  => GPIO20
//! - B1  => GPIO21
//! - R2  => GPIO22
//! - G2  => GPIO23
//! - B2  => GPIO15
//! - A   => GPIO10
//! - B   => GPIO8
//! - C   => GPIO1
//! - D   => GPIO0
//! - E   => GPIO11
//! - OE  => GPIO5
//! - CLK => GPIO7
//! - LAT => GPIO6
//!
//! Note that you most likeliy need level converters 3.3v to 5v for all HUB75
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
use esp_hal::dma::Dma;
use esp_hal::dma::DmaPriority;
use esp_hal::gpio::AnyPin;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::interrupt::Priority;
use esp_hal::peripherals::PARL_IO;
use esp_hal::prelude::*;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_embassy::InterruptExecutor;
use esp_hub75::framebuffer::compute_buffer_size;
use esp_hub75::framebuffer::DmaFrameBuffer;
use esp_hub75::framebuffer::Entry;
use esp_hub75::parl_io::Hub75;
use esp_hub75::Color;
use esp_hub75::Hub75Pins;
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

pub struct DisplayPeripherals {
    pub parl_io: PARL_IO,
    pub dma_channel: esp_hal::dma::ChannelCreator<0>,
    pub red1: AnyPin,
    pub grn1: AnyPin,
    pub blu1: AnyPin,
    pub red2: AnyPin,
    pub grn2: AnyPin,
    pub blu2: AnyPin,
    pub addr0: AnyPin,
    pub addr1: AnyPin,
    pub addr2: AnyPin,
    pub addr3: AnyPin,
    pub addr4: AnyPin,
    pub blank: AnyPin,
    pub clock: AnyPin,
    pub latch: AnyPin,
}

const ROWS: usize = 64;
const COLS: usize = 64;
const BITS: u8 = 3;
const SIZE: usize = compute_buffer_size(ROWS, COLS, BITS);

type Hub75Type<'d> = Hub75<'d, esp_hal::Async>;
type FBType = DmaFrameBuffer<ROWS, COLS, BITS, SIZE>;
type FrameBufferExchange = Signal<CriticalSectionRawMutex, &'static mut FBType>;

pub static REFRESH_RATE: AtomicU32 = AtomicU32::new(0);
pub static RENDER_RATE: AtomicU32 = AtomicU32::new(0);
pub static SIMPLE_COUNTER: AtomicU32 = AtomicU32::new(0);

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
            Point::new(0, 63),
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
            Point::new(0, 63 - 8),
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
            Point::new(0, 63 - 16),
            fps_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();

        // send the frame buffer to be rendered
        tx.signal(fb);
        // get the next frame buffer
        fb = rx.wait().await;
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
    peripherals: DisplayPeripherals,
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    fb: &'static mut FBType,
) {
    info!("hub75_task: starting!");
    let channel = peripherals
        .dma_channel
        .configure(false, DmaPriority::Priority0);
    let (_, tx_descriptors) = esp_hal::dma_descriptors!(0, SIZE * size_of::<Entry>());

    let pins = Hub75Pins {
        red1: peripherals.red1,
        grn1: peripherals.grn1,
        blu1: peripherals.blu1,
        red2: peripherals.red2,
        grn2: peripherals.grn2,
        blu2: peripherals.blu2,
        addr0: peripherals.addr0,
        addr1: peripherals.addr1,
        addr2: peripherals.addr2,
        addr3: peripherals.addr3,
        addr4: peripherals.addr4,
        blank: peripherals.blank,
        clock: peripherals.clock,
        latch: peripherals.latch,
    };

    let mut hub75 =
        Hub75Type::new_async(peripherals.parl_io, pins, channel, tx_descriptors, 15.MHz())
            .expect("failed to create hub75");

    let mut count = 0u32;
    let mut start = Instant::now();

    // keep the frame buffer in an option so we can swap it
    let mut fb = fb;

    loop {
        // if there is a new buffer available, swap it and send the old one
        if rx.signaled() {
            let new_fb = rx.wait().await;
            tx.signal(fb);
            fb = new_fb;
        }

        hub75.render_async(fb).await.expect("transfer failed");

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

#[main]
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
    info!("SIZE: {}", SIZE);
    let mut config = esp_hal::Config::default();
    config.cpu_clock = CpuClock::max();
    let peripherals = esp_hal::init(config);
    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let software_interrupt = sw_ints.software_interrupt2;
    let dma = Dma::new(peripherals.DMA);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    info!("init embassy");
    esp_hal_embassy::init(timg0.timer0);

    static TX: FrameBufferExchange = FrameBufferExchange::new();
    static RX: FrameBufferExchange = FrameBufferExchange::new();
    info!("init framebuffers");
    let fb0 = mk_static!(FBType, FBType::new());
    let fb1 = mk_static!(FBType, FBType::new());
    fb0.clear();
    fb1.clear();

    info!("fb0: {:?}", fb0);
    info!("fb1: {:?}", fb1);

    let display_peripherals = DisplayPeripherals {
        parl_io: peripherals.PARL_IO,
        dma_channel: dma.channel0,
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

    // run hub75 as high priority task (interrupt executor)
    let hp_executor = mk_static!(
        InterruptExecutor<2>,
        InterruptExecutor::new(software_interrupt)
    );
    let high_pri_spawner = hp_executor.start(Priority::Priority3);
    high_pri_spawner
        .spawn(hub75_task(display_peripherals, &RX, &TX, fb1))
        .ok();

    // display task runs as low priority task
    spawner.spawn(display_task(&TX, &RX, fb0)).ok();

    loop {
        if SIMPLE_COUNTER.fetch_add(1, Ordering::Relaxed) >= 99999 {
            SIMPLE_COUNTER.store(0, Ordering::Relaxed);
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
