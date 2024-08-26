#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::fmt;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;

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
use esp_hal::clock::ClockControl;
use esp_hal::clock::Clocks;
use esp_hal::cpu_control::CpuControl;
use esp_hal::dma::Dma;
use esp_hal::gpio::AnyPin;
use esp_hal::gpio::Io;
use esp_hal::interrupt::Priority;
use esp_hal::peripherals::Peripherals;
use esp_hal::peripherals::LCD_CAM;
use esp_hal::prelude::*;
use esp_hal::system::SystemControl;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::timer::ErasedTimer;
use esp_hal::timer::OneShotTimer;
use esp_hal_embassy::InterruptExecutor;
use esp_hub75::framebuffer::DmaFrameBuffer;
use esp_hub75::framebuffer::Entry;
use esp_hub75::lcd_cam::Hub75;
use esp_hub75::lcd_cam::Hub75Pins;
use esp_hub75::Color;
use heapless::String;
use static_cell::make_static;
use static_cell::StaticCell;

pub struct DisplayPeripherals<'a> {
    pub lcd_cam: LCD_CAM,
    pub dma_channel: esp_hal::dma::ChannelCreator<0>,
    pub red1: AnyPin<'a>,
    pub grn1: AnyPin<'a>,
    pub blu1: AnyPin<'a>,
    pub red2: AnyPin<'a>,
    pub grn2: AnyPin<'a>,
    pub blu2: AnyPin<'a>,
    pub addr0: AnyPin<'a>,
    pub addr1: AnyPin<'a>,
    pub addr2: AnyPin<'a>,
    pub addr3: AnyPin<'a>,
    pub addr4: AnyPin<'a>,
    pub blank: AnyPin<'a>,
    pub clock: AnyPin<'a>,
    pub latch: AnyPin<'a>,
}

const ROWS: usize = 64;
const COLS: usize = 64;
const BITS: u8 = 4;
const SIZE: usize = ROWS * COLS * (1 << BITS);

type Hub75Type = Hub75<'static, esp_hal::Async>;
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
    peripherals: DisplayPeripherals<'static>,
    clocks: &'static Clocks<'static>,
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    fb: &'static mut FBType,
) {
    let channel: esp_hal::dma::ChannelCreator<0> = peripherals.dma_channel;
    let (tx_descriptors, _) = esp_hal::dma_descriptors!(SIZE * size_of::<Entry>(), 0);

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
        Hub75Type::new_async(peripherals.lcd_cam, pins, channel, &clocks, tx_descriptors);

    let mut count = 0u32;
    let mut start = Instant::now();

    // keep the frame buffer in an option so we can swap it
    let mut fb = Some(fb);

    loop {
        // if there is a new buffer available, swap it and send the old one
        if rx.signaled() {
            let new_fb = rx.wait().await;
            let old_fb = fb.replace(new_fb).unwrap();
            tx.signal(old_fb);
        }

        if let Some(ref fb) = fb {
            hub75.render_async(fb).await;
        }

        count += 1;
        const FPS_INTERVAL: Duration = Duration::from_secs(1);
        if start.elapsed() > FPS_INTERVAL {
            REFRESH_RATE.store(count, Ordering::Relaxed);
            count = 0;
            start = Instant::now();
        }
    }
}

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let software_interrupt = system.software_interrupt_control.software_interrupt2;
    let cpu_control = CpuControl::new(peripherals.CPU_CTRL);

    let clocks = make_static!(ClockControl::max(system.clock_control).freeze());
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer0: ErasedTimer = timer_group0.timer0.into();
    let timer1: ErasedTimer = timer_group0.timer1.into();
    let timers = [OneShotTimer::new(timer0), OneShotTimer::new(timer1)];
    let timers = make_static!(timers);
    esp_hal_embassy::init(&clocks, timers);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let dma = Dma::new(peripherals.DMA);

    static TX: FrameBufferExchange = FrameBufferExchange::new();
    static RX: FrameBufferExchange = FrameBufferExchange::new();
    let fb0 = make_static!(FBType::new());
    let fb1 = make_static!(FBType::new());

    let display_peripherals = DisplayPeripherals {
        lcd_cam: peripherals.LCD_CAM,
        dma_channel: dma.channel0,
        red1: AnyPin::new(io.pins.gpio38),
        grn1: AnyPin::new(io.pins.gpio42),
        blu1: AnyPin::new(io.pins.gpio48),
        red2: AnyPin::new(io.pins.gpio47),
        grn2: AnyPin::new(io.pins.gpio2),
        blu2: AnyPin::new(io.pins.gpio21),
        addr0: AnyPin::new(io.pins.gpio14),
        addr1: AnyPin::new(io.pins.gpio46),
        addr2: AnyPin::new(io.pins.gpio13),
        addr3: AnyPin::new(io.pins.gpio9),
        addr4: AnyPin::new(io.pins.gpio3),
        blank: AnyPin::new_inverted(io.pins.gpio11), /* inverted to prevent ghosting when
                                                      * DMA stopped! */
        clock: AnyPin::new(io.pins.gpio12),
        latch: AnyPin::new(io.pins.gpio10),
    };

    // run hub75 and display on second core
    let cpu1_fnctn = {
        move || {
            static HP_EXECUTOR: StaticCell<InterruptExecutor<2>> = StaticCell::new();
            let executor = InterruptExecutor::new(software_interrupt);
            let hp_executor = HP_EXECUTOR.init(executor);
            let high_pri_spawner = hp_executor.start(Priority::Priority3);

            // hub75 runs as high priority task
            high_pri_spawner
                .spawn(hub75_task(display_peripherals, clocks, &RX, &TX, fb1))
                .ok();

            static LP_EXECUTOR: StaticCell<esp_hal_embassy::Executor> = StaticCell::new();
            let lp_executor = LP_EXECUTOR.init(esp_hal_embassy::Executor::new());
            // display task runs as low priority task
            lp_executor.run(|spawner| {
                spawner.spawn(display_task(&TX, &RX, fb0)).ok();
            });
        }
    };

    const DISPLAY_STACK_SIZE: usize = 8192;
    static APP_CORE_STACK: StaticCell<esp_hal::cpu_control::Stack<DISPLAY_STACK_SIZE>> =
        StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init(esp_hal::cpu_control::Stack::new());
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
