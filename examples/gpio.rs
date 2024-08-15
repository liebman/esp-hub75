#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use esp_backtrace as _;
use esp_hal::clock::ClockControl;
use esp_hal::gpio::Io;
use esp_hal::gpio::Level;
use esp_hal::gpio::Output;
use esp_hal::peripherals::Peripherals;
use esp_hal::prelude::*;
use esp_hal::system::SystemControl;
use esp_hub75::Color;
use esp_hub75::gpio::FrameBuffer;
use esp_hub75::gpio::Hub75;
use static_cell::make_static;
use static_cell::StaticCell;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let _clocks = make_static!(ClockControl::max(system.clock_control).freeze());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    static FRAME_BUFFER: StaticCell<FrameBuffer<64, 64>> = StaticCell::new();
    let buffer = FRAME_BUFFER.init(FrameBuffer::new());

    // control pins
    let blank = Output::new(io.pins.gpio11, Level::High);
    let clock = Output::new(io.pins.gpio12, Level::Low);
    let latch = Output::new(io.pins.gpio10, Level::Low);

    // address pins
    let addr0 = Output::new(io.pins.gpio14, Level::Low);
    let addr1 = Output::new(io.pins.gpio46, Level::Low);
    let addr2 = Output::new(io.pins.gpio13, Level::Low);
    let addr3 = Output::new(io.pins.gpio9, Level::Low);
    let addr4 = Output::new(io.pins.gpio3, Level::Low);

    // color pins
    let red1 = Output::new(io.pins.gpio38, Level::Low);
    let grn1 = Output::new(io.pins.gpio42, Level::Low);
    let blu1 = Output::new(io.pins.gpio48, Level::Low);
    let red2 = Output::new(io.pins.gpio47, Level::Low);
    let grn2 = Output::new(io.pins.gpio2, Level::Low);
    let blu2 = Output::new(io.pins.gpio21, Level::Low);

    let mut hub75: Hub75<_, _, _, _, _, _, _, _, _, _, _, _, _, _, 64, 64> = Hub75::new(
        addr0, addr1, addr2, addr3, addr4, red1, grn1, blu1, red2, grn2, blu2, clock, latch, blank,
    );
    hub75.blank(false);

    for x in 0..64 {
        for y in 0..64 {
            let intensity = (y as f32 / 64.0 * 255.0) as u8;
            let color = match x {
                0..=21 => Color::new(intensity, 0, 0),
                22..=43 => Color::new(0, intensity, 0),
                44..=63 => Color::new(0, 0, intensity),
                _ => unreachable!(),
            };
            buffer.set_pixel(x, y, color);
        }
    }
    loop {
        hub75.display(buffer);
    }
}
