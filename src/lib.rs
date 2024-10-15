#![no_std]
#![feature(type_alias_impl_trait)]

use embedded_graphics::pixelcolor::Rgb888;
use esp_hal::gpio::AnyPin;

pub mod framebuffer;
#[cfg(feature = "esp32s3")]
pub mod lcd_cam;
#[cfg(feature = "esp32c6")]
pub mod parl_io;
#[cfg(feature = "esp32")]
pub mod i2s_parallel;

pub type Color = Rgb888;
pub use fugit::HertzU32;

pub struct Hub75Pins {
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
    #[cfg(all(feature = "esp32c6", feature = "valid-pin"))]
    pub valid: AnyPin,
}
