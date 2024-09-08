#![no_std]
#![feature(type_alias_impl_trait)]

use embedded_graphics::pixelcolor::Rgb888;
use esp_hal::gpio::ErasedPin;

#[cfg(any(feature = "esp32s3", feature = "esp32c6"))]
pub mod framebuffer;
#[cfg(feature = "esp32s3")]
pub mod lcd_cam;
#[cfg(feature = "esp32c6")]
pub mod parl_io;

pub type Color = Rgb888;

pub struct Hub75Pins {
    pub red1: ErasedPin,
    pub grn1: ErasedPin,
    pub blu1: ErasedPin,
    pub red2: ErasedPin,
    pub grn2: ErasedPin,
    pub blu2: ErasedPin,
    pub addr0: ErasedPin,
    pub addr1: ErasedPin,
    pub addr2: ErasedPin,
    pub addr3: ErasedPin,
    pub addr4: ErasedPin,
    pub blank: ErasedPin,
    pub clock: ErasedPin,
    pub latch: ErasedPin,
}
