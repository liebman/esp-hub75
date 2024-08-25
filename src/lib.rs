#![no_std]
#![feature(type_alias_impl_trait)]

use embedded_graphics::pixelcolor::Rgb888;

#[cfg(any(feature = "esp32s3", feature = "esp32c6"))]
pub mod framebuffer;
pub mod gpio;
#[cfg(feature = "esp32s3")]
pub mod lcd_cam;
#[cfg(feature = "esp32c6")]
pub mod parl_io;

pub type Color = Rgb888;
