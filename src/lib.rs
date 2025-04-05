#![no_std]
#![feature(type_alias_impl_trait)]

use embedded_graphics::pixelcolor::Rgb888;
use esp_hal::gpio::AnyPin;

pub mod framebuffer;
#[cfg(feature = "esp32")]
pub mod i2s_parallel;
#[cfg(feature = "esp32")]
pub mod i2s_parallel_latch;
#[cfg(feature = "esp32s3")]
pub mod lcd_cam;
#[cfg(feature = "esp32s3")]
pub mod lcd_cam_latch;
#[cfg(feature = "esp32c6")]
pub mod parl_io;

pub type Color = Rgb888;

pub struct Hub75Pins<'d> {
    pub red1: AnyPin<'d>,
    pub grn1: AnyPin<'d>,
    pub blu1: AnyPin<'d>,
    pub red2: AnyPin<'d>,
    pub grn2: AnyPin<'d>,
    pub blu2: AnyPin<'d>,
    pub addr0: AnyPin<'d>,
    pub addr1: AnyPin<'d>,
    pub addr2: AnyPin<'d>,
    pub addr3: AnyPin<'d>,
    pub addr4: AnyPin<'d>,
    pub blank: AnyPin<'d>,
    pub clock: AnyPin<'d>,
    pub latch: AnyPin<'d>,
    #[cfg(all(feature = "esp32c6", feature = "valid-pin"))]
    pub valid: AnyPin<'d>,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Hub75Error {
    Dma(esp_hal::dma::DmaError),
    DmaBuf(esp_hal::dma::DmaBufError),
    #[cfg(feature = "esp32c6")]
    ParlIo(esp_hal::parl_io::Error),
    #[cfg(feature = "esp32s3")]
    I8080(esp_hal::lcd_cam::lcd::i8080::ConfigError),
}

impl From<esp_hal::dma::DmaError> for Hub75Error {
    fn from(e: esp_hal::dma::DmaError) -> Self {
        Hub75Error::Dma(e)
    }
}

impl From<esp_hal::dma::DmaBufError> for Hub75Error {
    fn from(e: esp_hal::dma::DmaBufError) -> Self {
        Hub75Error::DmaBuf(e)
    }
}

#[cfg(feature = "esp32c6")]
impl From<esp_hal::parl_io::Error> for Hub75Error {
    fn from(e: esp_hal::parl_io::Error) -> Self {
        Hub75Error::ParlIo(e)
    }
}
