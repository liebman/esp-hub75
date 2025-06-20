//! # ESP-HUB75
//!
//! A `no-std` Rust driver for HUB75-style LED matrix panels on ESP32-series
//! microcontrollers. HUB75 is a standard interface for driving large, bright,
//! and colorful RGB LED displays, commonly used in digital signage and art
//! installations.
//!
//! This library provides a high-performance implementation that uses Direct
//! Memory Access (DMA) to drive the display with minimal CPU overhead. It is
//! designed to work with a variety of ESP32 models, using the most efficient
//! peripheral available on each chip:
//!
//! - **ESP32-S3**: Uses the LCD_CAM peripheral
//! - **ESP32-C6**: Uses the PARL_IO peripheral
//! - **ESP32**: Uses the I2S peripheral in parallel mode
//!
//! ## Usage
//!
//! Here is an example of how to initialize the driver for an ESP32-S3:
//!
//! ```rust,no_run
#![doc = include_str!("../examples/hello_lcd_cam.rs")]
//! ```
//! 
//! ## Crate Features
//!
//! - `esp32`: Enable support for the ESP32
//! - `esp32s3`: Enable support for the ESP32-S3
//! - `esp32c6`: Enable support for the ESP32-C6
//! - `defmt`: Enable logging with `defmt`
//! - `log`: Enable logging with the `log` crate
//! - `invert-blank`: Invert the blank signal, required for some controller boards.
//! - `skip-black-pixels`: Forwards to the `hub75-framebuffer` crate, enabling an
//!   optimization that skips writing black pixels to the framebuffer.
//!
//! ## Safety
//!
//! This crate uses `unsafe` code to interface with hardware peripherals, but it
//! exposes a safe, high-level API.

#![no_std]
#![warn(missing_docs)]

use esp_hal::gpio::AnyPin;
pub use hub75_framebuffer as framebuffer;
#[cfg_attr(feature = "esp32", path = "i2s_parallel.rs")]
#[cfg_attr(feature = "esp32s3", path = "lcd_cam.rs")]
#[cfg_attr(feature = "esp32c6", path = "parl_io.rs")]
mod hub75;
pub use hub75::Hub75;
pub use hub75::Hub75Transfer;
/// The color type used by the HUB75 driver.
pub use hub75_framebuffer::Color;

/// Pin configuration for a HUB75 panel without an external address latch.
///
/// This configuration requires 16 bits of data per pixel transfer, as the row
/// address lines are driven directly along with the color data.
pub struct Hub75Pins16<'d> {
    /// Red data line for the upper half of the display
    pub red1: AnyPin<'d>,
    /// Green data line for the upper half of the display
    pub grn1: AnyPin<'d>,
    /// Blue data line for the upper half of the display
    pub blu1: AnyPin<'d>,
    /// Red data line for the lower half of the display
    pub red2: AnyPin<'d>,
    /// Green data line for the lower half of the display
    pub grn2: AnyPin<'d>,
    /// Blue data line for the lower half of the display
    pub blu2: AnyPin<'d>,
    /// Address line 0 for row selection
    pub addr0: AnyPin<'d>,
    /// Address line 1 for row selection
    pub addr1: AnyPin<'d>,
    /// Address line 2 for row selection
    pub addr2: AnyPin<'d>,
    /// Address line 3 for row selection
    pub addr3: AnyPin<'d>,
    /// Address line 4 for row selection
    pub addr4: AnyPin<'d>,
    /// Blank signal to control display output
    pub blank: AnyPin<'d>,
    /// Clock signal for data synchronization
    pub clock: AnyPin<'d>,
    /// Latch signal to update display data
    pub latch: AnyPin<'d>,
}

/// Pin configuration for a HUB75 panel with an external address latch.
///
/// This configuration is more memory-efficient, requiring only 8 bits of data
/// per pixel transfer. The row address is set once per row and held by an
/// external latch on the controller board. For an example of a latch circuit,
/// see the [`hub75-framebuffer` crate documentation](https://crates.io/crates/hub75-framebuffer)
/// and its [GitHub repository](https://github.com/liebman/hub75-framebuffer).
pub struct Hub75Pins8<'d> {
    /// Red data line for the upper half of the display
    pub red1: AnyPin<'d>,
    /// Green data line for the upper half of the display
    pub grn1: AnyPin<'d>,
    /// Blue data line for the upper half of the display
    pub blu1: AnyPin<'d>,
    /// Red data line for the lower half of the display
    pub red2: AnyPin<'d>,
    /// Green data line for the lower half of the display
    pub grn2: AnyPin<'d>,
    /// Blue data line for the lower half of the display
    pub blu2: AnyPin<'d>,
    /// Blank signal to control display output
    pub blank: AnyPin<'d>,
    /// Clock signal for data synchronization
    pub clock: AnyPin<'d>,
    /// Latch signal to update display data
    pub latch: AnyPin<'d>,
}

/// A trait for converting a set of HUB75 pins into the required format for a
/// specific ESP32 peripheral.
///
/// This allows the driver to abstract over the differences in pin
/// configurations between peripherals (I2S, LCD-CAM, PARL_IO) and between
/// direct-drive (16-bit) and latched (8-bit) HUB75 controller boards.
///
/// # Type Parameters
/// * `T` - The target pin configuration type for the specific peripheral.
pub trait Hub75Pins<'d, T> {
    /// Converts the high-level pin definition into the peripheral-specific
    /// format needed by the driver.
    ///
    /// # Returns
    /// A tuple containing:
    /// 1. The converted pin configuration for the specific peripheral.
    /// 2. The clock pin used for synchronization.
    fn convert_pins(self) -> (T, AnyPin<'d>);
}

/// Represents errors that can occur during HUB75 driver operations.
///
/// This enum consolidates errors from the underlying `esp-hal` DMA, peripheral,
/// and buffer management modules into a single type for easier error handling.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Hub75Error {
    /// Error occurred during DMA transfer operations
    Dma(esp_hal::dma::DmaError),
    /// Error occurred while managing DMA buffers
    DmaBuf(esp_hal::dma::DmaBufError),
    /// Error from the PARL_IO peripheral (ESP32-C6 only)
    #[cfg(feature = "esp32c6")]
    ParlIo(esp_hal::parl_io::Error),
    /// Configuration error for the PARL_IO peripheral (ESP32-C6 only)
    #[cfg(feature = "esp32c6")]
    ConfigError(esp_hal::parl_io::ConfigError),
    /// Configuration error for the I8080 interface (ESP32-S3 only)
    #[cfg(feature = "esp32s3")]
    I8080(esp_hal::lcd_cam::lcd::i8080::ConfigError),
}

impl From<esp_hal::dma::DmaError> for Hub75Error {
    fn from(e: esp_hal::dma::DmaError) -> Self {
        Self::Dma(e)
    }
}

impl From<esp_hal::dma::DmaBufError> for Hub75Error {
    fn from(e: esp_hal::dma::DmaBufError) -> Self {
        Self::DmaBuf(e)
    }
}

#[cfg(feature = "esp32c6")]
impl From<esp_hal::parl_io::Error> for Hub75Error {
    fn from(e: esp_hal::parl_io::Error) -> Self {
        Self::ParlIo(e)
    }
}

#[cfg(feature = "esp32c6")]
impl From<esp_hal::parl_io::ConfigError> for Hub75Error {
    fn from(e: esp_hal::parl_io::ConfigError) -> Self {
        Self::ConfigError(e)
    }
}
