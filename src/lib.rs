//! # ESP-HUB75
//!
//! A high-performance driver for HUB75 RGB LED matrix displays on ESP32
//! microcontrollers.
//!
//! ## Features
//!
//! - Support for multiple ESP32 variants
//! - High-performance DMA-based data transfer
//! - supports both async and blocking operation
//!
//! ## Supported Hardware
//!
//! - ESP32: Uses I2S parallel mode
//! - ESP32-S3: Uses LCD-CAM peripheral
//! - ESP32-C6: Uses PARL_IO peripheral
//!
//! ## Usage (example for ESP32-S3, others are similar)
//!
//! ```rust,no_run
#![doc = include_str!("../examples/hello_lcd_cam.rs")]
//! ```
//!
//! ## Crate Features
//!
//! - `esp32`: Enable support for ESP32
//! - `esp32s3`: Enable support for ESP32-S3
//! - `esp32c6`: Enable support for ESP32-C6
//! - `defmt`: Enable defmt logging support
//! - `log`: Enable log logging support
//!
//! ## Safety
//!
//! This crate uses unsafe code internally to interface with hardware peripherals, but provides
//! a safe public API.

#![no_std]
#![warn(missing_docs)]

use embedded_graphics::pixelcolor::Rgb888;
use esp_hal::gpio::AnyPin;

pub mod framebuffer;
#[cfg_attr(feature = "esp32", path = "i2s_parallel.rs")]
#[cfg_attr(feature = "esp32s3", path = "lcd_cam.rs")]
#[cfg_attr(feature = "esp32c6", path = "parl_io.rs")]
mod hub75;
pub use hub75::Hub75;
pub use hub75::Hub75Transfer;

/// The color type used by the HUB75 driver.
pub type Color = Rgb888;

/// Pin configuration HUB75 LED matrix displays using direct signals.
///
/// This structure defines the pin mapping for plain HUB75 displays. It includes
/// all necessary control signals and data lines for driving the display.
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

/// Pin configuration HUB75 LED matrix displays with an external latch.
///
/// This structure defines the pin mapping for HUB75 displays with an external
/// latch on the controller board. The external latch holds row address
/// selection, allowing for a more memory-efficient implementation with 8-bit
/// entries instead of 16-bit.
///
/// This configuration is used with controller boards that have
/// hardware latch support, which reduces memory requirements by handling row
/// addressing externally.
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

/// Trait for converting HUB75 pin configurations into the appropriate
/// peripheral-specific format.
///
/// This trait provides a common interface for converting different HUB75 pin
/// configurations (8-bit or 16-bit) into the format required by the specific
/// ESP32 peripheral being used (I2S, LCD-CAM, or PARL_IO).
///
/// # Type Parameters
/// * `T` - The target pin configuration type for the specific peripheral
pub trait Hub75Pins<'d, T> {
    /// Converts the HUB75 pin configuration into the peripheral-specific
    /// format.
    ///
    /// # Returns
    /// A tuple containing:
    /// 1. The converted pin configuration for the specific peripheral
    /// 2. The clock pin used for synchronization
    fn convert_pins(self) -> (T, AnyPin<'d>);
}

/// Error type for HUB75 display operations.
///
/// This enum represents all possible errors that can occur when working with
/// HUB75 displays, including DMA transfer errors, buffer management errors,
/// and peripheral-specific errors.
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
