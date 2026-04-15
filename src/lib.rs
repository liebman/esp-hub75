//! # ESP-HUB75
//!
//! A Rust driver for HUB75-style LED matrix panels on ESP32-series
//! microcontrollers. HUB75 is a standard interface for driving large, bright,
//! and colorful RGB LED displays, commonly used in digital signage and art
//! installations.
//!
//! This library provides a high-performance implementation that uses Direct
//! Memory Access (DMA) to drive the display with minimal CPU overhead. It is
//! designed to work with a variety of ESP32 models, using the most efficient
//! peripheral available on each chip:
//!
//! - **ESP32-S3**: Uses the LCD_CAM peripheral (via `esp-hal` or `esp-idf`)
//! - **ESP32-P4**: Uses the LCD_CAM peripheral (via `esp-idf`)
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
//! - `esp32s3`: Enable support for the ESP32-S3 (bare-metal, via `esp-hal`)
//! - `esp32s3-idf`: Enable support for the ESP32-S3 running ESP-IDF (std/FreeRTOS)
//! - `esp32p4-idf`: Enable support for the ESP32-P4 running ESP-IDF (std/FreeRTOS)
//! - `esp32c6`: Enable support for the ESP32-C6
//! - `defmt`: Enable logging with `defmt`
//! - `log`: Enable logging with the `log` crate
//! - `invert-blank`: Invert the blank signal, required for some controller boards.
//! - `invert-clock`: Invert the clock signal. By default the driver outputs data
//!   that changes on the falling edge of CLK so that it is stable when the panel
//!   latches on the rising edge. Enable this feature if your panel requires the
//!   opposite polarity.
//! - `skip-black-pixels`: Forwards to the `hub75-framebuffer` crate, enabling an
//!   optimization that skips writing black pixels to the framebuffer.
//! - `iram`: Place the driver's hot-path (render / DMA wait functions) in
//!   Instruction RAM (IRAM) to avoid flash-cache stalls (for example during
//!   Wi-Fi, PSRAM, or SPI-flash activity) that can cause visible flicker.
//!   Enabling this feature consumes roughly 5–10 KiB of IRAM. Not applicable
//!   when using `esp32s3-idf`.
//!
//! ## Safety
//!
//! This crate uses `unsafe` code to interface with hardware peripherals, but it
//! exposes a safe, high-level API.

// no_std for all bare-metal targets; std is available when using an IDF backend
#![cfg_attr(not(feature = "idf_backend"), no_std)]
#![warn(missing_docs)]

// esp-hal pin types are only used by the bare-metal backends
#[cfg(not(feature = "idf_backend"))]
use esp_hal::gpio::AnyPin;
pub use hub75_framebuffer as framebuffer;
#[cfg_attr(feature = "esp32", path = "i2s_parallel.rs")]
#[cfg_attr(feature = "esp32s3", path = "lcd_cam.rs")]
#[cfg_attr(feature = "esp32s3-idf", path = "lcd_cam_idf.rs")]
#[cfg_attr(feature = "esp32p4-idf", path = "lcd_cam_idf.rs")]
#[cfg_attr(feature = "esp32c6", path = "parl_io.rs")]
mod hub75;
pub use hub75::Hub75;
pub use hub75::Hub75Transfer;
/// The color type used by the HUB75 driver.
pub use hub75_framebuffer::Color;

/// Pin configuration for a HUB75 panel without an external address latch
/// (bare-metal `esp-hal` backends).
///
/// This configuration requires 16 bits of data per pixel transfer, as the row
/// address lines are driven directly along with the color data.
#[cfg(not(feature = "idf_backend"))]
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

/// Pin configuration for a HUB75 panel with an external address latch
/// (bare-metal `esp-hal` backends).
///
/// This configuration is more memory-efficient, requiring only 8 bits of data
/// per pixel transfer. The row address is set once per row and held by an
/// external latch on the controller board. For an example of a latch circuit,
/// see the [`hub75-framebuffer` crate documentation](https://crates.io/crates/hub75-framebuffer)
/// and its [GitHub repository](https://github.com/liebman/hub75-framebuffer).
#[cfg(not(feature = "idf_backend"))]
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

/// Pin configuration for a HUB75 panel without an external address latch,
/// for use with the ESP-IDF backend (`esp32s3-idf` or `esp32p4-idf` feature).
///
/// All pins are specified as raw GPIO numbers (`i32`).
/// Use `.pin()` from `esp_idf_hal::gpio::Pin` to convert typed pin handles
/// to numbers, e.g. `peripherals.pins.gpio38.pin()`.
///
/// Pin layout maps to the same DATA bus positions as the bare-metal
/// `esp32s3` backend:
/// DATA[0..4] = addr0-4, DATA[5] = latch, DATA[8] = blank (active-low),
/// DATA[9..14] = R1/G1/B1/R2/G2/B2.
///
/// **`dc_gpio`**: The ESP-IDF I80 bus driver requires a Data/Command (D/C) pin
/// even though HUB75 panels have no such signal. Assign any spare GPIO; the
/// driver drives it HIGH throughout every color transfer and the panel ignores
/// it. The pin does not need to be connected to anything on the panel side.
#[cfg(feature = "idf_backend")]
#[derive(Copy, Clone)]
pub struct Hub75IdfPins16 {
    /// Red data line for the upper half of the display
    pub red1: i32,
    /// Green data line for the upper half of the display
    pub grn1: i32,
    /// Blue data line for the upper half of the display
    pub blu1: i32,
    /// Red data line for the lower half of the display
    pub red2: i32,
    /// Green data line for the lower half of the display
    pub grn2: i32,
    /// Blue data line for the lower half of the display
    pub blu2: i32,
    /// Address line 0 for row selection
    pub addr0: i32,
    /// Address line 1 for row selection
    pub addr1: i32,
    /// Address line 2 for row selection
    pub addr2: i32,
    /// Address line 3 for row selection
    pub addr3: i32,
    /// Address line 4 for row selection
    pub addr4: i32,
    /// Blank signal to control display output (driven active-low by the driver)
    pub blank: i32,
    /// Clock / WR signal for data synchronization
    pub clock: i32,
    /// Latch signal to update display data
    pub latch: i32,
    /// Dummy D/C pin required by the IDF I80 bus driver (any spare GPIO).
    /// The panel does not use this signal.
    pub dc: i32,
}

/// Pin configuration for a HUB75 panel with an external address latch,
/// for use with the ESP-IDF backend (`esp32s3-idf` or `esp32p4-idf` feature).
///
/// All pins are specified as raw GPIO numbers (`i32`).
/// Use `.pin()` from `esp_idf_hal::gpio::Pin` to convert typed pin handles
/// to numbers, e.g. `peripherals.pins.gpio38.pin()`.
///
/// Pin layout: DATA[0..5] = R1/G1/B1/R2/G2/B2, DATA[6] = latch, DATA[7] =
/// blank.
///
/// **`dc_gpio`**: The ESP-IDF I80 bus driver requires a Data/Command (D/C) pin
/// even though HUB75 panels have no such signal. Assign any spare GPIO; the
/// driver drives it HIGH throughout every color transfer and the panel ignores
/// it. The pin does not need to be connected to anything on the panel side.
#[cfg(feature = "idf_backend")]
pub struct Hub75IdfPins8 {
    /// Red data line for the upper half of the display
    pub red1: i32,
    /// Green data line for the upper half of the display
    pub grn1: i32,
    /// Blue data line for the upper half of the display
    pub blu1: i32,
    /// Red data line for the lower half of the display
    pub red2: i32,
    /// Green data line for the lower half of the display
    pub grn2: i32,
    /// Blue data line for the lower half of the display
    pub blu2: i32,
    /// Blank signal to control display output
    pub blank: i32,
    /// Clock / WR signal for data synchronization
    pub clock: i32,
    /// Latch signal to update display data
    pub latch: i32,
    /// Dummy D/C pin required by the IDF I80 bus driver (any spare GPIO).
    /// The panel does not use this signal.
    pub dc: i32,
}

/// Trait for ESP-IDF pin configurations.
///
/// Implemented by [`Hub75IdfPins16`] and [`Hub75IdfPins8`].
/// Provides the GPIO numbers and bus width needed to configure the
/// ESP-IDF I80 LCD bus.
#[cfg(feature = "idf_backend")]
pub trait Hub75IdfPins {
    /// Number of data bus lines (8 or 16).
    fn bus_width(&self) -> usize;
    /// GPIO number for the WR/clock signal.
    fn wr_gpio(&self) -> i32;
    /// GPIO number for the D/C line (required by IDF, ignored by the panel).
    fn dc_gpio(&self) -> i32;
    /// Data GPIO numbers. Unused positions must be set to `-1` (GPIO_NUM_NC).
    fn data_gpios(&self) -> [i32; 16];
    /// GPIO number for the BLANK (OE) signal.
    fn blank_gpio(&self) -> i32;
    /// Whether the BLANK GPIO output should be hardware-inverted via the GPIO
    /// matrix. In 16-bit mode this is always `true` (the framebuffer encodes
    /// BLANK active-high; the panel requires OE active-low). In 8-bit mode
    /// this follows the `invert-blank` feature.
    fn invert_blank(&self) -> bool;
}

/// A trait for applying a set of HUB75 pins onto the specific ESP32 peripheral
///
/// This allows the driver to abstract over the differences in pin
/// configurations between peripherals (I2S, LCD-CAM, PARL_IO) and between
/// direct-drive (16-bit) and latched (8-bit) HUB75 controller boards.
#[cfg(feature = "esp32s3")]
pub trait Hub75Pins<'d> {
    /// Apply pin configuration to the i8080 driver.
    fn apply<DM: esp_hal::DriverMode>(
        self,
        i8080: esp_hal::lcd_cam::lcd::i8080::I8080<'d, DM>,
    ) -> esp_hal::lcd_cam::lcd::i8080::I8080<'d, DM>;
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
#[cfg(not(any(feature = "esp32s3", feature = "idf_backend")))]
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
/// This enum consolidates errors from the underlying HAL DMA, peripheral,
/// and buffer management modules into a single type for easier error handling.
#[derive(Debug)]
#[cfg_attr(
    all(feature = "defmt", not(feature = "idf_backend")),
    derive(defmt::Format)
)]
pub enum Hub75Error {
    /// Error occurred during DMA transfer operations (bare-metal backends)
    #[cfg(not(feature = "idf_backend"))]
    Dma(esp_hal::dma::DmaError),
    /// Error occurred while managing DMA buffers (bare-metal backends)
    #[cfg(not(feature = "idf_backend"))]
    DmaBuf(esp_hal::dma::DmaBufError),
    /// Error from the PARL_IO peripheral (ESP32-C6 only)
    #[cfg(feature = "esp32c6")]
    ParlIo(esp_hal::parl_io::Error),
    /// Configuration error for the PARL_IO peripheral (ESP32-C6 only)
    #[cfg(feature = "esp32c6")]
    ConfigError(esp_hal::parl_io::ConfigError),
    /// Configuration error for the I8080 interface (ESP32-S3 bare-metal only)
    #[cfg(feature = "esp32s3")]
    I8080(esp_hal::lcd_cam::lcd::i8080::ConfigError),
    /// Error from the ESP-IDF LCD driver (IDF backends only)
    #[cfg(feature = "idf_backend")]
    Idf(esp_idf_sys::EspError),
}

#[cfg(not(feature = "idf_backend"))]
impl From<esp_hal::dma::DmaError> for Hub75Error {
    fn from(e: esp_hal::dma::DmaError) -> Self {
        Self::Dma(e)
    }
}

#[cfg(not(feature = "idf_backend"))]
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

#[cfg(feature = "idf_backend")]
impl From<esp_idf_sys::EspError> for Hub75Error {
    fn from(e: esp_idf_sys::EspError) -> Self {
        Self::Idf(e)
    }
}
