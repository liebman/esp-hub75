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
//! use embedded_graphics::geometry::Point;
//! use embedded_graphics::mono_font::ascii::FONT_5X7;
//! use embedded_graphics::mono_font::MonoTextStyleBuilder;
//! use embedded_graphics::text::Alignment;
//! use embedded_graphics::text::Text;
//! use embedded_graphics::Drawable;
//! use esp_hal::clock::CpuClock;
//! use esp_hal::time::Rate;
//! use esp_hub75::framebuffer::compute_frame_count;
//! use esp_hub75::framebuffer::compute_rows;
//! use esp_hub75::framebuffer::plain::DmaFrameBuffer;
//! use esp_hub75::Color;
//! use esp_hub75::Hub75;
//! use esp_hub75::Hub75Pins16;
//!
//! const ROWS: usize = 64;
//! const COLS: usize = 64;
//! const BITS: u8 = 4;
//! const NROWS: usize = compute_rows(ROWS);
//! const FRAME_COUNT: usize = compute_frame_count(BITS);
//!
//! type FBType = DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>;
//!
//! #[main]
//! fn main() -> ! {
//!     let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
//!
//!     let (_, tx_descriptors) = esp_hal::dma_descriptors!(0, size_of::<FBType>());
//!
//!     let pins = Hub75Pins16 {
//!         red1: peripherals.GPIO38.degrade(),
//!         grn1: peripherals.GPIO42.degrade(),
//!         blu1: peripherals.GPIO48.degrade(),
//!         red2: peripherals.GPIO47.degrade(),
//!         grn2: peripherals.GPIO2.degrade(),
//!         blu2: peripherals.GPIO21.degrade(),
//!         addr0: peripherals.GPIO14.degrade(),
//!         addr1: peripherals.GPIO46.degrade(),
//!         addr2: peripherals.GPIO13.degrade(),
//!         addr3: peripherals.GPIO9.degrade(),
//!         addr4: peripherals.GPIO3.degrade(),
//!         blank: peripherals.GPIO11.degrade(),
//!         clock: peripherals.GPIO12.degrade(),
//!         latch: peripherals.GPIO10.degrade(),
//!     };
//!
//!     let mut hub75 = Hub75::new(
//!         peripherals.LCD_CAM,
//!         pins,
//!         peripherals.DMA_CH0,
//!         tx_descriptors,
//!         Rate::from_mhz(20),
//!     )
//!     .expect("failed to create Hub75!");
//!
//!     let mut fb = FBType::new();
//!     fb.clear();
//!     let text_style = MonoTextStyleBuilder::new()
//!         .font(&FONT_5X7)
//!         .text_color(Color::YELLOW)
//!         .background_color(Color::BLACK)
//!         .build();
//!     let point = Point::new(32, 31);
//!     Text::with_alignment("Hello, World!", point, text_style, Alignment::Center)
//!         .draw(&mut fb)
//!         .expect("failed to draw text");
//!     loop {
//!         let xfer = hub75
//!             .render(&fb)
//!             .map_err(|(e, _hub75)| e)
//!             .expect("failed to start render!");
//!         let (result, new_hub75) = xfer.wait();
//!         hub75 = new_hub75;
//!         result.expect("transfer failed");
//!     }
//! }
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
//! This crate requires `#![no_std]` and is designed for embedded use. It uses
//! unsafe code internally to interface with hardware peripherals, but provides
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
