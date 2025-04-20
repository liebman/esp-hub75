//! Framebuffer implementation for HUB75 LED matrix displays.
//!
//! This module provides two different framebuffer implementations optimized for
//! HUB75 LED matrix displays:
//!
//! 1. **Plain Implementation** (`plain` module)
//!    - No additional hardware requirements
//!    - Simpler implementation suitable for basic displays
//!
//! 2. **Latched Implementation** (`latched` module)
//!    - Requires external latch hardware for address lines
//!
//! Both implementations:
//! - Have configurable row and column dimensions
//! - Support different color depths through Binary Code Modulation (BCM)
//! - Implement the `ReadBuffer` trait for DMA compatibility
//!
//! # Examples
//!
//! ```rust
//! use embedded_graphics::pixelcolor::Rgb888;
//! use esp_hal::dma::ReadBuffer;
//!
//! use crate::framebuffer::FrameBuffer;
//! use crate::framebuffer::MutableFrameBuffer;
//!
//! // Create a framebuffer with specific dimensions
//! const ROWS: usize = 32;
//! const COLS: usize = 64;
//! const NROWS: usize = ROWS / 2;
//! const BITS: u8 = 8;
//! const FRAME_COUNT: usize = (1 << BITS) - 1;
//! ```

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::Rgb888;
use esp_hal::dma::ReadBuffer;

pub mod latched;
pub mod plain;

/// Color type used in the framebuffer
pub type Color = Rgb888;

/// Word size configuration for the framebuffer
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WordSize {
    /// 8-bit word size
    Eight,
    /// 16-bit word size
    Sixteen,
}

/// Computes the NROWS value from ROWS for DmaFrameBuffer
///
/// # Arguments
///
/// * `rows` - Total number of rows in the display
///
/// # Returns
///
/// Number of rows needed internally for DmaFrameBuffer
pub const fn compute_rows(rows: usize) -> usize {
    rows / 2
}

/// Computes the number of frames needed for a given bit depth
///
/// This is used to determine how many frames are needed to achieve
/// the desired color depth through Binary Code Modulation (BCM).
///
/// # Arguments
///
/// * `bits` - Number of bits per color channel
///
/// # Returns
///
/// Number of frames required for the given bit depth
pub const fn compute_frame_count(bits: u8) -> usize {
    (1usize << bits) - 1
}

/// Trait for read-only framebuffers
///
/// This trait defines the basic functionality required for a framebuffer
/// that can be read from and transferred via DMA.
///
/// # Type Parameters
///
/// * `ROWS` - Total number of rows in the display
/// * `COLS` - Number of columns in the display
/// * `NROWS` - Number of rows processed in parallel
/// * `BITS` - Number of bits per color channel
/// * `FRAME_COUNT` - Number of frames needed for BCM
pub trait FrameBuffer<
    const ROWS: usize,
    const COLS: usize,
    const NROWS: usize,
    const BITS: u8,
    const FRAME_COUNT: usize,
>: ReadBuffer
{
    /// Returns the word size configuration for this framebuffer
    fn get_word_size(&self) -> WordSize;
}

/// Trait for mutable framebuffers
///
/// This trait extends `FrameBuffer` with the ability to draw to the framebuffer
/// using the `embedded_graphics` drawing primitives.
///
/// # Type Parameters
///
/// * `ROWS` - Total number of rows in the display
/// * `COLS` - Number of columns in the display
/// * `NROWS` - Number of rows processed in parallel
/// * `BITS` - Number of bits per color channel
/// * `FRAME_COUNT` - Number of frames needed for BCM
pub trait MutableFrameBuffer<
    const ROWS: usize,
    const COLS: usize,
    const NROWS: usize,
    const BITS: u8,
    const FRAME_COUNT: usize,
>:
    FrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
    + DrawTarget<Color = Color, Error = core::convert::Infallible>
{
}
