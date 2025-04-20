//! DMA-based framebuffer implementation for HUB75 LED panels.
//!
//! This module provides a high-performance framebuffer implementation that uses
//! DMA (Direct Memory Access) to efficiently transfer pixel data to HUB75 LED
//! panels. It supports RGB color and brightness control through multiple frames
//! using Binary Code Modulation (BCM).
//!
//! # Features
//! - DMA-based data transfer for optimal performance
//! - Support for RGB color with brightness control
//! - Multiple frame buffers for Binary Code Modulation (BCM)
//! - Integration with embedded-graphics for easy drawing
//!
//! # Brightness Control
//! Brightness is controlled through Binary Code Modulation (BCM):
//! - The number of brightness levels is determined by the `BITS` parameter
//! - Each additional bit doubles the number of brightness levels
//! - More bits provide better brightness resolution but require more memory
//! - Memory usage grows exponentially with the number of bits: `(2^BITS)-1`
//!   frames
//! - Example: 8 bits = 256 levels, 4 bits = 16 levels
//!
//! # Memory Usage
//! The framebuffer's memory usage is determined by:
//! - Panel size (ROWS × COLS)
//! - Number of brightness bits (BITS)
//! - Memory grows exponentially with bits: `(2^BITS)-1` frames
//! - 16-bit entries provide direct signal mapping but use more memory
//!
//! # Example
//! ```rust,no_run
//! use embedded_graphics::pixelcolor::RgbColor;
//! use embedded_graphics::prelude::*;
//! use embedded_graphics::primitives::Circle;
//! use embedded_graphics::primitives::Rectangle;
//! use esp_hub75::compute_frame_count;
//! use esp_hub75::compute_rows;
//! use esp_hub75::Color;
//! use esp_hub75::DmaFrameBuffer;
//!
//! // Create a framebuffer for a 64x32 panel with 3-bit color depth
//! const ROWS: usize = 32;
//! const COLS: usize = 64;
//! const BITS: u8 = 3; // Color depth (8 brightness levels, 7 frames)
//! const NROWS: usize = compute_rows(ROWS); // Number of rows per scan
//! const FRAME_COUNT: usize = compute_frame_count(BITS); // Number of frames for BCM
//!
//! let mut framebuffer = DmaFrameBuffer::<ROWS, COLS, NROWS, BITS, FRAME_COUNT>::new();
//!
//! // Clear the framebuffer
//! framebuffer.clear();
//!
//! // Draw a red rectangle
//! Rectangle::new(Point::new(10, 10), Size::new(20, 20))
//!     .into_styled(PrimitiveStyle::with_fill(Color::RED))
//!     .draw(&mut framebuffer)
//!     .unwrap();
//!
//! // Draw a blue circle
//! Circle::new(Point::new(40, 20), 10)
//!     .into_styled(PrimitiveStyle::with_fill(Color::BLUE))
//!     .draw(&mut framebuffer)
//!     .unwrap();
//! ```
//!
//! # Implementation Details
//! The framebuffer is organized to directly match the HUB75 connector signals:
//! - Each 16-bit word maps directly to the HUB75 control signals
//! - Color data (R, G, B) for two sub-pixels is stored in dedicated bits
//! - Control signals (output enable, latch, address) are mapped to specific
//!   bits
//! - Multiple frames are used to achieve Binary Code Modulation (BCM)
//! - DMA transfers the data directly to the panel without any transformation
//!
//! # Safety
//! This implementation uses unsafe code for DMA operations. The framebuffer
//! must be properly aligned in memory and the DMA configuration must match the
//! buffer layout.

use core::convert::Infallible;

use bitfield::bitfield;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::prelude::Point;
use esp_hal::dma::ReadBuffer;

use super::Color;
use super::FrameBuffer;
use super::WordSize;

const BLANKING_DELAY: usize = 5;

bitfield! {
    /// A 16-bit word representing the HUB75 control signals for a single pixel.
    ///
    /// This structure directly maps to the HUB75 connector signals:
    /// - RGB color data for two sub-pixels (color0 and color1)
    /// - Panel control signals (output enable, latch, address)
    /// - Dummy bits for timing alignment
    ///
    /// The bit layout matches the HUB75 connector signals:
    /// - Bit 15: Dummy bit 2
    /// - Bit 14: Blue channel for color1
    /// - Bit 13: Green channel for color1
    /// - Bit 12: Red channel for color1
    /// - Bit 11: Blue channel for color0
    /// - Bit 10: Green channel for color0
    /// - Bit 9: Red channel for color0
    /// - Bit 8: Output enable
    /// - Bit 7: Dummy bit 1
    /// - Bit 6: Dummy bit 0
    /// - Bit 5: Latch signal
    /// - Bits 4-0: Row address
    #[derive(Clone, Copy, Default, PartialEq)]
    #[repr(transparent)]
    struct Entry(u16);
    impl Debug;
    dummy2, set_dummy2: 15;
    blu2, set_blu2: 14;
    grn2, set_grn2: 13;
    red2, set_red2: 12;
    blu1, set_blu1: 11;
    grn1, set_grn1: 10;
    red1, set_red1: 9;
    output_enable, set_output_enable: 8;
    dummy1, set_dummy1: 7;
    dummy0, set_dummy0: 6;
    latch, set_latch: 5;
    addr, set_addr: 4, 0;
}

#[cfg(feature = "defmt")]
impl defmt::Format for Entry {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Entry({=u16:#x})", self.0)
    }
}

impl Entry {
    const fn new() -> Self {
        Self(0)
    }

    fn set_color0<C: RgbColor>(&mut self, color: C, brightness: u8) {
        self.set_red1(color.r() >= brightness);
        self.set_grn1(color.g() >= brightness);
        self.set_blu1(color.b() >= brightness);
    }

    fn set_color1<C: RgbColor>(&mut self, color: C, brightness: u8) {
        self.set_red2(color.r() >= brightness);
        self.set_grn2(color.g() >= brightness);
        self.set_blu2(color.b() >= brightness);
    }
}

/// Represents a single row of pixels in the framebuffer.
///
/// Each row contains a fixed number of columns (`COLS`) and manages the timing
/// and control signals for the HUB75 panel. The row handles:
/// - Output enable timing to prevent ghosting
/// - Latch signal generation for row updates
/// - Row address management
/// - Color data for both sub-pixels
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(C)]
struct Row<const COLS: usize> {
    data: [Entry; COLS],
}

const fn map_index(i: usize) -> usize {
    #[cfg(feature = "esp32")]
    {
        i ^ 1
    }
    #[cfg(not(feature = "esp32"))]
    {
        i
    }
}

impl<const COLS: usize> Default for Row<COLS> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const COLS: usize> Row<COLS> {
    pub const fn new() -> Self {
        Self {
            data: [Entry::new(); COLS],
        }
    }

    pub fn format(&mut self, addr: u8, prev_addr: u8) {
        let mut entry = Entry::default();
        entry.set_addr(prev_addr as u16);
        entry.set_output_enable(false);
        for i in 0..COLS {
            // if we enable display too soon then we will have ghosting
            if i >= BLANKING_DELAY {
                entry.set_output_enable(true);
            }
            // last pixel, blank the display, open the latch, set the new row address
            // the latch will be closed on the first pixel of the next row.
            if i == COLS - 1 {
                entry.set_output_enable(false);
                entry.set_latch(true);
                entry.set_addr(addr as u16);
            }

            self.data[map_index(i)] = entry;
        }
    }

    pub fn set_color0(&mut self, col: usize, color: Color, brightness: u8) {
        let entry = &mut self.data[map_index(col)];
        entry.set_color0(color, brightness);
    }

    pub fn set_color1(&mut self, col: usize, color: Color, brightness: u8) {
        let entry = &mut self.data[map_index(col)];
        entry.set_color1(color, brightness);
    }
}

#[derive(Copy, Clone, Debug)]
#[repr(C)]
struct Frame<const ROWS: usize, const COLS: usize, const NROWS: usize> {
    rows: [Row<COLS>; NROWS],
}

impl<const ROWS: usize, const COLS: usize, const NROWS: usize> Frame<ROWS, COLS, NROWS> {
    pub const fn new() -> Self {
        Self {
            rows: [Row::new(); NROWS],
        }
    }

    pub fn format(&mut self) {
        for (addr, row) in self.rows.iter_mut().enumerate() {
            let prev_addr = if addr == 0 {
                NROWS as u8 - 1
            } else {
                addr as u8 - 1
            };
            row.format(addr as u8, prev_addr);
        }
    }

    pub fn set_pixel(&mut self, y: usize, x: usize, color: Color, brightness: u8) {
        let row = &mut self.rows[if y < NROWS { y } else { y - NROWS }];
        if y < NROWS {
            row.set_color0(x, color, brightness);
        } else {
            row.set_color1(x, color, brightness);
        }
    }
}

impl<const ROWS: usize, const COLS: usize, const NROWS: usize> Default
    for Frame<ROWS, COLS, NROWS>
{
    fn default() -> Self {
        Self::new()
    }
}

/// DMA-compatible framebuffer for HUB75 LED panels.
///
/// This is a framebuffer implementation that:
/// - Manages multiple frames for Binary Code Modulation (BCM)
/// - Provides DMA-compatible memory layout
/// - Implements the embedded-graphics DrawTarget trait
///
/// # Type Parameters
/// - `ROWS`: Total number of rows in the panel
/// - `COLS`: Number of columns in the panel
/// - `NROWS`: Number of rows per scan (typically half of ROWS)
/// - `BITS`: Color depth (1-8 bits)
/// - `FRAME_COUNT`: Number of frames used for Binary Code Modulation
///
/// # Helper Functions
/// Use these functions to compute the correct values:
/// - `esp_hub75::compute_frame_count(BITS)`: Computes the required number of
///   frames
/// - `esp_hub75::compute_rows(ROWS)`: Computes the number of rows per scan
///
/// # Memory Layout
/// The buffer is aligned to ensure efficient DMA transfers and contains:
/// - A 64-bit alignment field
/// - An array of frames, each containing the full panel data
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DmaFrameBuffer<
    const ROWS: usize,
    const COLS: usize,
    const NROWS: usize,
    const BITS: u8,
    const FRAME_COUNT: usize,
> {
    _align: u64,
    frames: [Frame<ROWS, COLS, NROWS>; FRAME_COUNT],
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > Default for DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    /// Create a new framebuffer.
    /// # Example
    /// ```rust,no_run
    /// const ROWS: usize = 32;
    /// const COLS: usize = 64;
    /// const BITS: u8 = 3; // Color depth (8 brightness levels, 7 frames)
    /// const NROWS: usize = compute_rows(ROWS); // Number of rows per scan
    /// const FRAME_COUNT: usize = compute_frame_count(BITS); // Number of frames for BCM
    ///
    /// let framebuffer = DmaFrameBuffer::<ROWS, COLS, NROWS, BITS, FRAME_COUNT>::new();
    /// ```
    pub const fn new() -> Self {
        assert!(BITS <= 8);

        Self {
            _align: 0,
            frames: [Frame::new(); FRAME_COUNT],
        }
    }

    /// This returns the size of the DMA buffer in bytes.  Its used to calculate the
    /// number of DMA descriptors needed.
    /// # Example
    /// ```rust,no_run
    /// const ROWS: usize = 32;
    /// const COLS: usize = 64;
    /// const BITS: u8 = 3; // Color depth (8 brightness levels, 7 frames)
    /// const NROWS: usize = compute_rows(ROWS); // Number of rows per scan
    /// const FRAME_COUNT: usize = compute_frame_count(BITS);
    ///
    /// type FBType = DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>;
    /// let (_, tx_descriptors) = esp_hal::dma_descriptors!(0, FBType::dma_buffer_size_bytes());
    /// ```
    pub const fn dma_buffer_size_bytes() -> usize {
        core::mem::size_of::<[Frame<ROWS, COLS, NROWS>; FRAME_COUNT]>()
    }

    /// Clear and format the framebuffer.
    /// Note:This must be called before the first use of the framebuffer!
    /// # Example
    /// ```rust,no_run
    /// let mut framebuffer = DmaFrameBuffer::<ROWS, COLS, NROWS, BITS, FRAME_COUNT>::new();
    /// framebuffer.clear();
    /// ```
    pub fn clear(&mut self) {
        for frame in self.frames.iter_mut() {
            frame.format();
        }
    }


    /// Set a pixel in the framebuffer.
    /// # Example
    /// ```rust,no_run
    /// let mut framebuffer = DmaFrameBuffer::<ROWS, COLS, NROWS, BITS, FRAME_COUNT>::new();
    /// framebuffer.clear();
    /// framebuffer.set_pixel(Point::new(10, 10), Color::RED);
    /// ```
    pub fn set_pixel(&mut self, p: Point, color: Color) {
        if p.x < 0 || p.y < 0 {
            return;
        }
        self.set_pixel_internal(p.x as usize, p.y as usize, color);
    }

    fn set_pixel_internal(&mut self, x: usize, y: usize, color: Color) {
        if x >= COLS || y >= ROWS {
            return;
        }
        // set the pixel in all frames
        for frame in 0..FRAME_COUNT {
            let brightness_step = 1 << (8 - BITS);
            let brightness = (frame as u8 + 1).saturating_mul(brightness_step);
            self.frames[frame].set_pixel(y, x, color, brightness);
        }
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > embedded_graphics::prelude::OriginDimensions
    for DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn size(&self) -> embedded_graphics::prelude::Size {
        embedded_graphics::prelude::Size::new(COLS as u32, ROWS as u32)
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > embedded_graphics::prelude::OriginDimensions
    for &DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn size(&self) -> embedded_graphics::prelude::Size {
        embedded_graphics::prelude::Size::new(COLS as u32, ROWS as u32)
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > embedded_graphics::prelude::OriginDimensions
    for &mut DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn size(&self) -> embedded_graphics::prelude::Size {
        embedded_graphics::prelude::Size::new(COLS as u32, ROWS as u32)
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > embedded_graphics::draw_target::DrawTarget
    for DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    type Color = Color;

    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>>,
    {
        for pixel in pixels {
            self.set_pixel_internal(pixel.0.x as usize, pixel.0.y as usize, pixel.1);
        }
        Ok(())
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > embedded_graphics::draw_target::DrawTarget
    for &mut DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    type Color = Color;

    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>>,
    {
        for pixel in pixels {
            self.set_pixel_internal(pixel.0.x as usize, pixel.0.y as usize, pixel.1);
        }
        Ok(())
    }
}

unsafe impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > ReadBuffer for DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        let ptr = &self.frames as *const _ as *const u8;
        let len = core::mem::size_of_val(&self.frames);
        (ptr, len)
    }
}

unsafe impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > ReadBuffer for &DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        let ptr = &self.frames as *const _ as *const u8;
        let len = core::mem::size_of_val(&self.frames);
        (ptr, len)
    }
}

unsafe impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > ReadBuffer for &mut DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        let ptr = &self.frames as *const _ as *const u8;
        let len = core::mem::size_of_val(&self.frames);
        (ptr, len)
    }
}

#[cfg(feature = "log")]
impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > core::fmt::Debug for DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let brightness_step = 1 << (8 - BITS);
        f.debug_struct("DmaFrameBuffer")
            .field("size", &core::mem::size_of_val(&self.frames))
            .field("frame_count", &self.frames.len())
            .field("frame_size", &core::mem::size_of_val(&self.frames[0]))
            .field("brightness_step", &&brightness_step)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > defmt::Format for DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn format(&self, f: defmt::Formatter) {
        let brightness_step = 1 << (8 - BITS);
        defmt::write!(
            f,
            "DmaFrameBuffer<{}, {}, {}, {}, {}>",
            ROWS,
            COLS,
            NROWS,
            BITS,
            FRAME_COUNT
        );
        defmt::write!(f, " size: {}", core::mem::size_of_val(&self.frames));
        defmt::write!(
            f,
            " frame_size: {}",
            core::mem::size_of_val(&self.frames[0])
        );
        defmt::write!(f, " brightness_step: {}", brightness_step);
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > FrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
    for DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn get_word_size(&self) -> WordSize {
        WordSize::Sixteen
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > FrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
    for &DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn get_word_size(&self) -> WordSize {
        WordSize::Sixteen
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > FrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
    for &mut DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn get_word_size(&self) -> WordSize {
        WordSize::Sixteen
    }
}
