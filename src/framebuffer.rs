use core::convert::Infallible;

use bitfield::bitfield;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::prelude::Point;
use esp_hal::dma::ReadBuffer;
const BLANKING_DELAY: usize = 5;
type Color = Rgb888;

bitfield! {
    #[derive(Clone, Copy, Default, PartialEq)]
    #[repr(transparent)]
    pub struct Entry(u16);
    impl Debug;
    blu2, set_blu2: 15;
    grn2, set_grn2: 14;
    red2, set_red2: 13;
    blu1, set_blu1: 12;
    grn1, set_grn1: 11;
    red1, set_red1: 10;
    dummy2, set_dummy2: 9;
    dummy1, set_dummy1: 8;
    dummy0, set_dummy0: 7;
    output_enable, set_output_enable: 6;
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

pub struct DmaFrameBuffer<const ROWS: usize, const COLS: usize, const BITS: u8, const SIZE: usize> {
    buffer: [Entry; SIZE],
    frame_count: usize,
    frame_size: usize,
    brightness_step: u8,
}

pub const fn compute_frame_size(rows: usize, cols: usize) -> usize {
    rows / 2 * cols
}

pub const fn compute_buffer_size(rows: usize, cols: usize, bits: u8) -> usize {
    compute_frame_size(rows, cols) * (1 << bits)
}

impl<const ROWS: usize, const COLS: usize, const BITS: u8, const SIZE: usize>
    DmaFrameBuffer<ROWS, COLS, BITS, SIZE>
{
    pub const fn new() -> Self {
        assert!(BITS > 0 && BITS < 8);
        assert!(SIZE == compute_buffer_size(ROWS, COLS, BITS));
        Self {
            buffer: [Entry::new(); SIZE],
            frame_count: 1usize << BITS,
            frame_size: compute_frame_size(ROWS, COLS),
            brightness_step: 1 << (8 - BITS),
        }
    }

    pub const fn dma_buffer_size_bytes() -> usize {
        core::mem::size_of::<Entry>() * SIZE
    }

    pub fn frames(&self) -> core::slice::Chunks<'_, Entry> {
        self.buffer.chunks(self.frame_size)
    }

    // clear and format the frame
    fn clear_frame(&mut self, frame: usize) {
        let start = frame * self.frame_size;
        let end = (frame + 1) * self.frame_size;
        let buffer = &mut self.buffer[start..end];

        let mut prev_addr = 0u8;

        for y in 0usize..(ROWS / 2) {
            // we render in reverse order because when the lcd_cam device finishes renering
            // it sets the address lines back to 0
            let addr = ROWS as u8 / 2 - 1 - y as u8;
            let start = y * COLS;
            let mut entry: Entry = Entry::new();
            entry.set_addr(prev_addr as u16);
            entry.set_output_enable(false);
            for x in 0..COLS {
                // if we enable display too soon then we will have ghosting
                if x >= BLANKING_DELAY {
                    entry.set_output_enable(true);
                }
                // last pixel, blank the display, open the latch, set the new row address
                // the latch will be closed on the first pixel of the next row.
                if x == COLS - 1 {
                    entry.set_output_enable(false);
                    entry.set_latch(true);
                    entry.set_addr(addr as u16);
                }
                buffer[start + x] = entry;
            }
            // next address
            prev_addr = addr;
        }
    }

    pub fn clear(&mut self) {
        for frame in 0..self.frame_count {
            self.clear_frame(frame);
        }
    }

    pub fn set_pixel(&mut self, p: Point, color: Color) {
        if p.x < 0 || p.y < 0 {
            return;
        }
        self.set_pixel_internal(p.x as usize, p.y as usize, color);
    }

    fn set_pixel_internal(&mut self, x: usize, y: usize, color: Rgb888) {
        if x >= COLS || y >= ROWS {
            return;
        }
        let y = ROWS - 1 - y;
        let pixel_index = if y < ROWS / 2 {
            y * COLS + x
        } else {
            (y - (ROWS / 2)) * COLS + x
        };
        // set the pixel in all frames
        for frame in 0..self.frame_count {
            let brightness = (frame as u8 + 1).saturating_mul(self.brightness_step);
            let start = frame * self.frame_size;
            let end = (frame + 1) * self.frame_size;
            let buffer = &mut self.buffer[start..end];
            if y >= ROWS / 2 {
                buffer[pixel_index].set_color0(color, brightness);
            } else {
                buffer[pixel_index].set_color1(color, brightness);
            }
        }
    }
}

impl<const ROWS: usize, const COLS: usize, const BITS: u8, const SIZE: usize> Default
    for DmaFrameBuffer<ROWS, COLS, BITS, SIZE>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const ROWS: usize, const COLS: usize, const BITS: u8, const SIZE: usize>
    embedded_graphics::prelude::OriginDimensions for DmaFrameBuffer<ROWS, COLS, BITS, SIZE>
{
    fn size(&self) -> embedded_graphics::prelude::Size {
        embedded_graphics::prelude::Size::new(COLS as u32, ROWS as u32)
    }
}

impl<const ROWS: usize, const COLS: usize, const BITS: u8, const SIZE: usize>
    embedded_graphics::draw_target::DrawTarget for DmaFrameBuffer<ROWS, COLS, BITS, SIZE>
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

unsafe impl<const ROWS: usize, const COLS: usize, const BITS: u8, const SIZE: usize> ReadBuffer
    for DmaFrameBuffer<ROWS, COLS, BITS, SIZE>
{
    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        (
            self.buffer.as_ptr() as *const u8,
            self.buffer.len() * core::mem::size_of::<Entry>(),
        )
    }
}

#[cfg(feature = "log")]
impl<const ROWS: usize, const COLS: usize, const BITS: u8, const SIZE: usize> core::fmt::Debug
    for DmaFrameBuffer<ROWS, COLS, BITS, SIZE>
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DmaFrameBuffer")
            .field("buffer_len", &self.buffer.len())
            .field("frame_count", &self.frame_count)
            .field("frame_size", &self.frame_size)
            .field("brightness_step", &self.brightness_step)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl<const ROWS: usize, const COLS: usize, const BITS: u8, const SIZE: usize> defmt::Format
    for DmaFrameBuffer<ROWS, COLS, BITS, SIZE>
{
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "DmaFrameBuffer<{}, {}, {}>", ROWS, COLS, BITS);
        defmt::write!(f, " frame_count: {}", self.frame_count);
        defmt::write!(f, " frame_size: {}", self.frame_size);
        defmt::write!(f, " brightness_step: {}", self.brightness_step);
        defmt::write!(f, " buffer size: {}", self.buffer.len());
    }
}
