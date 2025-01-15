use core::convert::Infallible;

use bitfield::bitfield;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::prelude::Point;
use esp_hal::dma::ReadBuffer;

pub mod latched;

const BLANKING_DELAY: usize = 5;
type Color = Rgb888;

pub const fn compute_rows(rows: usize) -> usize {
    rows / 2
}

pub const fn compute_frame_count(bits: u8) -> usize {
    (1usize << bits) - 1
}

bitfield! {
    #[derive(Clone, Copy, Default, PartialEq)]
    #[repr(transparent)]
    pub struct Entry(u16);
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

#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(C)]
pub struct Row<const COLS: usize> {
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
pub struct Frame<const ROWS: usize, const COLS: usize, const NROWS: usize> {
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
    pub const fn new() -> Self {
        assert!(BITS <= 8);

        Self {
            _align: 0,
            frames: [Frame::new(); FRAME_COUNT],
        }
    }

    pub const fn dma_buffer_size_bytes() -> usize {
        core::mem::size_of::<[Frame<ROWS, COLS, NROWS>; FRAME_COUNT]>()
    }

    pub fn clear(&mut self) {
        for frame in self.frames.iter_mut() {
            frame.format();
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
