use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::Rgb888;
use esp_hal::dma::ReadBuffer;

pub mod latched;
pub mod plain;

pub const BLANKING_DELAY: usize = 5;
pub type Color = Rgb888;

pub enum WordSize {
    Eight,
    Sixteen,
}

pub const fn compute_rows(rows: usize) -> usize {
    rows / 2
}

pub const fn compute_frame_count(bits: u8) -> usize {
    (1usize << bits) - 1
}

pub trait FrameBuffer<
    const ROWS: usize,
    const COLS: usize,
    const NROWS: usize,
    const BITS: u8,
    const FRAME_COUNT: usize,
>: ReadBuffer
{
    fn get_word_size(&self) -> WordSize;
}

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
