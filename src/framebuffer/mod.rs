use embedded_graphics::pixelcolor::Rgb888;

pub mod latched;
pub mod plain;

pub const BLANKING_DELAY: usize = 5;
pub type Color = Rgb888;

pub const fn compute_rows(rows: usize) -> usize {
    rows / 2
}

pub const fn compute_frame_count(bits: u8) -> usize {
    (1usize << bits) - 1
}
