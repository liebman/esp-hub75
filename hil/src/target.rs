//! The panel configuration the HIL suite runs against.
//!
//! Everything that depends on the selected chip or bus width and is *not* a
//! pin map lives here, so that a test file never has to know which wiring it
//! is running on: it uses [`FrameBuffer`], this module's constants and the
//! `hub75_*` macros, and compiles unchanged for both.
//!
//! Bus width is a property of the *types*: a 16-bit direct-drive panel needs
//! `Hub75Pins16` (word `u16`) with the `plain` framebuffer, an 8-bit latched
//! panel `Hub75Pins8` (word `u8`) with the `latched` one. The driver's bounds
//! tie the two together (`P: Hub75Pins<Word = FB::Word>`), so pairing a pin
//! struct with the wrong framebuffer is a compile error rather than a
//! plausible-looking panel.

use esp_hal::time::Rate;
use esp_hub75::framebuffer::FrameBuffer as FrameBufferTrait;

/// Panel height in pixels.
pub const ROWS: usize = 32;
/// Panel width in pixels.
pub const COLS: usize = 32;
/// Addressable rows: HUB75 panels are driven in two halves.
pub const NROWS: usize = esp_hub75::framebuffer::compute_rows(ROWS);
/// Bitplanes (color depth); the framebuffer supports `1..=8`.
///
/// The smallest geometry worth refreshing keeps the framebuffer, the DMA
/// transfer and the flash image small. It is also far below the 64 KiB
/// single-transfer limit of the ESP32-C6/C5 `PARL_IO`, which the examples have
/// to work around at 64x64 (it forces `PLANES = 4` there).
pub const PLANES: usize = 4;

/// The framebuffer for the selected bus width: `plain` (16-bit word, direct
/// drive) with `bus16`, `latched` (8-bit word) with `bus8`.
#[cfg(feature = "bus16")]
pub type FrameBuffer = esp_hub75::framebuffer::bitplane::plain::DmaFrameBuffer<NROWS, COLS, PLANES>;
/// See the `bus16` variant of [`FrameBuffer`].
#[cfg(feature = "bus8")]
pub type FrameBuffer =
    esp_hub75::framebuffer::bitplane::latched::DmaFrameBuffer<NROWS, COLS, PLANES>;

/// Pixel-clock frequency, inside every backend's limit (the ESP32's I2S tops
/// out around 19 MHz).
pub const RATE: Rate = Rate::from_mhz(10);

/// Pixel clocks needed for one complete panel refresh, from the framebuffer's
/// BCM sequence.
pub const CYCLES: u64 = esp_hub75::frame_clock_cycles::<FrameBuffer>();

/// The refresh rate `CYCLES` implies at [`RATE`].
pub const REFRESH_HZ: u32 = esp_hub75::refresh_hz::<FrameBuffer>(RATE);

/// DMA descriptors a driver for [`FrameBuffer`] needs.
pub const DESCRIPTOR_COUNT: usize =
    esp_hub75::dma_descriptor_count::<FrameBuffer>(esp_hub75::MAX_DMA_CHUNK_SIZE);

// ---------------------------------------------------------------------------
// Compile-time expectations
// ---------------------------------------------------------------------------
//
// Checked by the build rather than by a board, so a change to the framebuffer
// or the BCM plumbing under the tests fails here first.

const _: () = assert!(NROWS == ROWS / 2);
const _: () = assert!(PLANES >= 1 && PLANES <= 8);
const _: () = assert!(CYCLES > 0);
const _: () = assert!(REFRESH_HZ > 0);
const _: () = assert!(REFRESH_HZ < RATE.as_hz());
const _: () = assert!(DESCRIPTOR_COUNT > 0);

/// The bus-width feature really selected the framebuffer family it promises:
/// a 16-bit word for direct drive, an 8-bit one for the latched layout.
#[cfg(feature = "bus16")]
const _: () = assert!(core::mem::size_of::<<FrameBuffer as FrameBufferTrait>::Word>() == 2);
#[cfg(feature = "bus8")]
const _: () = assert!(core::mem::size_of::<<FrameBuffer as FrameBufferTrait>::Word>() == 1);

/// The refresh model is a pure function of the framebuffer type and the rate,
/// so the build can hold it still rather than only checking it at runtime. The
/// two wirings differ because the latched framebuffer carries its own
/// lead/trail blanking and the plain one does not (here 16 rows x 60 clocks).
///
/// Re-measure deliberately when the BCM sequence changes: replace one assert
/// with `const _: [(); CYCLES as usize] = [(); 0];` and read the expected
/// length off the type error.
#[cfg(feature = "bus16")]
pub const EXPECTED_CYCLES: u64 = 7680;
#[cfg(feature = "bus8")]
pub const EXPECTED_CYCLES: u64 = 8640;

/// See [`EXPECTED_CYCLES`]; this is `RATE` divided by it.
#[cfg(feature = "bus16")]
pub const EXPECTED_REFRESH_HZ: u32 = 1302;
#[cfg(feature = "bus8")]
pub const EXPECTED_REFRESH_HZ: u32 = 1157;

const _: () = assert!(CYCLES == EXPECTED_CYCLES);
const _: () = assert!(REFRESH_HZ == EXPECTED_REFRESH_HZ);
const _: () = assert!(REFRESH_HZ == RATE.as_hz() / CYCLES as u32);
