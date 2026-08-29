//! # ESP-HUB75
//!
//! A `no-std` driver for HUB75-style LED matrix panels on ESP32-series
//! microcontrollers.
//!
//! The panel is refreshed over DMA with almost no CPU involvement, using
//! whichever peripheral fits each chip best:
//!
//! - **ESP32-S3**: Uses the `LCD_CAM` peripheral
//! - **ESP32-C6**: Uses the `PARL_IO` peripheral
//! - **ESP32-C5**: Uses the `PARL_IO` peripheral (8-bit mode only; requires a
//!   latch circuit and `Hub75Pins8`)
//! - **ESP32**: Uses the I2S peripheral in parallel mode
//!
//! ## Framebuffers
//!
//! Use the **bitplane** framebuffers from the `hub75-framebuffer` crate.
//! They come in two variants: direct-drive (16-bit, no external latch) and
//! latched (8-bit, needs an external address-latch circuit). Both can be
//! handed to the peripheral as-is; there is no extra formatting step.
//!
//! Bitplane framebuffers (`framebuffer::bitplane::plain::DmaFrameBuffer` /
//! `framebuffer::bitplane::latched::DmaFrameBuffer`) store only one bit per
//! pixel per plane. The driver assembles the BCM (Binary Code Modulation)
//! output on the fly with DMA descriptors, so RAM use stays low without
//! losing visual quality.
//!
//! ## Usage
//!
//! Example for ESP32-C6:
//!
//! ```rust,no_run
//! #![no_std]
//! #![no_main]
//!
//! use embedded_graphics::Drawable;
//! use embedded_graphics::geometry::Point;
//! use embedded_graphics::mono_font::MonoTextStyleBuilder;
//! use embedded_graphics::mono_font::ascii::FONT_5X7;
//! use embedded_graphics::prelude::RgbColor;
//! use embedded_graphics::text::Alignment;
//! use embedded_graphics::text::Text;
//! use esp_backtrace as _;
//! use esp_hal::clock::CpuClock;
//! use esp_hal::gpio::Pin;
//! use esp_hal::main;
//! use esp_hal::time::Rate;
//! use esp_hub75::Color;
//! use esp_hub75::Hub75;
//! use esp_hub75::Hub75Pins16;
//! use esp_hub75::framebuffer::bitplane::plain::DmaFrameBuffer;
//! use esp_hub75::framebuffer::compute_rows;
//!
//! esp_bootloader_esp_idf::esp_app_desc!();
//!
//! const ROWS: usize = 64;
//! const COLS: usize = 64;
//! const NROWS: usize = compute_rows(ROWS);
//! const PLANES: usize = 4;
//!
//! type FBType = DmaFrameBuffer<NROWS, COLS, PLANES>;
//!
//! macro_rules! mk_static {
//!     ($t:ty,$val:expr) => {{
//!         static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
//!         #[deny(unused_attributes)]
//!         let x = STATIC_CELL.uninit().write($val);
//!         x
//!     }};
//! }
//!
//! #[main]
//! fn main() -> ! {
//!     let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
//!
//!     let tx_descriptors = esp_hub75::hub75_dma_descriptors!(FBType);
//!
//!     let pins = Hub75Pins16 {
//!         red1: peripherals.GPIO19.degrade(),
//!         grn1: peripherals.GPIO20.degrade(),
//!         blu1: peripherals.GPIO21.degrade(),
//!         red2: peripherals.GPIO22.degrade(),
//!         grn2: peripherals.GPIO23.degrade(),
//!         blu2: peripherals.GPIO15.degrade(),
//!         addr0: peripherals.GPIO10.degrade(),
//!         addr1: peripherals.GPIO8.degrade(),
//!         addr2: peripherals.GPIO1.degrade(),
//!         addr3: peripherals.GPIO0.degrade(),
//!         addr4: peripherals.GPIO11.degrade(),
//!         blank: peripherals.GPIO5.degrade(),
//!         clock: peripherals.GPIO7.degrade(),
//!         latch: peripherals.GPIO6.degrade(),
//!     };
//!
//!     let fb = mk_static!(FBType, FBType::new());
//!     let text_style = MonoTextStyleBuilder::new()
//!         .font(&FONT_5X7)
//!         .text_color(Color::YELLOW)
//!         .background_color(Color::BLACK)
//!         .build();
//!     let point = Point::new(32, 32);
//!     Text::with_alignment("Hello, World!", point, text_style, Alignment::Center)
//!         .draw(fb)
//!         .expect("failed to draw text");
//!
//!     let _hub75 = Hub75::new(
//!         peripherals.PARL_IO,
//!         pins,
//!         peripherals.DMA_CH0,
//!         tx_descriptors,
//!         Rate::from_mhz(20),
//!         &*fb,
//!     )
//!     .expect("failed to create Hub75");
//!
//!     loop {
//!         core::hint::spin_loop();
//!     }
//! }
//! ```
//!
//! ## Crate Features
//!
//! - `esp32`: Enable support for the ESP32
//! - `esp32s3`: Enable support for the ESP32-S3
//! - `esp32c5`: Enable support for the ESP32-C5
//! - `esp32c6`: Enable support for the ESP32-C6
//! - `defmt`: Enable logging with `defmt`
//! - `log`: Enable logging with the `log` crate
//! - `invert-blank`: Invert the blank signal in hardware by enabling the GPIO
//!   output inverter on the blank pin. Applies to both 8-bit latched
//!   (`Hub75Pins8`) and 16-bit direct-drive (`Hub75Pins16`) configurations.
//!   Some latch controller boards include a hardware inverter on the blank
//!   line; enable this feature to compensate.
//! - `invert-clock`: Invert the clock signal. By default the driver outputs
//!   data that changes on the falling edge of CLK so that it is stable when the
//!   panel latches on the rising edge. Enable this feature if your panel
//!   requires the opposite polarity.
//! - `invert-oe`: Forwards to the `hub75-framebuffer` crate, inverting the
//!   output-enable (OE) signal in the generated data stream. Whereas
//!   `invert-blank` inverts the blank pin in hardware, this feature flips the
//!   OE polarity at the framebuffer level instead. The two features may seem
//!   redundant but are meant to be used together: where the peripheral drives
//!   all pins to 0 when a transfer completes, `invert-blank` turns that idle 0
//!   into a 1 (blanked), and `invert-oe` compensates for the now-inverted pin.
//! - `full-chain-dma`: Build the entire BCM repetition chain in a single DMA
//!   transfer instead of one plane per interrupt. This reduces interrupt
//!   frequency at the cost of more DMA descriptor RAM. Note that the ESP32-C6
//!   `PARL_IO` peripheral has a 65 535-byte per-transfer limit, which
//!   constrains the maximum panel size and plane count when this feature is
//!   enabled.
//! - `circular-dma`: Circular DMA descriptor chain (implies `full-chain-dma`).
//!   The DMA engine starts once and loops forever; buffer swaps are instant
//!   pointer-delta updates with no DMA stop/restart. A frame-boundary ISR is
//!   always active in this mode, providing both `frame_count()` and the
//!   completion signal for [`Hub75Swap::wait()`] /
//!   [`Hub75Swap::wait_for_done()`]. Only supported on ESP32 and ESP32-S3; on
//!   ESP32-C5/C6 this is a compile-time error because `PARL_IO` cannot do
//!   circular chains.
//! - `skip-black-pixels`: Forwards to the `hub75-framebuffer` crate, enabling
//!   an optimization that skips writing black pixels to the framebuffer.
//! - `tail-closes-latch`: Forwards to the `hub75-framebuffer` crate. Appends a
//!   tail word at the end of each DMA buffer (`plain` framebuffers) or at the
//!   end of each bit-plane (`bitplane::plain`) that drives LATCH LOW when the
//!   transfer completes. Does not apply to latched framebuffers.
//! - `iram`: Place the driver's hot path (render / DMA wait functions) in
//!   Instruction RAM (IRAM) to avoid flash-cache stalls (for example during
//!   Wi-Fi, PSRAM, or SPI-flash activity) that can cause visible flicker. Costs
//!   roughly 5-10 KiB of IRAM.
//! - `lead-blank-1/2/4/8/16` / `trail-blank-1/2/4/8/16`: Forwards to
//!   `hub75-framebuffer`. Control the number of pixel-clock cycles of blanking
//!   (OE HIGH) inserted around row address changes. The lead blank controls
//!   blanking *before* the address change, and the trail blank controls
//!   blanking *after*. Higher values reduce ghosting at the cost of slightly
//!   less brightness.
//! - `inter-row-blank-4/8/16/32`: Forwards to `hub75-framebuffer`. Insert
//!   additional dead clock cycles at the end of each row. In plain framebuffers
//!   the gap defers the address change to the first pixel of the next row,
//!   giving slow panels more time to finish blanking. In latched framebuffers
//!   the gap adds extra blanked cycles after the address change.
//! - `reverse-row-order`: Forwards to `hub75-framebuffer`. Stores the rows of
//!   the framebuffer in reverse scan order so that the DMA stream renders the
//!   last panel row first and row 0 last.
//!
//! ## Safety
//!
//! This crate uses `unsafe` code to interface with hardware peripherals, but it
//! exposes a safe, high-level API.

#![no_std]
#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]

use esp_hal::gpio::AnyPin;
pub use hub75_framebuffer as framebuffer;
#[doc(hidden)]
pub use static_cell;
pub(crate) mod bcm;

#[cfg_attr(hub75_use_i2s_parallel, path = "i2s_parallel.rs")]
#[cfg_attr(hub75_use_lcd_cam, path = "lcd_cam.rs")]
#[cfg_attr(hub75_use_parl_io, path = "parl_io.rs")]
mod hub75;
mod isr;
pub use hub75::Hub75;
/// The color type used by the HUB75 driver.
pub use hub75_framebuffer::Color;
pub use isr::Hub75Swap;

#[cfg(all(feature = "circular-dma", any(esp32c5, esp32c6)))]
compile_error!(
    "circular-dma is not supported on ESP32-C5/C6: the PARL_IO peripheral \
     stops after the first transfer even with a circular descriptor chain."
);

/// Maximum number of bytes a single DMA descriptor can transfer on this
/// platform.
///
/// Used by [`dma_descriptor_count`] and [`hub75_dma_descriptors!`] to
/// compute the required number of DMA descriptors.
#[doc(hidden)]
pub const MAX_DMA_CHUNK_SIZE: usize = esp_hal::dma::CHUNK_SIZE;

/// Computes the number of DMA descriptors this driver needs for a
/// framebuffer of type `FB`.
///
/// `max_chunk` is the maximum number of bytes a single DMA descriptor can
/// transfer (see [`MAX_DMA_CHUNK_SIZE`]).
///
/// The count is derived from the framebuffer's static BCM sequence
/// ([`framebuffer::FrameBuffer::BCM_SEQUENCE`]): a segment of `len`
/// bytes streamed `reps` times needs `ceil(len / max_chunk) * reps`
/// descriptors. What exactly is returned depends on the driver's DMA mode:
///
/// - **`full-chain-dma` (implied by `circular-dma`):** the whole BCM scan
///   sequence is chained into a single transfer, so this is the total over all
///   segments of all periods.
/// - **Default (group-based):** each transfer covers one group of
///   [`framebuffer::FrameBuffer::BCM_SEGMENTS_PER_GROUP`] segments and the
///   descriptor table is rebuilt between transfers, so this is the maximum over
///   the groups of one period, considerably smaller than the total for
///   row-major framebuffers.
///
/// This is a `const fn` of the framebuffer *type* (no instance needed), so
/// descriptor tables can be allocated statically, e.g. via
/// [`hub75_dma_descriptors!`].
///
/// # Panics
///
/// * In const evaluation (compile-time) if `max_chunk` is zero.
/// * At compile time if
///   [`BCM_SEQUENCE_LEN`](framebuffer::FrameBuffer::BCM_SEQUENCE_LEN) is not
///   divisible by
///   [`BCM_SEGMENTS_PER_GROUP`](framebuffer::FrameBuffer::BCM_SEGMENTS_PER_GROUP).
///   Well-formed [`FrameBuffer`](framebuffer::FrameBuffer) implementations
///   always satisfy this invariant.
#[must_use]
pub const fn dma_descriptor_count<FB: framebuffer::FrameBuffer>(max_chunk: usize) -> usize {
    assert!(max_chunk > 0, "max_chunk must be greater than zero");
    const {
        assert!(
            FB::BCM_SEQUENCE_LEN % FB::BCM_SEGMENTS_PER_GROUP == 0,
            "BCM_SEQUENCE_LEN must be divisible by BCM_SEGMENTS_PER_GROUP"
        );
    }
    let seq = FB::BCM_SEQUENCE;
    let period = FB::BCM_SEQUENCE_LEN;
    #[cfg(feature = "full-chain-dma")]
    let spg = period; // the whole period is chained into one transfer
    #[cfg(not(feature = "full-chain-dma"))]
    let spg = FB::BCM_SEGMENTS_PER_GROUP;
    let mut max_group = 0usize;
    let mut base = 0usize;
    while base < period {
        let mut group = 0usize;
        let mut j = 0usize;
        while j < spg {
            let entry = seq[base + j];
            group += entry.len.div_ceil(max_chunk) * entry.reps;
            j += 1;
        }
        if group > max_group {
            max_group = group;
        }
        base += spg;
    }
    #[cfg(feature = "full-chain-dma")]
    {
        // all periods are identical and chained together
        max_group *= FB::BCM_SEQUENCE_COUNT;
    }
    max_group
}

/// Allocates static DMA descriptors sized for the given framebuffer type.
///
/// This macro computes the required number of DMA descriptors at compile
/// time with [`dma_descriptor_count`] and allocates them in a static cell.
/// It returns `&'static mut [DmaDescriptor]` suitable for passing to
/// [`Hub75::new`] or [`Hub75::new_async`].
///
/// # Example
/// ```rust,ignore
/// type FBType = DmaFrameBuffer<NROWS, COLS, PLANES>;
/// let tx_descriptors = esp_hub75::hub75_dma_descriptors!(FBType);
/// ```
#[macro_export]
macro_rules! hub75_dma_descriptors {
    ($fb_type:ty) => {{
        const __N: usize = $crate::dma_descriptor_count::<$fb_type>($crate::MAX_DMA_CHUNK_SIZE);
        static __DESC_CELL: $crate::static_cell::StaticCell<[esp_hal::dma::DmaDescriptor; __N]> =
            $crate::static_cell::StaticCell::new();
        __DESC_CELL
            .uninit()
            .write([esp_hal::dma::DmaDescriptor::EMPTY; __N])
            .as_mut_slice()
    }};
}

/// Pin configuration for a HUB75 panel without an external address latch.
///
/// This configuration requires 16 bits of data per pixel transfer, as the row
/// address lines are driven directly along with the color data.
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

/// Pin configuration for a HUB75 panel with an external address latch.
///
/// This configuration is more memory-efficient, requiring only 8 bits of data
/// per pixel transfer. The row address is set once per row and held by an
/// external latch on the controller board. For an example of a latch circuit,
/// see the [`hub75-framebuffer` crate documentation](https://crates.io/crates/hub75-framebuffer)
/// and its [GitHub repository](https://github.com/liebman/hub75-framebuffer).
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

/// Applies a set of HUB75 pins to the specific ESP32 peripheral.
///
/// This hides the differences in pin configuration between peripherals
/// (I2S, LCD-CAM, `PARL_IO`) and between direct-drive (16-bit) and latched
/// (8-bit) HUB75 controller boards.
#[cfg(hub75_use_lcd_cam)]
pub trait Hub75Pins<'d> {
    /// The word type for this pin configuration (`u8` for 8-bit, `u16` for
    /// 16-bit). Must match
    /// [`FrameBuffer::Word`](framebuffer::FrameBuffer::Word).
    type Word;

    /// Returns the bus width (8-bit or 16-bit) for this pin configuration.
    fn word_size(&self) -> crate::framebuffer::WordSize;

    /// Apply pin configuration to the i8080 driver.
    fn apply<DM: esp_hal::DriverMode>(
        self,
        i8080: esp_hal::lcd_cam::lcd::i8080::I8080<'d, DM>,
    ) -> esp_hal::lcd_cam::lcd::i8080::I8080<'d, DM>;
}

/// Converts a set of HUB75 pins into the format a specific ESP32 peripheral
/// expects.
///
/// This hides the differences in pin configuration between peripherals
/// (I2S, LCD-CAM, `PARL_IO`) and between direct-drive (16-bit) and latched
/// (8-bit) HUB75 controller boards.
///
/// # Type Parameters
/// * `T` - The target pin configuration type for the specific peripheral.
#[cfg(not(hub75_use_lcd_cam))]
pub trait Hub75Pins<'d, T> {
    /// The word type for this pin configuration (`u8` for 8-bit, `u16` for
    /// 16-bit). Must match
    /// [`FrameBuffer::Word`](framebuffer::FrameBuffer::Word).
    type Word;

    /// Converts the high-level pin definition into the peripheral-specific
    /// format needed by the driver.
    ///
    /// # Returns
    /// A tuple containing:
    /// 1. The converted pin configuration for the specific peripheral.
    /// 2. The clock pin used for synchronization.
    fn convert_pins(self) -> (T, AnyPin<'d>);
}

/// Errors returned by the HUB75 driver.
///
/// Wraps the `esp-hal` DMA, buffer, and peripheral errors in one type.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Hub75Error {
    /// The driver has not been initialised (no `Hub75` instance exists).
    NotInitialised,
    /// The driver has already been initialised. A `Hub75` instance runs for
    /// the whole program and cannot be released, so only one may exist.
    AlreadyInitialised,
    /// Error during a DMA transfer
    Dma(esp_hal::dma::DmaError),
    /// Error while managing DMA buffers
    DmaBuf(esp_hal::dma::DmaBufError),
    /// A framebuffer swap is already in flight; only one
    /// [`Hub75Swap`] may be outstanding at a time. Call
    /// `.wait()` (or `.wait_for_done().await` then `.wait()`) on the
    /// previous swap before calling `swap()` again.
    SwapInFlight,
    /// Error from the `PARL_IO` peripheral
    #[cfg(hub75_use_parl_io)]
    ParlIo(esp_hal::parl_io::Error),
    /// Configuration error for the `PARL_IO` peripheral
    #[cfg(hub75_use_parl_io)]
    ConfigError(esp_hal::parl_io::ConfigError),
    /// Configuration error for the I8080 interface (`LCD_CAM`)
    #[cfg(hub75_use_lcd_cam)]
    I8080(esp_hal::lcd_cam::lcd::i8080::ConfigError),
    /// The driver is already running. `restart()` or `start()` was called while
    /// a transfer was in-flight. Wait for the transfer to complete before
    /// restarting.
    AlreadyRunning,
}

impl core::fmt::Display for Hub75Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NotInitialised => write!(f, "Hub75 not initialised"),
            Self::AlreadyInitialised => write!(f, "Hub75 driver already initialised"),
            Self::SwapInFlight => write!(f, "framebuffer swap already in flight"),
            Self::Dma(e) => write!(f, "DMA error: {e:?}"),
            Self::DmaBuf(e) => write!(f, "DMA buffer error: {e:?}"),
            #[cfg(hub75_use_parl_io)]
            Self::ParlIo(e) => write!(f, "PARL_IO error: {e:?}"),
            #[cfg(hub75_use_parl_io)]
            Self::ConfigError(e) => write!(f, "PARL_IO config error: {e:?}"),
            #[cfg(hub75_use_lcd_cam)]
            Self::I8080(e) => write!(f, "I8080 config error: {e:?}"),
            Self::AlreadyRunning => write!(
                f,
                "driver is already running; call wait() on the outstanding swap before restarting"
            ),
        }
    }
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

#[cfg(hub75_use_parl_io)]
impl From<esp_hal::parl_io::Error> for Hub75Error {
    fn from(e: esp_hal::parl_io::Error) -> Self {
        Self::ParlIo(e)
    }
}

#[cfg(hub75_use_parl_io)]
impl From<esp_hal::parl_io::ConfigError> for Hub75Error {
    fn from(e: esp_hal::parl_io::ConfigError) -> Self {
        Self::ConfigError(e)
    }
}
