//! I2S Parallel driver for HUB75 LED matrix displays
//!
//! This module provides a high-level interface for driving HUB75 LED matrix
//! displays using the ESP32's I2S peripheral in parallel mode.
//!
//! # Features
//! - Async and blocking operation modes
//! - DMA-based transfers for efficient data movement
//! - Support for 8-bit and 16-bit configurations
//! - Configurable clock frequency
//!
//! # Example
//! ```no_run
//! use esp_hal::dma::DmaChannelFor;
//! use esp_hal::dma::DmaDescriptor;
//! use esp_hal::i2s::AnyI2s;
//! use esp_hal::time::Rate;
//! use esp_hub75::plain::DmaFrameBuffer;
//! use esp_hub75::Color;
//! use esp_hub75::Hub75;
//! use esp_hub75::Hub75Pins16;
//!
//! // Create and initialize frame buffer
//! let mut fb = DmaFrameBuffer::<32, 64, 16, 4, 4>::new();
//! fb.clear();
//! fb.set_pixel(0, 0, Color::new(255, 0, 0));
//!
//! // Initialize HUB75
//! let mut hub75 = Hub75::new(
//!     i2s,
//!     Hub75Pins16::new(/* pins */),
//!     channel,
//!     &mut DESCRIPTORS,
//!     Rate::MHz(10),
//! )?
//! .into_async();
//!
//! // Main display loop
//! loop {
//!     // Render the frame
//!     let xfer = hub75.render(&fb)?;
//!
//!     // Wait for the transfer to complete and get back the Hub75 instance
//!     let (result, new_hub75) = xfer.wait();
//!     hub75 = new_hub75;
//!     result.expect("transfer failed");
//! }
//! ```
//!
//! # Safety
//! - The DMA descriptors must be properly aligned and sized
//! - The frame buffer must remain valid for the duration of the transfer

use esp_hal::dma::DmaChannelFor;
use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaError;
use esp_hal::dma::DmaTxBuf;
use esp_hal::gpio::AnyPin;
use esp_hal::gpio::NoPin;
use esp_hal::i2s::parallel::I2sParallel;
use esp_hal::i2s::parallel::I2sParallelTransfer;
use esp_hal::i2s::parallel::TxEightBits;
use esp_hal::i2s::parallel::TxPins;
use esp_hal::i2s::parallel::TxSixteenBits;
use esp_hal::i2s::AnyI2s;
use esp_hal::time::Rate;

use crate::framebuffer::FrameBuffer;
use crate::Hub75Error;
use crate::Hub75Pins;
use crate::Hub75Pins16;
use crate::Hub75Pins8;

/// HUB75 LED matrix display driver using I2S Parallel
///
/// This driver uses the ESP32's I2S peripheral in parallel mode to drive HUB75
/// LED matrix displays. It supports both 8-bit and 16-bit configurations and
/// can operate in either blocking or async mode.
pub struct Hub75<'d, DM: esp_hal::DriverMode> {
    i2s: I2sParallel<'d, DM>,
    tx_descriptors: &'static mut [DmaDescriptor],
}

impl<'d> Hub75<'d, esp_hal::Blocking> {
    /// Creates a new blocking HUB75 driver instance
    ///
    /// # Arguments
    /// * `i2s` - The I2S peripheral instance
    /// * `hub75_pins` - The HUB75 pin configuration
    /// * `channel` - The DMA channel to use for transfers
    /// * `tx_descriptors` - DMA descriptors for the transfer buffer
    /// * `frequency` - The clock frequency for the display
    ///
    /// # Returns
    /// A new `Hub75` instance configured for blocking operation
    ///
    /// # Errors
    /// Returns an error if the peripheral cannot be configured
    pub fn new<T: TxPins<'d>>(
        i2s: AnyI2s<'d>,
        hub75_pins: impl Hub75Pins<'d, T>,
        channel: impl DmaChannelFor<AnyI2s<'d>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
    ) -> Result<Self, Hub75Error> {
        let (pins, clock) = hub75_pins.convert_pins();
        let i2s = I2sParallel::new(i2s, channel, frequency, pins, clock);
        Ok(Self {
            i2s,
            tx_descriptors,
        })
    }

    /// Converts this blocking instance into an async instance
    pub fn into_async(self) -> Hub75<'d, esp_hal::Async> {
        Hub75 {
            i2s: self.i2s.into_async(),
            tx_descriptors: self.tx_descriptors,
        }
    }
}

impl<'d, DM: esp_hal::DriverMode> Hub75<'d, DM> {
    /// Renders a frame buffer to the display
    ///
    /// # Arguments
    /// * `fb` - The frame buffer to render
    ///
    /// # Returns
    /// A `Hub75Transfer` instance that can be used to wait for the transfer to
    /// complete
    ///
    /// # Errors
    /// Returns an error if the transfer cannot be started
    ///
    /// # Note
    /// The frame buffer must remain valid for the duration of the transfer
    pub fn render<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    >(
        self,
        fb: &impl FrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>,
    ) -> Result<Hub75Transfer<'d, DM>, (Hub75Error, Self)> {
        let i2s = self.i2s;
        let tx_descriptors = self.tx_descriptors;
        let tx_buffer = unsafe {
            let (ptr, len) = fb.read_buffer();
            // SAFETY: tx_buffer is only used until the tx_buf.split below!
            core::slice::from_raw_parts_mut(ptr as *mut u8, len)
        };
        let tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).expect("DmaTxBuf::new failed");
        let xfer = i2s.send(tx_buf).map_err(|(e, i2s, buf)| {
            let (tx_descriptors, _) = buf.split();
            (
                e.into(),
                Self {
                    i2s,
                    tx_descriptors,
                },
            )
        })?;
        Ok(Hub75Transfer { xfer })
    }
}

/// Represents an in-progress transfer to the HUB75 display
///
/// This struct is returned by `Hub75::render` and can be used to wait for the
/// transfer to complete. It provides both blocking and async methods for
/// waiting.
pub struct Hub75Transfer<'d, DM: esp_hal::DriverMode> {
    pub xfer: I2sParallelTransfer<'d, DmaTxBuf, DM>,
}

impl<'d, DM: esp_hal::DriverMode> Hub75Transfer<'d, DM> {
    /// Checks if the transfer is complete
    ///
    /// # Returns
    /// `true` if the transfer is complete and `wait()` will not block
    pub fn is_done(&self) -> bool {
        self.xfer.is_done()
    }

    /// Waits for the transfer to complete
    ///
    /// # Returns
    /// A tuple containing:
    /// 1. The result of the transfer
    /// 2. The `Hub75` instance for reuse
    ///
    /// # Note
    /// This method clears the transfer interrupt flag
    pub fn wait(self) -> (Result<(), DmaError>, Hub75<'d, DM>) {
        let (i2s, tx_buf) = self.xfer.wait();
        let (tx_descriptors, _) = tx_buf.split();
        (
            Ok(()),
            Hub75 {
                i2s,
                tx_descriptors,
            },
        )
    }
}

impl Hub75Transfer<'_, esp_hal::Async> {
    /// Asynchronously waits for the transfer to complete
    ///
    /// # Returns
    /// A `Result` indicating whether the transfer completed successfully
    ///
    /// # Note
    /// This method does not return the `Hub75` instance. Use `wait()` if you
    /// need to reuse the instance.
    pub async fn wait_for_done(&mut self) -> Result<(), DmaError> {
        self.xfer.wait_for_done().await
    }
}

impl<'d> crate::Hub75Pins<'d, TxSixteenBits<'d>> for Hub75Pins16<'d> {
    fn convert_pins(self) -> (TxSixteenBits<'d>, AnyPin<'d>) {
        let (_, blank) = self.blank.split();
        let pins = TxSixteenBits::new(
            self.addr0,
            self.addr1,
            self.addr2,
            self.addr3,
            self.addr4,
            self.latch,
            NoPin,
            NoPin,
            blank.inverted(),
            self.red1,
            self.grn1,
            self.blu1,
            self.red2,
            self.grn2,
            self.blu2,
            NoPin,
        );
        (pins, self.clock)
    }
}

impl<'d> crate::Hub75Pins<'d, TxEightBits<'d>> for Hub75Pins8<'d> {
    fn convert_pins(self) -> (TxEightBits<'d>, AnyPin<'d>) {
        let (_, blank) = self.blank.split();
        let pins = TxEightBits::new(
            self.red1,
            self.grn1,
            self.blu1,
            self.red2,
            self.grn2,
            self.blu2,
            self.latch,
            #[cfg(feature = "invert-blank")]
            blank.inverted(),
            #[cfg(not(feature = "invert-blank"))]
            blank,
        );
        (pins, self.clock)
    }
}
