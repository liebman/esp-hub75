use esp_hal::dma::DmaChannelFor;
use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaError;
use esp_hal::dma::DmaTxBuf;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::dma::ReadBuffer;
use esp_hal::gpio::AnyPin;
use esp_hal::gpio::NoPin;
use esp_hal::i2s::parallel::I2sParallel;
use esp_hal::i2s::parallel::I2sParallelTransfer;
use esp_hal::i2s::parallel::TxEightBits;
use esp_hal::i2s::parallel::TxPins;
use esp_hal::i2s::parallel::TxSixteenBits;
use esp_hal::i2s::AnyI2s;
#[cfg(feature = "iram")]
use esp_hal::ram;
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
        // by default, we want data to change on the falling edge of the clock
        // the esp32 i2s peripheral wants to do this on the rising edge so we invert the
        // clock opposite of the feature flag
        #[cfg(not(feature = "invert-clock"))]
        let clock = clock.into_output_signal().with_output_inverter(true);
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
    /// Renders a frame buffer to the display.
    ///
    /// Calling render consumes the `Hub75` instance and returns a
    /// `Hub75Transfer` instance that can be used to wait for the transfer
    /// to complete.  After the transfer is complete, the `Hub75` will be
    /// returned from the `wait()` method on the `Hub75Transfer` instance.
    ///
    /// # Arguments
    /// * `fb` - The frame buffer to render
    ///
    /// # Returns
    /// A `Hub75Transfer` instance that can be used to wait for the transfer to
    /// complete
    ///
    /// # Errors
    /// Returns a tuple of `Hub75Error` and the `Hub75` instance if the transfer
    /// cannot be started
    #[cfg_attr(feature = "iram", ram)]
    pub fn render(
        self,
        fb: &(impl FrameBuffer + ReadBuffer),
    ) -> Result<Hub75Transfer<'d, DmaTxBuf, DM>, (Hub75Error, Self)> {
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
        Ok(Hub75Transfer {
            xfer,
            tx_descriptors: None,
        })
    }

    #[cfg_attr(feature = "iram", ram)]
    /// Renders using a caller-provided DMA transmit buffer.
    ///
    /// This is the generic render path for custom [`DmaTxBuffer`]
    /// implementations such as
    /// [`hub75_framebuffer::bitplane::latched::BcmDmaTxBuf`], where descriptor chaining
    /// defines transfer timing/weighting.
    ///
    /// # Arguments
    /// * `buf` - DMA buffer implementing [`DmaTxBuffer`].
    ///
    /// # Returns
    /// A [`Hub75Transfer`] which can be awaited and then consumed via
    /// [`Hub75Transfer::wait_with_buf`] to recover both the finalized buffer and
    /// the driver instance.
    ///
    /// # Errors
    /// Returns a tuple of `Hub75Error`, the recovered `Hub75` instance, and the
    /// original buffer if the transfer cannot be started.
    pub fn render_buf<BUF: DmaTxBuffer>(
        self,
        buf: BUF,
    ) -> Result<Hub75Transfer<'d, BUF, DM>, (Hub75Error, Self, BUF)> {
        let i2s = self.i2s;
        let tx_descriptors = self.tx_descriptors;
        match i2s.send(buf) {
            Ok(xfer) => Ok(Hub75Transfer {
                xfer,
                tx_descriptors: Some(tx_descriptors),
            }),
            Err((e, i2s, buf)) => Err((
                e.into(),
                Self {
                    i2s,
                    tx_descriptors,
                },
                buf,
            )),
        }
    }
}

/// Represents an in-progress transfer to the HUB75 display
///
/// This struct is returned by `Hub75::render` and can be used to wait for the
/// transfer to complete. It provides both blocking and async methods for
/// waiting.
pub struct Hub75Transfer<'d, BUF: DmaTxBuffer, DM: esp_hal::DriverMode> {
    xfer: I2sParallelTransfer<'d, BUF, DM>,
    tx_descriptors: Option<&'static mut [DmaDescriptor]>,
}

impl<'d, BUF: DmaTxBuffer, DM: esp_hal::DriverMode> Hub75Transfer<'d, BUF, DM> {
    /// Checks if the transfer is complete
    ///
    /// # Returns
    /// `true` if the transfer is complete and `wait()` will not block
    #[cfg_attr(feature = "iram", ram)]
    pub fn is_done(&self) -> bool {
        self.xfer.is_done()
    }

    /// Waits for transfer completion and returns the finalized buffer plus hub.
    #[cfg_attr(feature = "iram", ram)]
    pub fn wait_with_buf(self) -> (Result<(), DmaError>, BUF::Final, Hub75<'d, DM>) {
        let Hub75Transfer {
            xfer,
            tx_descriptors,
        } = self;
        let (i2s, final_buf) = xfer.wait();
        let tx_descriptors = tx_descriptors
            .expect("wait_with_buf is only supported for render_buf()-started transfers");
        (
            Ok(()),
            final_buf,
            Hub75 {
                i2s,
                tx_descriptors,
            },
        )
    }
}

impl<'d, DM: esp_hal::DriverMode> Hub75Transfer<'d, DmaTxBuf, DM> {
    /// Waits for the transfer to complete
    ///
    /// # Returns
    /// A tuple containing:
    /// 1. The result of the transfer
    /// 2. The `Hub75` instance for reuse
    ///
    /// # Note
    /// This method clears the transfer interrupt flag
    #[cfg_attr(feature = "iram", ram)]
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

impl<BUF: DmaTxBuffer> Hub75Transfer<'_, BUF, esp_hal::Async> {
    /// Asynchronously waits for the transfer to complete
    ///
    /// # Returns
    /// A `Result` indicating whether the transfer completed successfully
    ///
    /// # Note
    /// This method does not return the `Hub75` instance. Use `wait()` after
    /// `wait_for_done` returns to get the `Hub75` instance, it won't block at
    /// that point.
    #[cfg_attr(feature = "iram", ram)]
    pub async fn wait_for_done(&mut self) -> Result<(), DmaError> {
        self.xfer.wait_for_done().await
    }
}

impl<'d> crate::Hub75Pins<'d, TxSixteenBits<'d>> for Hub75Pins16<'d> {
    fn convert_pins(self) -> (TxSixteenBits<'d>, AnyPin<'d>) {
        let (_, blank) = unsafe { self.blank.split() };
        let pins = TxSixteenBits::new(
            self.addr0,
            self.addr1,
            self.addr2,
            self.addr3,
            self.addr4,
            self.latch,
            NoPin,
            NoPin,
            blank.with_output_inverter(true),
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
        let (_, blank) = unsafe { self.blank.split() };
        let pins = TxEightBits::new(
            self.red1,
            self.grn1,
            self.blu1,
            self.red2,
            self.grn2,
            self.blu2,
            self.latch,
            #[cfg(feature = "invert-blank")]
            blank.with_output_inverter(true),
            #[cfg(not(feature = "invert-blank"))]
            blank,
        );
        (pins, self.clock)
    }
}
