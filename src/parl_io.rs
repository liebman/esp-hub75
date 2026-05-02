use esp_hal::dma::DmaChannelFor;
use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaError;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::gpio::AnyPin;
#[cfg(not(feature = "esp32c5"))]
use esp_hal::gpio::NoPin;
use esp_hal::parl_io::BitPackOrder;
use esp_hal::parl_io::ClkOutPin;
use esp_hal::parl_io::ConfigurePins;
use esp_hal::parl_io::ParlIo;
use esp_hal::parl_io::ParlIoTx;
use esp_hal::parl_io::ParlIoTxTransfer;
use esp_hal::parl_io::SampleEdge;
use esp_hal::parl_io::TxConfig;
use esp_hal::parl_io::TxEightBits;
use esp_hal::parl_io::TxPins;
#[cfg(not(feature = "esp32c5"))]
use esp_hal::parl_io::TxSixteenBits;
use esp_hal::peripherals::PARL_IO;
#[cfg(feature = "iram")]
use esp_hal::ram;
use esp_hal::time::Rate;

use crate::framebuffer::FrameBuffer;
use crate::Hub75Error;
use crate::Hub75Pins;
#[cfg(not(feature = "esp32c5"))]
use crate::Hub75Pins16;
use crate::Hub75Pins8;
/// HUB75 LED matrix display driver
pub struct Hub75<'d, DM: esp_hal::DriverMode> {
    parl_io: ParlIoTx<'d, DM>,
    tx_descriptors: &'static mut [DmaDescriptor],
}

impl<'d> Hub75<'d, esp_hal::Async> {
    /// Creates a new async HUB75 driver instance
    ///
    /// # Arguments
    /// * `parl_io` - The PARL_IO peripheral instance
    /// * `hub75_pins` - The HUB75 pin configuration
    /// * `channel` - The DMA channel to use for transfers
    /// * `tx_descriptors` - DMA descriptors for the transfer buffer
    /// * `frequency` - The clock frequency for the display
    ///
    /// # Returns
    /// A new `Hub75` instance configured for async operation
    pub fn new_async<T: TxPins + ConfigurePins + 'd>(
        parl_io: PARL_IO<'d>,
        hub75_pins: impl Hub75Pins<'d, T>,
        channel: impl DmaChannelFor<PARL_IO<'d>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
    ) -> Result<Self, Hub75Error> {
        let (parl_io, pins, clk_pin, config) =
            Self::new_internal(parl_io, hub75_pins, channel, frequency)?;
        let parl_io = parl_io.into_async().tx.with_config(pins, clk_pin, config)?;
        #[cfg(feature = "esp32c5")]
        unsafe {
            // on the C5 we don't need to tell PARL_IO how many bytes are in the transfer
            use esp32c5 as pac;
            let pio = pac::PARL_IO::steal();
            pio.tx_genrl_cfg()
                .modify(|_, w| w.tx_eof_gen_sel().set_bit());
        }
        Ok(Self {
            parl_io,
            tx_descriptors,
        })
    }
}

impl<'d> Hub75<'d, esp_hal::Blocking> {
    /// Creates a new blocking HUB75 driver instance
    ///
    /// # Arguments
    /// * `parl_io` - The PARL_IO peripheral instance
    /// * `hub75_pins` - The HUB75 pin configuration
    /// * `channel` - The DMA channel to use for transfers
    /// * `tx_descriptors` - DMA descriptors for the transfer buffer
    /// * `frequency` - The clock frequency for the display
    ///
    /// # Returns
    /// A new `Hub75` instance configured for blocking operation
    pub fn new<T: TxPins + ConfigurePins + 'd>(
        parl_io: PARL_IO<'d>,
        hub75_pins: impl Hub75Pins<'d, T>,
        channel: impl DmaChannelFor<PARL_IO<'d>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
    ) -> Result<Self, Hub75Error> {
        let (parl_io, pins, clk_pin, config) =
            Self::new_internal(parl_io, hub75_pins, channel, frequency)?;
        let parl_io = parl_io.tx.with_config(pins, clk_pin, config)?;
        #[cfg(feature = "esp32c5")]
        unsafe {
            // on the C5 we don't need to tell PARL_IO how many bytes are in the transfer
            use esp32c5 as pac;
            let pio = pac::PARL_IO::steal();
            pio.tx_genrl_cfg()
                .modify(|_, w| w.tx_eof_gen_sel().set_bit());
        }
        Ok(Self {
            parl_io,
            tx_descriptors,
        })
    }
}

impl<'d, DM: esp_hal::DriverMode> Hub75<'d, DM> {
    fn new_internal<T: TxPins + ConfigurePins + 'd>(
        parl_io: PARL_IO<'d>,
        hub75_pins: impl Hub75Pins<'d, T>,
        channel: impl DmaChannelFor<PARL_IO<'d>>,
        frequency: Rate,
    ) -> Result<(ParlIo<'d, esp_hal::Blocking>, T, ClkOutPin<'d>, TxConfig), esp_hal::parl_io::Error>
    {
        let (pins, clock_pin) = hub75_pins.convert_pins();
        let parl_io = ParlIo::new(parl_io, channel)?;
        #[cfg(feature = "invert-clock")]
        let sample_edge = SampleEdge::Normal;
        #[cfg(not(feature = "invert-clock"))]
        let sample_edge = SampleEdge::Invert;
        let config = TxConfig::default()
            .with_frequency(frequency)
            .with_idle_value(0)
            .with_sample_edge(sample_edge)
            .with_bit_order(BitPackOrder::Msb);
        let clock_pin = ClkOutPin::new(clock_pin);
        Ok((parl_io, pins, clock_pin, config))
    }

    /// Renders a frame buffer to the display.
    ///
    /// This method handles both contiguous (single-plane) and bitplane
    /// (multi-plane BCM-weighted) framebuffers automatically. The DMA
    /// descriptor chain is built internally — callers never need to manage
    /// DMA buffer types.
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
        fb: &impl FrameBuffer,
    ) -> Result<Hub75Transfer<'d, crate::bcm_dma_buf::BcmTxDmaBuf, DM>, (Hub75Error, Self)> {
        let parl_io = self.parl_io;
        let tx_descriptors = self.tx_descriptors;
        let tx_buf = crate::bcm_dma_buf::BcmTxDmaBuf::new(tx_descriptors, fb);

        // on the C5 we don't need to tell PARL_IO how many bytes are in the transfer
        #[cfg(feature = "esp32c5")]
        let len = 0;
        #[cfg(not(feature = "esp32c5"))]
        let len = tx_buf.transfer_len();

        let xfer = parl_io.write(len, tx_buf).map_err(|(e, parl_io, buf)| {
            let tx_descriptors = buf.split();
            (
                e.into(),
                Self {
                    parl_io,
                    tx_descriptors,
                },
            )
        })?;
        Ok(Hub75Transfer {
            xfer,
            tx_descriptors: None,
        })
    }

    /// Renders using a caller-provided DMA transmit buffer.
    ///
    /// This is an escape hatch for custom [`DmaTxBuffer`] implementations
    /// where the caller manages the descriptor chain directly.
    ///
    /// # Arguments
    /// * `len` - Number of bytes to transmit with PARL_IO.
    /// * `buf` - DMA buffer implementing [`DmaTxBuffer`].
    ///
    /// # Returns
    /// A [`Hub75Transfer`] which can be awaited and then consumed via
    /// [`Hub75Transfer::wait_with_buf`] to recover both the finalized buffer
    /// and the driver instance.
    ///
    /// # Errors
    /// Returns a tuple of `Hub75Error`, the recovered `Hub75` instance, and the
    /// original buffer if the transfer cannot be started.
    #[cfg_attr(feature = "iram", ram)]
    pub fn render_buf<BUF: DmaTxBuffer>(
        self,
        #[cfg_attr(feature = "esp32c5", allow(unused))] len: usize,
        buf: BUF,
    ) -> Result<Hub75Transfer<'d, BUF, DM>, (Hub75Error, Self, BUF)> {
        let parl_io = self.parl_io;
        let tx_descriptors = self.tx_descriptors;

        // on the C5 we don't need to tell PARL_IO how many bytes are in the transfer
        #[cfg(feature = "esp32c5")]
        let len = 0;

        match parl_io.write(len, buf) {
            Ok(xfer) => Ok(Hub75Transfer {
                xfer,
                tx_descriptors: Some(tx_descriptors),
            }),
            Err((e, parl_io, buf)) => Err((
                e.into(),
                Self {
                    parl_io,
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
/// transfer to complete.
pub struct Hub75Transfer<'d, BUF: DmaTxBuffer, DM: esp_hal::DriverMode> {
    xfer: ParlIoTxTransfer<'d, BUF, DM>,
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
        let (result, parl_io, final_buf) = xfer.wait();
        let tx_descriptors = tx_descriptors
            .expect("wait_with_buf is only supported for render_buf()-started transfers");
        (
            result,
            final_buf,
            Hub75 {
                parl_io,
                tx_descriptors,
            },
        )
    }
}

impl<'d, DM: esp_hal::DriverMode> Hub75Transfer<'d, crate::bcm_dma_buf::BcmTxDmaBuf, DM> {
    /// Waits for the transfer to complete and recovers the `Hub75` instance.
    ///
    /// # Returns
    /// A tuple containing:
    /// 1. The result of the transfer
    /// 2. The `Hub75` instance for reuse
    #[cfg_attr(feature = "iram", ram)]
    pub fn wait(self) -> (Result<(), DmaError>, Hub75<'d, DM>) {
        let (result, parl_io, bcm_buf) = self.xfer.wait();
        let tx_descriptors = bcm_buf.split();
        (
            result,
            Hub75 {
                parl_io,
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
        self.xfer.wait_for_done().await;
        Ok(())
    }
}

#[cfg(not(feature = "esp32c5"))]
impl<'d> Hub75Pins<'d, TxSixteenBits<'d>> for Hub75Pins16<'d> {
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

impl<'d> Hub75Pins<'d, TxEightBits<'d>> for Hub75Pins8<'d> {
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
