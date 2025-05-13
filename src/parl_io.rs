use esp_hal::dma::DmaChannelFor;
use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaError;
use esp_hal::dma::DmaTxBuf;
use esp_hal::gpio::AnyPin;
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
use esp_hal::parl_io::TxSixteenBits;
use esp_hal::peripherals::PARL_IO;
use esp_hal::time::Rate;

use crate::framebuffer::FrameBuffer;
use crate::Hub75Error;
use crate::Hub75Pins16;
use crate::Hub75Pins8;
use crate::Hub75Pins;
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
        let config = TxConfig::default()
            .with_frequency(frequency)
            .with_idle_value(0)
            .with_sample_edge(SampleEdge::Normal)
            .with_bit_order(BitPackOrder::Msb);
        let clock_pin = ClkOutPin::new(clock_pin);
        Ok((parl_io, pins, clock_pin, config))
    }

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
        let parl_io = self.parl_io;
        let tx_descriptors = self.tx_descriptors;
        let tx_buffer = unsafe {
            let (ptr, len) = fb.read_buffer();
            // SAFETY: tx_buffer is only used until the tx_buf.split below!
            core::slice::from_raw_parts_mut(ptr as *mut u8, len)
        };
        // TODO: can't recover from this because tx_descriptors is consumed!
        let tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).expect("failed to create DmaTxBuf!");

        // TODO: parl_io has a max size limit of 32736 bytes so we need to send the
        // framebuffer in chunks
        let xfer = parl_io
            .write(tx_buf.len(), tx_buf)
            .map_err(|(e, parl_io, buf)| {
                let (tx_descriptors, _) = buf.split();
                (
                    e.into(),
                    Self {
                        parl_io,
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
/// transfer to complete.
pub struct Hub75Transfer<'d, DM: esp_hal::DriverMode> {
    xfer: ParlIoTxTransfer<'d, DmaTxBuf, DM>,
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
        let (result, parl_io, tx_buf) = self.xfer.wait();
        let (tx_descriptors, _) = tx_buf.split();
        match result {
            Ok(()) => (
                Ok(()),
                Hub75 {
                    parl_io,
                    tx_descriptors,
                },
            ),
            Err(e) => (
                Err(e),
                Hub75 {
                    parl_io,
                    tx_descriptors,
                },
            ),
        }
    }
}

impl Hub75Transfer<'_, esp_hal::Async> {
    /// Asynchronously waits for the transfer to complete
    ///
    /// # Returns
    /// A `Result` indicating whether the transfer completed successfully
    ///
    /// # Note
    /// This method does not return the `Hub75` instance. Use `wait()` after
    /// `wait_for_done` returns to get the `Hub75` instance, it won't block at
    /// that point.
    pub async fn wait_for_done(&mut self) -> Result<(), DmaError> {
        self.xfer.wait_for_done().await;
        Ok(())
    }
}

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
