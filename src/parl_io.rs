use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaError;
use esp_hal::dma::DmaTxBuf;
use esp_hal::dma::TxChannelFor;
use esp_hal::gpio::NoPin;
use esp_hal::parl_io::BitPackOrder;
use esp_hal::parl_io::ClkOutPin;
use esp_hal::parl_io::ParlIoTx;
use esp_hal::parl_io::ParlIoTxOnly;
use esp_hal::parl_io::ParlIoTxTransfer;
use esp_hal::parl_io::SampleEdge;
#[cfg(feature = "valid-pin")]
use esp_hal::parl_io::TxPinConfigIncludingValidPin;
use esp_hal::parl_io::TxSixteenBits;
use esp_hal::peripheral::Peripheral;
use esp_hal::peripherals::PARL_IO;

use crate::framebuffer::DmaFrameBuffer;
use crate::HertzU32;
use crate::Hub75Error;
use crate::Hub75Pins;

type Hub75TxSixteenBits<'d> = TxSixteenBits<'d>;

use static_cell::StaticCell;
pub struct Hub75<'d, DM: esp_hal::DriverMode> {
    parl_io: ParlIoTx<'d, DM>,
    tx_descriptors: &'static mut [DmaDescriptor],
}

impl<'d> Hub75<'d, esp_hal::Async> {
    pub fn new_async<CH>(
        parl_io: PARL_IO,
        hub75_pins: Hub75Pins, // TODO: how can we make this non-static?
        channel: impl Peripheral<P = CH> + 'd,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: HertzU32,
    ) -> Result<Self, Hub75Error>
    where
        CH: TxChannelFor<PARL_IO>,
    {
        let (parl_io, pins, clock_pin) =
            Self::new_internal(parl_io, hub75_pins, channel, frequency)?;
        let parl_io = parl_io.into_async().tx.with_config(
            pins,
            clock_pin,
            0,
            SampleEdge::Normal,
            BitPackOrder::Msb,
        )?;
        Ok(Self {
            parl_io,
            tx_descriptors,
        })
    }
}

impl<'d> Hub75<'d, esp_hal::Blocking> {
    pub fn new<CH>(
        parl_io: PARL_IO,
        hub75_pins: Hub75Pins, // TODO: how can we make this non-static?
        channel: impl Peripheral<P = CH> + 'd,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: HertzU32,
    ) -> Result<Self, Hub75Error>
    where
        CH: TxChannelFor<PARL_IO>,
    {
        let (parl_io, pins, clock_pin) =
            Self::new_internal(parl_io, hub75_pins, channel, frequency)?;
        let parl_io =
            parl_io
                .tx
                .with_config(pins, clock_pin, 0, SampleEdge::Normal, BitPackOrder::Msb)?;
        Ok(Self {
            parl_io,
            tx_descriptors,
        })
    }
}

impl<'d, DM: esp_hal::DriverMode> Hub75<'d, DM> {
    fn new_internal<CH>(
        parl_io: PARL_IO,
        hub75_pins: Hub75Pins, // TODO: how can we make this non-static?
        channel: impl Peripheral<P = CH> + 'd,
        frequency: HertzU32,
    ) -> Result<
        (
            ParlIoTxOnly<'d, esp_hal::Blocking>,
            &'static mut TxSixteenBits<'static>,
            &'static mut ClkOutPin<'static>,
        ),
        esp_hal::parl_io::Error,
    >
    where
        CH: TxChannelFor<PARL_IO>,
    {
        let (_, blank) = hub75_pins.blank.split();

        // TODO: how can we make this non-static?
        cfg_if::cfg_if! {
            if #[cfg(feature = "valid-pin")] {
                static PINS: StaticCell<TxPinConfigIncludingValidPin<Hub75TxSixteenBits<'static>>> = StaticCell::new();
                let pins = PINS.init(TxPinConfigIncludingValidPin::new(TxSixteenBits::new(
                    hub75_pins.addr0,
                    hub75_pins.addr1,
                    hub75_pins.addr2,
                    hub75_pins.addr3,
                    hub75_pins.addr4,
                    hub75_pins.latch,
                    NoPin,
                    NoPin,
                    blank.inverted(),
                    hub75_pins.red1,
                    hub75_pins.grn1,
                    hub75_pins.blu1,
                    hub75_pins.red2,
                    hub75_pins.grn2,
                    hub75_pins.blu2,
                    hub75_pins.valid,
                )));
            } else {
                static PINS: StaticCell<Hub75TxSixteenBits<'static>> = StaticCell::new();
                let pins = PINS.init(TxSixteenBits::new(
                    hub75_pins.addr0,
                    hub75_pins.addr1,
                    hub75_pins.addr2,
                    hub75_pins.addr3,
                    hub75_pins.addr4,
                    hub75_pins.latch,
                    NoPin,
                    NoPin,
                    blank.inverted(),
                    hub75_pins.red1,
                    hub75_pins.grn1,
                    hub75_pins.blu1,
                    hub75_pins.red2,
                    hub75_pins.grn2,
                    hub75_pins.blu2,
                    NoPin,
                ));
            }
        }

        // TODO: how can we make this non-static?
        static CLOCK_PIN: StaticCell<ClkOutPin> = StaticCell::new();
        let clock_pin = CLOCK_PIN.init(ClkOutPin::new(hub75_pins.clock));
        let parl_io = ParlIoTxOnly::new(parl_io, channel, frequency)?;
        Ok((parl_io, pins, clock_pin))
    }

    pub fn render<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    >(
        self,
        fb: &mut DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>,
    ) -> Result<Hub75Transfer<'d, DM>, (Hub75Error, Self)> {
        let parl_io = self.parl_io;
        let tx_descriptors = self.tx_descriptors;
        let tx_buffer = unsafe {
            use esp_hal::dma::ReadBuffer;
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

pub struct Hub75Transfer<'d, DM: esp_hal::DriverMode> {
    pub xfer: ParlIoTxTransfer<'d, DmaTxBuf, DM>,
}

impl<'d, DM: esp_hal::DriverMode> Hub75Transfer<'d, DM> {
    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        self.xfer.is_done()
    }

    /// Waits for the transfer to finish and returns the peripheral and buffer.
    ///
    /// Note: This also clears the transfer interrupt so it can be used in
    /// interrupt handlers to "handle" the interrupt.
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

impl<'d> Hub75Transfer<'d, esp_hal::Async> {
    pub async fn wait_for_done(&mut self) -> Result<(), DmaError> {
        self.xfer.wait_for_done().await;
        Ok(())
    }
}
