use esp_hal::dma::Channel;
use esp_hal::dma::DmaChannelConvert;
use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaEligible;
use esp_hal::dma::ReadBuffer;
use esp_hal::gpio::NoPin;
use esp_hal::parl_io::BitPackOrder;
use esp_hal::parl_io::ClkOutPin;
use esp_hal::parl_io::ParlIoTx;
use esp_hal::parl_io::ParlIoTxOnly;
use esp_hal::parl_io::SampleEdge;
#[cfg(feature = "valid-pin")]
use esp_hal::parl_io::TxPinConfigIncludingValidPin;
use esp_hal::parl_io::TxSixteenBits;
use esp_hal::peripherals::PARL_IO;

use crate::framebuffer::DmaFrameBuffer;
use crate::HertzU32;
use crate::Hub75Error;
use crate::Hub75Pins;

type Hub75TxSixteenBits<'d> = TxSixteenBits<'d>;

use static_cell::StaticCell;
pub struct Hub75<'d, DM: esp_hal::Mode> {
    parl_io: ParlIoTx<'d, DM>,
}

impl<'d> Hub75<'d, esp_hal::Async> {
    pub fn new_async<CH>(
        parl_io: PARL_IO,
        hub75_pins: Hub75Pins, // TODO: how can we make this non-static?
        channel: Channel<'d, esp_hal::Blocking, CH>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: HertzU32,
    ) -> Result<Self, Hub75Error>
    where
        CH: DmaChannelConvert<<PARL_IO as DmaEligible>::Dma>,
    {
        Self::new(
            parl_io,
            hub75_pins,
            channel.into_async(),
            tx_descriptors,
            frequency,
        )
    }

    pub async fn render_async<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    >(
        &mut self,
        fb: &mut DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>,
    ) -> Result<(), Hub75Error> {
        // parl_io has a max size limit of 32736 bytes so we need to send the
        // framebuffer in chunks
        let buffer = unsafe {
            let (ptr, len) = fb.read_buffer();
            core::slice::from_raw_parts(ptr, len)
        };
        self.parl_io.write_dma_async(&buffer).await?;
        Ok(())
    }
}

impl<'d, DM: esp_hal::Mode> Hub75<'d, DM> {
    pub fn new<CH>(
        parl_io: PARL_IO,
        hub75_pins: Hub75Pins, // TODO: how can we make this non-static?
        channel: Channel<'d, DM, CH>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: HertzU32,
    ) -> Result<Self, Hub75Error>
    where
        CH: DmaChannelConvert<<PARL_IO as DmaEligible>::Dma>,
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
        let parl_io = ParlIoTxOnly::new(parl_io, channel, tx_descriptors, frequency)?;

        let parl_io =
            parl_io
                .tx
                .with_config(pins, clock_pin, 0, SampleEdge::Normal, BitPackOrder::Msb)?;
        Ok(Self { parl_io })
    }

    pub fn render<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    >(
        &mut self,
        fb: &mut DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>,
    ) -> Result<(), Hub75Error> {
        // parl_io has a max size limit of 32736 bytes so we need to send the
        // framebuffer in chunks
        let buffer = unsafe {
            let (ptr, len) = fb.read_buffer();
            core::slice::from_raw_parts(ptr, len)
        };
        let xfer = self.parl_io.write_dma(&buffer)?;
        xfer.wait()?;
        Ok(())
    }
}
