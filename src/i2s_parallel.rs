use core::cell::Cell;

use esp_hal::dma::Channel;
use esp_hal::dma::DmaChannelConvert;
use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaEligible;
use esp_hal::dma::DmaTxBuf;
use esp_hal::gpio::NoPin;
use esp_hal::i2s::parallel::I2sParallel;
use esp_hal::i2s::parallel::TxSixteenBits;
use esp_hal::i2s::parallel::AnyI2s;
use esp_hal::peripheral::Peripheral;

use crate::framebuffer::DmaFrameBuffer;
use crate::HertzU32;
use crate::Hub75Pins;

pub struct Hub75<'d, DM: esp_hal::Mode> {
    i2s: Cell<Option<I2sParallel<'d, DM>>>,
    tx_descriptors: Cell<Option<&'static mut [DmaDescriptor]>>,
}

impl<'d> Hub75<'d, esp_hal::Async> {
    // pub fn new_async(
    //     i2s: I2S0,
    //     hub75_pins: Hub75Pins,
    //     channel: Channel<esp_hal::Async>,
    //     tx_descriptors: &'static mut [DmaDescriptor],
    //     frequency: HertzU32,
    // ) -> Self {
    //     let lcd_cam = I2sParallel::new(i2s);
    //     Self::new_internal(i2s, hub75_pins, channel, tx_descriptors, frequency)
    // }

    pub async fn render_async<
        const ROWS: usize,
        const COLS: usize,
        const BITS: u8,
        const SIZE: usize,
    >(
        &mut self,
        fb: &mut DmaFrameBuffer<ROWS, COLS, BITS, SIZE>,
    ) {
        let i2s = self.i2s.take().expect("i2s is None!");
        let tx_descriptors = self.tx_descriptors.take().expect("tx_descriptors is None!");
        let tx_buffer = unsafe {
            use esp_hal::dma::ReadBuffer;
            let (ptr, len) = fb.read_buffer();
            // SAFETY: tx_buffer is only used until the tx_buf.split below!
            core::slice::from_raw_parts_mut(ptr as *mut u8, len)
        };
        let tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).expect("DmaTxBuf::new failed");
        let mut xfer = match i2s.send(tx_buf) {
            Ok(xfer) => xfer,
            Err(_) => {
                panic!("Failed to send buffer");
            }
        };
        xfer.wait_for_done().await.expect("transfer failed");
        let (i2s, tx_buf) = xfer.wait();
        let (tx_descriptors, _) = tx_buf.split();
        self.i2s.set(Some(i2s));
        self.tx_descriptors.set(Some(tx_descriptors));
    }
}

impl<'d, DM: esp_hal::Mode> Hub75<'d, DM> {
    pub fn new<CH: DmaChannelConvert<<AnyI2s as DmaEligible>::Dma>>(
        i2s: impl Peripheral<P = AnyI2s> + 'd,
        hub75_pins: Hub75Pins,
        channel: Channel<'d, CH, DM>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: HertzU32,
    ) -> Self {
        let (_, blank) = hub75_pins.blank.split();
        let pins = TxSixteenBits::new(
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
        );

        let i2s = I2sParallel::new(i2s, channel, frequency, pins, hub75_pins.clock);
        let i2s = Cell::new(Some(i2s));
        let tx_descriptors = Cell::new(Some(tx_descriptors));
        Self {
            i2s,
            tx_descriptors,
        }
    }
}
