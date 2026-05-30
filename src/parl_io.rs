//! HUB75 driver for PARL_IO peripherals (ESP32-C5 / ESP32-C6).
//!
//! This module provides an interrupt-driven display controller that
//! continuously refreshes a HUB75 panel from a framebuffer. The PARL_IO
//! `TxEof` interrupt drives the entire BCM (Binary Code Modulation) refresh
//! loop. Buffer swaps happen atomically at frame boundaries.
//!
//! # Blocking example
//!
//! ```rust,ignore
//! let hub75 = Hub75::new(
//!     peripherals.PARL_IO, pins, peripherals.DMA_CH0,
//!     tx_descriptors, Rate::from_mhz(20),
//! ).expect("failed to create Hub75");
//! hub75.start(&*fb).expect("failed to start Hub75");
//!
//! // Display refreshes automatically — main thread is free.
//! loop { core::hint::spin_loop(); }
//! ```
//!
//! # Async example
//!
//! ```rust,ignore
//! let hub75 = Hub75::new_async(
//!     peripherals.PARL_IO, pins, peripherals.DMA_CH0,
//!     tx_descriptors, Rate::from_mhz(20),
//! ).expect("failed to create Hub75");
//! hub75.start(&*fb0).expect("failed to start Hub75");
//!
//! // Swap buffers — yields to the executor, returns Err on DMA failure.
//! let old_ptr = hub75.swap(&*fb1).await.expect("DMA error");
//! ```

use core::cell::RefCell;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;
use core::task::Waker;

use critical_section::Mutex;
use esp_hal::dma::DmaChannelFor;
use esp_hal::dma::DmaDescriptor;
use esp_hal::handler;
use esp_hal::parl_io::BitPackOrder;
use esp_hal::parl_io::ClkOutPin;
use esp_hal::parl_io::ConfigurePins;
use esp_hal::parl_io::ParlIo;
use esp_hal::parl_io::ParlIoInterrupt;
use esp_hal::parl_io::ParlIoTx;
use esp_hal::parl_io::ParlIoTxTransfer;
use esp_hal::parl_io::SampleEdge;
use esp_hal::parl_io::TxConfig;
use esp_hal::parl_io::TxPins;
use esp_hal::peripherals::PARL_IO;
#[cfg(feature = "iram")]
use esp_hal::ram;
use esp_hal::time::Rate;
use esp_hal::Blocking;

use crate::bcm_buf::planes_from_fb;
use crate::bcm_buf::BcmBuf;
use crate::bcm_buf::PlaneInfo;
use crate::framebuffer::FrameBuffer;
use crate::Hub75Error;
use crate::Hub75Pins;
#[cfg(not(feature = "esp32c5"))]
use crate::Hub75Pins16;
use crate::Hub75Pins8;

// ---------------------------------------------------------------------------
// ISR shared state
// ---------------------------------------------------------------------------

enum TransferPhase {
    Idle(ParlIoTx<'static, Blocking>, BcmBuf),
    InFlight(ParlIoTxTransfer<'static, BcmBuf, Blocking>),
    Error(Hub75Error, ParlIoTx<'static, Blocking>, BcmBuf),
    Transitioning,
}

struct IsrState {
    transfer: TransferPhase,

    current_fb_ptr: *const (),

    pending_planes: Option<PlaneInfo>,
    pending_fb_ptr: *const (),

    returned_fb_ptr: *const (),
}

// SAFETY: All access is serialised by `critical_section` on single-core C5/C6.
unsafe impl Send for IsrState {}

static ISR_STATE: Mutex<RefCell<Option<IsrState>>> = Mutex::new(RefCell::new(None));
static SWAP_DONE: AtomicBool = AtomicBool::new(false);
static HAS_ERROR: AtomicBool = AtomicBool::new(false);
static SWAP_WAKER: Mutex<RefCell<Option<Waker>>> = Mutex::new(RefCell::new(None));
static FRAME_COUNT: AtomicU32 = AtomicU32::new(0);

fn signal_swap_done(cs: critical_section::CriticalSection) {
    SWAP_DONE.store(true, Ordering::Release);
    if let Some(waker) = SWAP_WAKER.borrow_ref_mut(cs).take() {
        waker.wake();
    }
}

// ---------------------------------------------------------------------------
// TxEof interrupt handler
// ---------------------------------------------------------------------------

#[handler]
#[cfg_attr(feature = "iram", ram)]
fn hub75_isr() {
    critical_section::with(|cs| {
        let mut borrow = ISR_STATE.borrow_ref_mut(cs);
        let state = match borrow.as_mut() {
            Some(s) => s,
            None => return,
        };

        let xfer = match core::mem::replace(&mut state.transfer, TransferPhase::Transitioning) {
            TransferPhase::InFlight(xfer) => xfer,
            other => {
                state.transfer = other;
                return;
            }
        };

        // .wait() returns instantly — TxEof already fired.
        let (result, parl_io_tx, mut buf) = xfer.wait();

        if let Err(err) = result {
            state.transfer = TransferPhase::Error(Hub75Error::Dma(err), parl_io_tx, buf);
            HAS_ERROR.store(true, Ordering::Release);
            signal_swap_done(cs);
            return;
        }

        let frame_boundary = buf.advance();

        if frame_boundary {
            FRAME_COUNT.fetch_add(1, Ordering::Relaxed);

            if let Some(pending) = state.pending_planes.take() {
                state.returned_fb_ptr = state.current_fb_ptr;
                state.current_fb_ptr = state.pending_fb_ptr;
                state.pending_fb_ptr = core::ptr::null();
                buf.update_planes(pending);
                signal_swap_done(cs);
            }
        }

        #[cfg(feature = "esp32c5")]
        let transfer_len = 0;
        #[cfg(not(feature = "esp32c5"))]
        let transfer_len = buf.current_transfer_len();

        match parl_io_tx.write(transfer_len, buf) {
            Ok(new_xfer) => {
                state.transfer = TransferPhase::InFlight(new_xfer);
            }
            Err((err, parl_io_tx, buf)) => {
                state.transfer = TransferPhase::Error(Hub75Error::ParlIo(err), parl_io_tx, buf);
                HAS_ERROR.store(true, Ordering::Release);
                signal_swap_done(cs);
            }
        }
    });
}

// ---------------------------------------------------------------------------
// Public driver handle
// ---------------------------------------------------------------------------

/// HUB75 display controller for PARL_IO peripherals.
///
/// Created in an idle state via [`Hub75::new`] or [`Hub75::new_async`].
/// Call [`Hub75::start`] with a framebuffer to begin DMA-driven display
/// refresh. The PARL_IO `TxEof` interrupt drives the entire BCM refresh
/// loop automatically.
///
/// If [`Hub75::swap`] returns an error, the driver stops and can be
/// restarted by calling [`Hub75::start`] again (possibly with a different
/// framebuffer).
///
/// The `DM` type parameter selects the swap API:
/// - [`Hub75<Blocking>`](esp_hal::Blocking): [`swap()`](Hub75::swap) spin-loops
///   until the next frame boundary.
/// - [`Hub75<Async>`](esp_hal::Async): [`swap()`](Hub75::swap) is an async
///   method that yields to the executor.
pub struct Hub75<DM: esp_hal::DriverMode> {
    _dm: core::marker::PhantomData<DM>,
}

impl<DM: esp_hal::DriverMode> Hub75<DM> {
    fn new_internal<T: TxPins + ConfigurePins + 'static>(
        parl_io: PARL_IO<'static>,
        hub75_pins: impl Hub75Pins<'static, T>,
        channel: impl DmaChannelFor<PARL_IO<'static>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
    ) -> Result<Self, Hub75Error> {
        let (pins, clock_pin) = hub75_pins.convert_pins();

        let mut parl_io_dev = ParlIo::new(parl_io, channel)?;

        parl_io_dev.set_interrupt_handler(hub75_isr);
        parl_io_dev.listen(ParlIoInterrupt::TxEof);

        #[cfg(feature = "invert-clock")]
        let sample_edge = SampleEdge::Normal;
        #[cfg(not(feature = "invert-clock"))]
        let sample_edge = SampleEdge::Invert;

        let config = TxConfig::default()
            .with_frequency(frequency)
            .with_idle_value(0)
            .with_sample_edge(sample_edge)
            .with_bit_order(BitPackOrder::Msb);

        let clk_pin = ClkOutPin::new(clock_pin);
        let parl_io_tx = parl_io_dev.tx.with_config(pins, clk_pin, config)?;

        #[cfg(feature = "esp32c5")]
        unsafe {
            use esp32c5 as pac;
            let pio = pac::PARL_IO::steal();
            pio.tx_genrl_cfg()
                .modify(|_, w| w.tx_eof_gen_sel().set_bit());
        }

        let buf = BcmBuf::new(tx_descriptors);

        critical_section::with(|cs| {
            *ISR_STATE.borrow_ref_mut(cs) = Some(IsrState {
                transfer: TransferPhase::Idle(parl_io_tx, buf),
                current_fb_ptr: core::ptr::null(),
                pending_planes: None,
                pending_fb_ptr: core::ptr::null(),
                returned_fb_ptr: core::ptr::null(),
            });
        });

        Ok(Self {
            _dm: core::marker::PhantomData,
        })
    }

    /// Start (or restart) display refresh with the given framebuffer.
    ///
    /// Callable from `Idle` state (after [`Hub75::new`] / [`Hub75::new_async`])
    /// or from `Error` state (after [`Hub75::swap`] returned an error).
    /// Sets up the BCM plane data and kicks off the first DMA transfer.
    ///
    /// # Panics
    ///
    /// Panics if called while a transfer is already in flight.
    pub fn start(&self, fb: &'static (impl FrameBuffer + 'static)) -> Result<(), Hub75Error> {
        let planes = planes_from_fb(fb);
        let plane_count = fb.plane_count();

        critical_section::with(|cs| {
            let mut borrow = ISR_STATE.borrow_ref_mut(cs);
            let state = borrow.as_mut().expect("Hub75 not initialised");

            let (parl_io_tx, mut buf) =
                match core::mem::replace(&mut state.transfer, TransferPhase::Transitioning) {
                    TransferPhase::Idle(tx, buf) | TransferPhase::Error(_, tx, buf) => (tx, buf),
                    other => {
                        state.transfer = other;
                        panic!("start() called while already running");
                    }
                };

            buf.reset_with_planes(planes, plane_count);
            state.current_fb_ptr = fb as *const _ as *const ();
            state.pending_planes = None;
            state.pending_fb_ptr = core::ptr::null();
            state.returned_fb_ptr = core::ptr::null();

            #[cfg(feature = "esp32c5")]
            let transfer_len = 0;
            #[cfg(not(feature = "esp32c5"))]
            let transfer_len = buf.current_transfer_len();

            match parl_io_tx.write(transfer_len, buf) {
                Ok(xfer) => {
                    state.transfer = TransferPhase::InFlight(xfer);
                    HAS_ERROR.store(false, Ordering::Release);
                    SWAP_DONE.store(false, Ordering::Relaxed);
                    Ok(())
                }
                Err((err, tx, buf)) => {
                    let hub_err = Hub75Error::ParlIo(err);
                    state.transfer = TransferPhase::Error(hub_err, tx, buf);
                    HAS_ERROR.store(true, Ordering::Release);
                    Err(hub_err)
                }
            }
        })
    }

    /// Returns the number of complete BCM frames rendered since driver
    /// creation.
    pub fn frame_count(&self) -> u32 {
        FRAME_COUNT.load(Ordering::Relaxed)
    }
}

impl Hub75<Blocking> {
    /// Create a new blocking HUB75 driver.
    ///
    /// The driver is created in an idle state. Call [`Hub75::start`] with a
    /// framebuffer to begin display refresh.
    ///
    /// # Arguments
    /// * `parl_io` -- The PARL_IO peripheral instance
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `frequency` -- PARL_IO clock rate
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new<T: TxPins + ConfigurePins + 'static>(
        parl_io: PARL_IO<'static>,
        hub75_pins: impl Hub75Pins<'static, T>,
        channel: impl DmaChannelFor<PARL_IO<'static>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(parl_io, hub75_pins, channel, tx_descriptors, frequency)
    }

    /// Replace the displayed framebuffer.
    ///
    /// Blocks (spin-loops) until the current BCM frame completes, then
    /// atomically swaps in `new_fb` and returns the old framebuffer pointer.
    ///
    /// Returns `Err` if the ISR encountered a DMA error. After an error the
    /// driver is stopped; call [`Hub75::start`] to restart with a (possibly
    /// different) framebuffer.
    ///
    /// # Safety contract
    ///
    /// The caller **must** ensure that every call to `swap` uses the same
    /// concrete framebuffer type that was passed to [`Hub75::start`]. The
    /// returned raw pointer must be cast back to that same type.
    pub fn swap(&self, new_fb: &'static impl FrameBuffer) -> Result<*const (), Hub75Error> {
        let new_planes = planes_from_fb(new_fb);

        critical_section::with(|cs| {
            let mut borrow = ISR_STATE.borrow_ref_mut(cs);
            let state = borrow.as_mut().expect("Hub75 not initialised");
            state.pending_planes = Some(new_planes);
            state.pending_fb_ptr = new_fb as *const _ as *const ();
            SWAP_DONE.store(false, Ordering::Relaxed);
        });

        loop {
            if HAS_ERROR.load(Ordering::Acquire) {
                return critical_section::with(|cs| {
                    let borrow = ISR_STATE.borrow_ref(cs);
                    let state = borrow.as_ref().expect("Hub75 not initialised");
                    match &state.transfer {
                        TransferPhase::Error(err, _, _) => Err(*err),
                        _ => Err(Hub75Error::Dma(esp_hal::dma::DmaError::DescriptorError)),
                    }
                });
            }
            if SWAP_DONE.load(Ordering::Acquire) {
                break;
            }
            core::hint::spin_loop();
        }

        critical_section::with(|cs| {
            let borrow = ISR_STATE.borrow_ref(cs);
            let state = borrow.as_ref().expect("Hub75 not initialised");
            Ok(state.returned_fb_ptr)
        })
    }
}

impl Hub75<esp_hal::Async> {
    /// Create a new async HUB75 driver.
    ///
    /// The driver is created in an idle state. Call [`Hub75::start`] with a
    /// framebuffer to begin display refresh.
    ///
    /// # Arguments
    /// * `parl_io` -- The PARL_IO peripheral instance
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `frequency` -- PARL_IO clock rate
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new_async<T: TxPins + ConfigurePins + 'static>(
        parl_io: PARL_IO<'static>,
        hub75_pins: impl Hub75Pins<'static, T>,
        channel: impl DmaChannelFor<PARL_IO<'static>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(parl_io, hub75_pins, channel, tx_descriptors, frequency)
    }

    /// Replace the displayed framebuffer.
    ///
    /// Yields to the executor until the current BCM frame completes, then
    /// atomically swaps in `new_fb` and returns the old framebuffer pointer.
    ///
    /// Returns `Err` if the ISR encountered a DMA error. After an error the
    /// driver is stopped; call [`Hub75::start`] to restart with a (possibly
    /// different) framebuffer.
    ///
    /// # Safety contract
    ///
    /// The caller **must** ensure that every call to `swap` uses the same
    /// concrete framebuffer type that was passed to [`Hub75::start`].
    /// The returned raw pointer must be cast back to that same type.
    pub async fn swap(&self, new_fb: &'static impl FrameBuffer) -> Result<*const (), Hub75Error> {
        let new_planes = planes_from_fb(new_fb);

        critical_section::with(|cs| {
            let mut borrow = ISR_STATE.borrow_ref_mut(cs);
            let state = borrow.as_mut().expect("Hub75 not initialised");
            state.pending_planes = Some(new_planes);
            state.pending_fb_ptr = new_fb as *const _ as *const ();
            SWAP_DONE.store(false, Ordering::Relaxed);
        });

        core::future::poll_fn(|cx| {
            if SWAP_DONE.load(Ordering::Acquire) || HAS_ERROR.load(Ordering::Acquire) {
                return core::task::Poll::Ready(());
            }
            critical_section::with(|cs| {
                if SWAP_DONE.load(Ordering::Relaxed) || HAS_ERROR.load(Ordering::Relaxed) {
                    return core::task::Poll::Ready(());
                }
                *SWAP_WAKER.borrow_ref_mut(cs) = Some(cx.waker().clone());
                core::task::Poll::Pending
            })
        })
        .await;

        if HAS_ERROR.load(Ordering::Acquire) {
            return critical_section::with(|cs| {
                let borrow = ISR_STATE.borrow_ref(cs);
                let state = borrow.as_ref().expect("Hub75 not initialised");
                match &state.transfer {
                    TransferPhase::Error(err, _, _) => Err(*err),
                    _ => Err(Hub75Error::Dma(esp_hal::dma::DmaError::DescriptorError)),
                }
            });
        }

        critical_section::with(|cs| {
            let borrow = ISR_STATE.borrow_ref(cs);
            let state = borrow.as_ref().expect("Hub75 not initialised");
            Ok(state.returned_fb_ptr)
        })
    }
}

// ---------------------------------------------------------------------------
// Pin configurations
// ---------------------------------------------------------------------------

use esp_hal::gpio::AnyPin;
#[cfg(not(feature = "esp32c5"))]
use esp_hal::gpio::NoPin;
use esp_hal::parl_io::TxEightBits;
#[cfg(not(feature = "esp32c5"))]
use esp_hal::parl_io::TxSixteenBits;

#[cfg(not(feature = "esp32c5"))]
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
