//! Shared ISR-driven BCM refresh infrastructure.
//!
//! This module is compiled for all platforms that use an interrupt-driven
//! display refresh loop (ESP32 via I2S Parallel, ESP32-S3 via LCD_CAM,
//! ESP32-C5/C6 via PARL_IO). It contains the transfer state machine, the
//! interrupt handler, and the public [`Hub75`] driver handle with `start()` /
//! `swap()` API.

use core::cell::RefCell;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;
use core::task::Waker;

use critical_section::Mutex;
use esp_hal::handler;
#[cfg(feature = "iram")]
use esp_hal::ram;
use esp_hal::Blocking;

use crate::bcm_buf::planes_from_fb;
use crate::bcm_buf::BcmBuf;
use crate::bcm_buf::PlaneInfo;
use crate::framebuffer::FrameBuffer;
#[cfg(feature = "esp32s3")]
use crate::framebuffer::WordSize;
use crate::Hub75Error;

// ---------------------------------------------------------------------------
// Platform-specific type aliases
// ---------------------------------------------------------------------------

#[cfg(feature = "esp32")]
pub(crate) type TxDriver = esp_hal::i2s::parallel::I2sParallel<'static, Blocking>;

#[cfg(feature = "esp32")]
pub(crate) type TxTransfer = esp_hal::i2s::parallel::I2sParallelTransfer<'static, BcmBuf, Blocking>;

#[cfg(any(feature = "esp32c5", feature = "esp32c6"))]
pub(crate) type TxDriver = esp_hal::parl_io::ParlIoTx<'static, Blocking>;

#[cfg(any(feature = "esp32c5", feature = "esp32c6"))]
pub(crate) type TxTransfer = esp_hal::parl_io::ParlIoTxTransfer<'static, BcmBuf, Blocking>;

#[cfg(feature = "esp32s3")]
pub(crate) type TxDriver = esp_hal::lcd_cam::lcd::i8080::I8080<'static, Blocking>;

#[cfg(feature = "esp32s3")]
pub(crate) type TxTransfer = esp_hal::lcd_cam::lcd::i8080::I8080Transfer<'static, BcmBuf, Blocking>;

// ---------------------------------------------------------------------------
// ISR shared state
// ---------------------------------------------------------------------------

pub(crate) enum TransferPhase {
    Idle(TxDriver, BcmBuf),
    InFlight(TxTransfer),
    Error(Hub75Error, TxDriver, BcmBuf),
    Transitioning,
}

pub(crate) struct IsrState {
    pub(crate) transfer: TransferPhase,
    #[cfg(feature = "esp32s3")]
    pub(crate) word_size: crate::framebuffer::WordSize,

    pub(crate) current_fb_ptr: *const (),
    pub(crate) pending_planes: Option<PlaneInfo>,
    pub(crate) pending_fb_ptr: *const (),
    pub(crate) returned_fb_ptr: *const (),
}

// SAFETY: All access is serialised by `critical_section`.
unsafe impl Send for IsrState {}

type SharedIsrState = Mutex<RefCell<Option<IsrState>>>;

static ISR_STATE: SharedIsrState = Mutex::new(RefCell::new(None));
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
// Platform-specific transfer helpers
// ---------------------------------------------------------------------------

#[cfg(feature = "esp32")]
#[cfg_attr(feature = "iram", ram)]
fn start_transfer(tx: TxDriver, buf: BcmBuf) -> Result<TxTransfer, (Hub75Error, TxDriver, BcmBuf)> {
    tx.send(buf)
        .map_err(|(err, tx, buf)| (Hub75Error::Dma(err), tx, buf))
}

#[cfg(feature = "esp32")]
#[cfg_attr(feature = "iram", ram)]
fn finish_transfer(xfer: TxTransfer) -> (Result<(), Hub75Error>, TxDriver, BcmBuf) {
    let (tx, buf) = xfer.wait();
    (Ok(()), tx, buf)
}

#[cfg(any(feature = "esp32c5", feature = "esp32c6"))]
#[cfg_attr(feature = "iram", ram)]
fn start_transfer(tx: TxDriver, buf: BcmBuf) -> Result<TxTransfer, (Hub75Error, TxDriver, BcmBuf)> {
    #[cfg(feature = "esp32c5")]
    let transfer_len = 0;
    #[cfg(not(feature = "esp32c5"))]
    let transfer_len = buf.current_transfer_len();

    tx.write(transfer_len, buf)
        .map_err(|(err, tx, buf)| (Hub75Error::ParlIo(err), tx, buf))
}

#[cfg(any(feature = "esp32c5", feature = "esp32c6"))]
#[cfg_attr(feature = "iram", ram)]
fn finish_transfer(xfer: TxTransfer) -> (Result<(), Hub75Error>, TxDriver, BcmBuf) {
    let (result, tx, buf) = xfer.wait();
    (result.map_err(Hub75Error::Dma), tx, buf)
}

#[cfg(feature = "esp32s3")]
#[cfg_attr(feature = "iram", ram)]
fn start_transfer(
    tx: TxDriver,
    buf: BcmBuf,
    word_size: WordSize,
) -> Result<TxTransfer, (Hub75Error, TxDriver, BcmBuf)> {
    use esp_hal::lcd_cam::lcd::i8080::Command;

    let result = match word_size {
        WordSize::Eight => tx.send(Command::<u8>::None, 0, buf),
        WordSize::Sixteen => tx.send(Command::<u16>::None, 0, buf),
    };
    result.map_err(|(err, tx, buf)| (Hub75Error::Dma(err), tx, buf))
}

#[cfg(feature = "esp32s3")]
#[cfg_attr(feature = "iram", ram)]
fn finish_transfer(xfer: TxTransfer) -> (Result<(), Hub75Error>, TxDriver, BcmBuf) {
    let (result, tx, buf) = xfer.wait();
    (result.map_err(Hub75Error::Dma), tx, buf)
}

// ---------------------------------------------------------------------------
// Interrupt handler
// ---------------------------------------------------------------------------

#[handler]
#[cfg_attr(feature = "iram", ram)]
pub(crate) fn hub75_isr() {
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

        // .wait() returns instantly — the interrupt already fired.
        let (result, tx, mut buf) = finish_transfer(xfer);

        if let Err(err) = result {
            state.transfer = TransferPhase::Error(err, tx, buf);
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

        #[cfg(feature = "esp32s3")]
        let xfer_result = start_transfer(tx, buf, state.word_size);
        #[cfg(not(feature = "esp32s3"))]
        let xfer_result = start_transfer(tx, buf);

        match xfer_result {
            Ok(new_xfer) => {
                state.transfer = TransferPhase::InFlight(new_xfer);
            }
            Err((hub_err, tx, buf)) => {
                state.transfer = TransferPhase::Error(hub_err, tx, buf);
                HAS_ERROR.store(true, Ordering::Release);
                signal_swap_done(cs);
            }
        }
    });
}

// ---------------------------------------------------------------------------
// ISR state initialisation (called by platform constructors)
// ---------------------------------------------------------------------------

#[cfg(not(feature = "esp32s3"))]
pub(crate) fn init_isr_state(tx: TxDriver, buf: BcmBuf) {
    critical_section::with(|cs| {
        *ISR_STATE.borrow_ref_mut(cs) = Some(IsrState {
            transfer: TransferPhase::Idle(tx, buf),
            current_fb_ptr: core::ptr::null(),
            pending_planes: None,
            pending_fb_ptr: core::ptr::null(),
            returned_fb_ptr: core::ptr::null(),
        });
    });
}

#[cfg(feature = "esp32s3")]
pub(crate) fn init_isr_state(tx: TxDriver, buf: BcmBuf, word_size: WordSize) {
    critical_section::with(|cs| {
        *ISR_STATE.borrow_ref_mut(cs) = Some(IsrState {
            transfer: TransferPhase::Idle(tx, buf),
            word_size,
            current_fb_ptr: core::ptr::null(),
            pending_planes: None,
            pending_fb_ptr: core::ptr::null(),
            returned_fb_ptr: core::ptr::null(),
        });
    });
}

// ---------------------------------------------------------------------------
// Public driver handle
// ---------------------------------------------------------------------------

/// HUB75 display controller driven by an interrupt-based BCM refresh loop.
///
/// Created in an idle state via [`Hub75::new`] or [`Hub75::new_async`].
/// Call [`Hub75::start`] with a framebuffer to begin DMA-driven display
/// refresh. The peripheral's transfer-done interrupt drives the entire BCM
/// refresh loop automatically.
///
/// The `FB` type parameter tracks the framebuffer type and is set by
/// [`Hub75::start`]. Constructors return `Hub75<DM>` (i.e. `Hub75<DM, ()>`)
/// and `start()` consumes it, returning a `Hub75<DM, FB>` that ensures
/// [`Hub75::swap`] always uses the same concrete framebuffer type.
///
/// If [`Hub75::swap`] returns an error, the driver stops and can be
/// restarted by calling [`Hub75::start`] again.
///
/// The `DM` type parameter selects the swap API:
/// - [`Hub75<Blocking, FB>`](esp_hal::Blocking): [`swap()`](Hub75::swap)
///   spin-loops until the next frame boundary.
/// - [`Hub75<Async, FB>`](esp_hal::Async): [`swap()`](Hub75::swap) is an
///   async method that yields to the executor.
pub struct Hub75<DM: esp_hal::DriverMode, FB = ()> {
    _dm: core::marker::PhantomData<DM>,
    _fb: core::marker::PhantomData<fn() -> FB>,
}

// SAFETY: Hub75 is a zero-sized handle; all mutable state lives in statics
// guarded by critical_section, so it is safe to send across threads.
unsafe impl<DM: esp_hal::DriverMode, FB> Send for Hub75<DM, FB> {}

impl<DM: esp_hal::DriverMode> Hub75<DM> {
    pub(crate) fn from_phantom() -> Self {
        Self {
            _dm: core::marker::PhantomData,
            _fb: core::marker::PhantomData,
        }
    }

    /// Start display refresh with the given framebuffer.
    ///
    /// Consumes the idle driver handle and returns a typed handle that binds
    /// the framebuffer type `FB`. This ensures that all subsequent calls to
    /// [`Hub75::swap`] use the same concrete framebuffer type.
    ///
    /// # Panics
    ///
    /// Panics if called while a transfer is already in flight.
    pub fn start<FB: FrameBuffer + 'static>(
        self,
        fb: &'static FB,
    ) -> Result<Hub75<DM, FB>, Hub75Error> {
        start_internal(fb)?;
        Ok(Hub75 {
            _dm: core::marker::PhantomData,
            _fb: core::marker::PhantomData,
        })
    }
}

impl<DM: esp_hal::DriverMode, FB: FrameBuffer + 'static> Hub75<DM, FB> {
    /// Restart display refresh after an error.
    ///
    /// Callable after [`Hub75::swap`] returned an error. Sets up the BCM
    /// plane data and kicks off the first DMA transfer with the same
    /// framebuffer type.
    ///
    /// # Panics
    ///
    /// Panics if called while a transfer is already in flight.
    pub fn restart(&self, fb: &'static FB) -> Result<(), Hub75Error> {
        start_internal(fb)
    }

    /// Returns the number of complete BCM frames rendered since driver
    /// creation.
    pub fn frame_count(&self) -> u32 {
        FRAME_COUNT.load(Ordering::Relaxed)
    }
}

fn start_internal(fb: &'static impl FrameBuffer) -> Result<(), Hub75Error> {
    let planes = planes_from_fb(fb);
    let plane_count = fb.plane_count();

    critical_section::with(|cs| {
        let mut borrow = ISR_STATE.borrow_ref_mut(cs);
        let state = borrow.as_mut().expect("Hub75 not initialised");

        let (tx, mut buf) =
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

        #[cfg(feature = "esp32s3")]
        let xfer_result = start_transfer(tx, buf, state.word_size);
        #[cfg(not(feature = "esp32s3"))]
        let xfer_result = start_transfer(tx, buf);

        match xfer_result {
            Ok(xfer) => {
                state.transfer = TransferPhase::InFlight(xfer);
                HAS_ERROR.store(false, Ordering::Release);
                SWAP_DONE.store(false, Ordering::Relaxed);
                Ok(())
            }
            Err((hub_err, tx, buf)) => {
                state.transfer = TransferPhase::Error(hub_err, tx, buf);
                HAS_ERROR.store(true, Ordering::Release);
                Err(hub_err)
            }
        }
    })
}

impl<FB: FrameBuffer + 'static> Hub75<Blocking, FB> {
    /// Replace the displayed framebuffer.
    ///
    /// Blocks (spin-loops) until the current BCM frame completes, then
    /// atomically swaps in `new_fb` and returns exclusive access to the
    /// previously displayed framebuffer.
    ///
    /// Returns `Err` if the ISR encountered a DMA error. After an error the
    /// driver is stopped; call [`Hub75::start`] to restart.
    pub fn swap(&self, new_fb: &'static mut FB) -> Result<&'static mut FB, Hub75Error> {
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
            // SAFETY: The typestate guarantees that the returned pointer
            // originated from a `&'static mut FB` passed to a prior
            // `start()` or `swap()` call with the same `FB` type. The ISR
            // has finished using this buffer (it swapped to the new one at
            // the frame boundary), so exclusive access is restored.
            Ok(unsafe { &mut *(state.returned_fb_ptr as *mut FB) })
        })
    }
}

impl<FB: FrameBuffer + 'static> Hub75<esp_hal::Async, FB> {
    /// Replace the displayed framebuffer.
    ///
    /// Yields to the executor until the current BCM frame completes, then
    /// atomically swaps in `new_fb` and returns exclusive access to the
    /// previously displayed framebuffer.
    ///
    /// Returns `Err` if the ISR encountered a DMA error. After an error the
    /// driver is stopped; call [`Hub75::start`] to restart.
    pub async fn swap(&self, new_fb: &'static mut FB) -> Result<&'static mut FB, Hub75Error> {
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
            // SAFETY: The typestate guarantees that the returned pointer
            // originated from a `&'static mut FB` passed to a prior
            // `start()` or `swap()` call with the same `FB` type. The ISR
            // has finished using this buffer (it swapped to the new one at
            // the frame boundary), so exclusive access is restored.
            Ok(unsafe { &mut *(state.returned_fb_ptr as *mut FB) })
        })
    }
}
