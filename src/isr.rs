//! Shared ISR-driven BCM refresh infrastructure.
//!
//! This module is compiled for all platforms that use an interrupt-driven
//! display refresh loop (ESP32 via I2S Parallel, ESP32-S3 via LCD_CAM,
//! ESP32-C5/C6 via PARL_IO). It contains the transfer state machine, the
//! interrupt handler, and the public [`Hub75`] driver handle with `swap()` API.

use core::cell::RefCell;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;
use core::task::Waker;

use critical_section::Mutex;
use esp_hal::Blocking;
use esp_hal::handler;
#[cfg(feature = "iram")]
use esp_hal::ram;

use crate::Hub75Error;
#[cfg(not(feature = "circular-dma"))]
use crate::bcm::SegmentCache;
#[cfg(feature = "circular-dma")]
use crate::bcm::circular::CircularBcmBuf;
#[cfg(not(feature = "circular-dma"))]
use crate::bcm::linear::BcmBuf;
#[cfg(not(feature = "circular-dma"))]
use crate::bcm::segments_from_fb;
use crate::framebuffer::FrameBuffer;
#[cfg(all(hub75_use_lcd_cam, not(feature = "circular-dma")))]
use crate::framebuffer::WordSize;

// ---------------------------------------------------------------------------
// Platform-specific type aliases
// ---------------------------------------------------------------------------

#[cfg(all(hub75_use_i2s_parallel, not(feature = "circular-dma")))]
pub(crate) type TxDriver = esp_hal::i2s::parallel::I2sParallel<'static, Blocking>;

#[cfg(all(hub75_use_i2s_parallel, not(feature = "circular-dma")))]
pub(crate) type TxTransfer = esp_hal::i2s::parallel::I2sParallelTransfer<'static, BcmBuf, Blocking>;

#[cfg(all(hub75_use_i2s_parallel, feature = "circular-dma"))]
pub(crate) type TxTransfer =
    esp_hal::i2s::parallel::I2sParallelTransfer<'static, CircularBcmBuf, Blocking>;

#[cfg(all(hub75_use_parl_io, not(feature = "circular-dma")))]
pub(crate) type TxDriver = esp_hal::parl_io::ParlIoTx<'static, Blocking>;

#[cfg(all(hub75_use_parl_io, not(feature = "circular-dma")))]
pub(crate) type TxTransfer = esp_hal::parl_io::ParlIoTxTransfer<'static, BcmBuf, Blocking>;

#[cfg(all(hub75_use_parl_io, feature = "circular-dma"))]
pub(crate) type TxTransfer = esp_hal::parl_io::ParlIoTxTransfer<'static, CircularBcmBuf, Blocking>;

#[cfg(all(hub75_use_lcd_cam, not(feature = "circular-dma")))]
pub(crate) type TxDriver = esp_hal::lcd_cam::lcd::i8080::I8080<'static, Blocking>;

#[cfg(all(hub75_use_lcd_cam, not(feature = "circular-dma")))]
pub(crate) type TxTransfer = esp_hal::lcd_cam::lcd::i8080::I8080Transfer<'static, BcmBuf, Blocking>;

#[cfg(all(hub75_use_lcd_cam, feature = "circular-dma"))]
pub(crate) type TxTransfer =
    esp_hal::lcd_cam::lcd::i8080::I8080Transfer<'static, CircularBcmBuf, Blocking>;

// ---------------------------------------------------------------------------
// ISR shared state
// ---------------------------------------------------------------------------

#[cfg(not(feature = "circular-dma"))]
pub(crate) enum TransferPhase {
    Idle(TxDriver, BcmBuf),
    InFlight(TxTransfer),
    Error(Hub75Error, TxDriver, BcmBuf),
    Transitioning,
}

#[cfg(not(feature = "circular-dma"))]
pub(crate) struct IsrState {
    pub(crate) transfer: TransferPhase,
    #[cfg(hub75_use_lcd_cam)]
    pub(crate) word_size: crate::framebuffer::WordSize,

    pub(crate) current_fb_ptr: *const (),
    pub(crate) pending_segments: Option<SegmentCache>,
    pub(crate) pending_fb_ptr: *const (),
}

#[cfg(not(feature = "circular-dma"))]
// SAFETY: All access is serialised by `critical_section`, which disables
// interrupts and acquires a cross-core spinlock on multi-core chips.
// The raw pointers (`current_fb_ptr`, `pending_fb_ptr`)
// are only dereferenced inside `critical_section::with` blocks, so they
// are never accessed concurrently from multiple cores.
unsafe impl Send for IsrState {}

#[cfg(not(feature = "circular-dma"))]
type SharedIsrState = Mutex<RefCell<Option<IsrState>>>;

#[cfg(not(feature = "circular-dma"))]
static ISR_STATE: SharedIsrState = Mutex::new(RefCell::new(None));

// ---------------------------------------------------------------------------
// Circular-DMA shared state
// ---------------------------------------------------------------------------

#[cfg(feature = "circular-dma")]
pub(crate) struct CircularState {
    // kept alive to prevent DMA from stopping
    pub(crate) _transfer: Option<TxTransfer>,
    pub(crate) descriptors: *mut esp_hal::dma::DmaDescriptor,
    pub(crate) desc_count: usize,
    pub(crate) current_fb_ptr: *const (),
}

#[cfg(feature = "circular-dma")]
// SAFETY: Same justification as IsrState — all access serialised by
// critical_section.
unsafe impl Send for CircularState {}

#[cfg(feature = "circular-dma")]
type SharedCircularState = Mutex<RefCell<Option<CircularState>>>;

#[cfg(feature = "circular-dma")]
static CIRCULAR_STATE: SharedCircularState = Mutex::new(RefCell::new(None));

// ---------------------------------------------------------------------------
// Swap signalling
// ---------------------------------------------------------------------------

static SWAP_DONE: AtomicBool = AtomicBool::new(false);
#[cfg(not(feature = "circular-dma"))]
static HAS_ERROR: AtomicBool = AtomicBool::new(false);
static SWAP_WAKER: Mutex<RefCell<Option<Waker>>> = Mutex::new(RefCell::new(None));
static FRAME_COUNT: AtomicU32 = AtomicU32::new(0);

#[cfg(feature = "circular-dma")]
#[allow(clippy::type_complexity)]
static CLEAR_INTERRUPT: Mutex<RefCell<Option<fn()>>> = Mutex::new(RefCell::new(None));

fn signal_swap_done(cs: critical_section::CriticalSection) {
    SWAP_DONE.store(true, Ordering::Release);
    if let Some(waker) = SWAP_WAKER.borrow_ref_mut(cs).take() {
        waker.wake();
    }
}

// ---------------------------------------------------------------------------
// Platform-specific transfer helpers (non-circular only)
// ---------------------------------------------------------------------------

#[cfg(all(hub75_use_i2s_parallel, not(feature = "circular-dma")))]
#[cfg_attr(feature = "iram", ram)]
fn start_transfer(tx: TxDriver, buf: BcmBuf) -> Result<TxTransfer, (Hub75Error, TxDriver, BcmBuf)> {
    tx.send(buf)
        .map_err(|(err, tx, buf)| (Hub75Error::Dma(err), tx, buf))
}

#[cfg(all(hub75_use_i2s_parallel, not(feature = "circular-dma")))]
#[cfg_attr(feature = "iram", ram)]
fn finish_transfer(xfer: TxTransfer) -> (Result<(), Hub75Error>, TxDriver, BcmBuf) {
    let (tx, buf) = xfer.wait();
    (Ok(()), tx, buf)
}

/// On ESP32-C5, the GDMA EOF signal is generated by the DMA channel rather
/// than the PARL_IO byte counter, so the transfer-length field is unused.
#[cfg(all(hub75_use_parl_io, not(feature = "circular-dma"), esp32c5))]
const PARL_IO_DUMMY_TRANSFER_LEN: usize = 0;

#[cfg(all(hub75_use_parl_io, not(feature = "circular-dma")))]
#[cfg_attr(feature = "iram", ram)]
fn start_transfer(tx: TxDriver, buf: BcmBuf) -> Result<TxTransfer, (Hub75Error, TxDriver, BcmBuf)> {
    #[cfg(esp32c5)]
    let transfer_len = PARL_IO_DUMMY_TRANSFER_LEN;
    #[cfg(not(esp32c5))]
    let transfer_len = buf.current_transfer_len();

    tx.write(transfer_len, buf)
        .map_err(|(err, tx, buf)| (Hub75Error::ParlIo(err), tx, buf))
}

#[cfg(all(hub75_use_parl_io, not(feature = "circular-dma")))]
#[cfg_attr(feature = "iram", ram)]
fn finish_transfer(xfer: TxTransfer) -> (Result<(), Hub75Error>, TxDriver, BcmBuf) {
    let (result, tx, buf) = xfer.wait();
    (result.map_err(Hub75Error::Dma), tx, buf)
}

#[cfg(all(hub75_use_lcd_cam, not(feature = "circular-dma")))]
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

#[cfg(all(hub75_use_lcd_cam, not(feature = "circular-dma")))]
#[cfg_attr(feature = "iram", ram)]
fn finish_transfer(xfer: TxTransfer) -> (Result<(), Hub75Error>, TxDriver, BcmBuf) {
    let (result, tx, buf) = xfer.wait();
    (result.map_err(Hub75Error::Dma), tx, buf)
}

// ---------------------------------------------------------------------------
// Interrupt handler (non-circular)
// ---------------------------------------------------------------------------

#[cfg(not(feature = "circular-dma"))]
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

            if let Some(pending) = state.pending_segments.take() {
                state.current_fb_ptr = state.pending_fb_ptr;
                state.pending_fb_ptr = core::ptr::null();
                buf.update_segments(pending);
                signal_swap_done(cs);
            }
        }

        #[cfg(hub75_use_lcd_cam)]
        let xfer_result = start_transfer(tx, buf, state.word_size);
        #[cfg(not(hub75_use_lcd_cam))]
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
// Frame-boundary ISR (circular-dma)
// ---------------------------------------------------------------------------

#[cfg(feature = "circular-dma")]
#[handler]
#[cfg_attr(feature = "iram", ram)]
pub(crate) fn hub75_frame_count_isr() {
    critical_section::with(|cs| {
        if let Some(clear) = CLEAR_INTERRUPT.borrow_ref(cs).as_ref() {
            (clear)();
        }
        FRAME_COUNT.fetch_add(1, Ordering::Relaxed);
        signal_swap_done(cs);
    });
}

// ---------------------------------------------------------------------------
// ISR state initialisation (called by platform constructors)
// ---------------------------------------------------------------------------

#[cfg(all(not(hub75_use_lcd_cam), not(feature = "circular-dma")))]
pub(crate) fn init_isr_state(tx: TxDriver, buf: BcmBuf) {
    critical_section::with(|cs| {
        *ISR_STATE.borrow_ref_mut(cs) = Some(IsrState {
            transfer: TransferPhase::Idle(tx, buf),
            current_fb_ptr: core::ptr::null(),
            pending_segments: None,
            pending_fb_ptr: core::ptr::null(),
        });
    });
}

#[cfg(all(hub75_use_lcd_cam, not(feature = "circular-dma")))]
pub(crate) fn init_isr_state(tx: TxDriver, buf: BcmBuf, word_size: WordSize) {
    critical_section::with(|cs| {
        *ISR_STATE.borrow_ref_mut(cs) = Some(IsrState {
            transfer: TransferPhase::Idle(tx, buf),
            word_size,
            current_fb_ptr: core::ptr::null(),
            pending_segments: None,
            pending_fb_ptr: core::ptr::null(),
        });
    });
}

// ---------------------------------------------------------------------------
// Circular-DMA state storage (called by platform constructors after starting
// DMA)
// ---------------------------------------------------------------------------

#[cfg(feature = "circular-dma")]
pub(crate) fn store_circular_state(
    xfer: TxTransfer,
    desc_ptr: *mut esp_hal::dma::DmaDescriptor,
    desc_count: usize,
    fb_ptr: *const (),
) {
    critical_section::with(|cs| {
        *CIRCULAR_STATE.borrow_ref_mut(cs) = Some(CircularState {
            _transfer: Some(xfer),
            descriptors: desc_ptr,
            desc_count,
            current_fb_ptr: fb_ptr,
        });
    });
}

#[cfg(feature = "circular-dma")]
pub(crate) fn store_clear_interrupt(clear: fn()) {
    critical_section::with(|cs| {
        *CLEAR_INTERRUPT.borrow_ref_mut(cs) = Some(clear);
    });
}

// ---------------------------------------------------------------------------
// Public driver handle
// ---------------------------------------------------------------------------

/// HUB75 display controller driven by an interrupt-based BCM refresh loop.
///
/// Created via [`Hub75::new`] (blocking) or [`Hub75::new_async`] (async).
/// The constructor configures the peripheral, applies pin assignments, and
/// immediately starts DMA-driven display refresh with the provided
/// framebuffer.
///
/// The pin configuration's [`Hub75Pins::Word`](crate::Hub75Pins) type must
/// match the framebuffer's
/// [`FrameBuffer::Word`](crate::framebuffer::FrameBuffer::Word) — mismatches
/// are caught at compile time.
///
/// # Type Parameters
///
/// * `DM` — Driver mode ([`Blocking`](esp_hal::Blocking) or
///   [`Async`](esp_hal::Async)).
/// * `FB` — The concrete framebuffer type.
///
/// # Buffer Swapping
///
/// Call [`swap()`](Hub75::swap) to exchange framebuffers. It returns a
/// [`Hub75Swap`] transfer object that can be waited on:
/// - [`Hub75Swap::wait()`] — spin-loops until the DMA is guaranteed to no
///   longer read from the old buffer, then returns it.
/// - [`Hub75Swap::wait_for_done()`] — yields to the executor (async contexts).
///   Call [`Hub75Swap::wait()`] afterwards to get the result.
/// - [`Hub75Swap::is_done()`] — non-blocking completion check.
///
/// # Limitations
///
/// Only **one** `Hub75` instance may exist at a time. The driver uses
/// module-level statics for the ISR state machine, so creating a second
/// instance would overwrite the first.
///
/// `Hub75` does not implement [`Drop`]. The ISR-driven display refresh is
/// designed to run for the lifetime of the program.
pub struct Hub75<DM: esp_hal::DriverMode, FB> {
    _dm: core::marker::PhantomData<DM>,
    _fb: core::marker::PhantomData<fn() -> FB>,
    _not_sync: core::marker::PhantomData<*const ()>,
}

// SAFETY: Hub75 is a zero-sized handle; all mutable state lives in statics
// guarded by critical_section, so it is safe to send across threads.
// Hub75 is intentionally `!Sync` because concurrent `swap()` calls from
// multiple threads would race on the shared ISR state.
unsafe impl<DM: esp_hal::DriverMode, FB> Send for Hub75<DM, FB> {}

impl<DM: esp_hal::DriverMode, FB> Hub75<DM, FB> {
    pub(crate) fn from_phantom() -> Self {
        Self {
            _dm: core::marker::PhantomData,
            _fb: core::marker::PhantomData,
            _not_sync: core::marker::PhantomData,
        }
    }
}

impl<DM: esp_hal::DriverMode, FB: FrameBuffer + 'static> Hub75<DM, FB> {
    /// Restart display refresh after an error.
    ///
    /// Callable after [`Hub75::swap`] returned an error. Sets up the BCM
    /// segment data and kicks off the first DMA transfer with the same
    /// framebuffer type.
    ///
    /// This method is only available when `circular-dma` is **not** enabled.
    /// In circular-DMA mode the DMA engine never stops, so there is no
    /// restart path.
    ///
    /// # Panics
    ///
    /// Panics if called while a transfer is already in flight.
    #[cfg(not(feature = "circular-dma"))]
    pub fn restart(&self, fb: &'static FB) -> Result<(), Hub75Error> {
        start_internal(fb)
    }

    /// Returns the number of complete BCM frames rendered since driver
    /// creation.
    pub fn frame_count(&self) -> u32 {
        FRAME_COUNT.load(Ordering::Relaxed)
    }
}

#[cfg(not(feature = "circular-dma"))]
pub(crate) fn start_internal(fb: &'static impl FrameBuffer) -> Result<(), Hub75Error> {
    let cache = segments_from_fb(fb);

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

        buf.reset_with_segments(cache);
        state.current_fb_ptr = fb as *const _ as *const ();
        state.pending_segments = None;
        state.pending_fb_ptr = core::ptr::null();

        #[cfg(hub75_use_lcd_cam)]
        let xfer_result = start_transfer(tx, buf, state.word_size);
        #[cfg(not(hub75_use_lcd_cam))]
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

// ---------------------------------------------------------------------------
// Hub75Swap — transfer object returned by swap()
// ---------------------------------------------------------------------------

/// A pending framebuffer swap.
///
/// Returned by [`Hub75::swap`]. The old framebuffer is not safe to reuse until
/// the DMA is guaranteed to no longer be reading from it. Call
/// [`wait_for_done()`](Self::wait_for_done) (async) to yield until safe, then
/// [`wait()`](Self::wait) to obtain the old framebuffer. Or call `wait()`
/// directly for a blocking spin-loop.
///
/// In non-circular mode, "safe" means the ISR has hit a frame boundary and
/// completed the swap. In circular-DMA mode, "safe" means at least one
/// `suc_eof` interrupt has fired after the pointer update, guaranteeing the
/// DMA has completed a full pass and is reading exclusively from the new
/// buffer.
pub struct Hub75Swap<FB: 'static> {
    old_fb_ptr: *mut FB,
    #[cfg(not(feature = "circular-dma"))]
    new_fb_ptr: *mut FB,
}

// SAFETY: The raw pointer always originates from a `&'static mut FB`. Only
// one `Hub75Swap` exists at a time (enforced by the single-instance Hub75
// constraint and the fact that `swap()` takes `&self` on a `!Sync` type).
unsafe impl<FB: 'static> Send for Hub75Swap<FB> {}

impl<FB: FrameBuffer + 'static> Hub75Swap<FB> {
    /// Check whether the swap is complete without blocking.
    ///
    /// Returns `true` once the DMA is guaranteed to no longer be reading
    /// from the old framebuffer.
    pub fn is_done(&self) -> bool {
        #[cfg(not(feature = "circular-dma"))]
        {
            SWAP_DONE.load(Ordering::Acquire) || HAS_ERROR.load(Ordering::Acquire)
        }
        #[cfg(feature = "circular-dma")]
        {
            SWAP_DONE.load(Ordering::Acquire)
        }
    }

    /// Spin-loops until the DMA is guaranteed to no longer be reading from
    /// the old framebuffer, then returns it for reuse.
    ///
    /// In non-circular mode this waits for the next frame boundary. In
    /// circular-DMA mode this waits for one complete frame to be rendered
    /// after the pointer update.
    ///
    /// If [`wait_for_done()`](Self::wait_for_done) was already awaited, this
    /// returns immediately.
    #[cfg(not(feature = "circular-dma"))]
    pub fn wait(self) -> Result<&'static mut FB, (Hub75Error, &'static mut FB)> {
        loop {
            if HAS_ERROR.load(Ordering::Acquire) {
                return critical_section::with(|cs| {
                    let mut borrow = ISR_STATE.borrow_ref_mut(cs);
                    let state = borrow.as_mut().unwrap();
                    state.pending_segments = None;
                    state.pending_fb_ptr = core::ptr::null();
                    let err = match &state.transfer {
                        TransferPhase::Error(err, _, _) => *err,
                        _ => Hub75Error::Dma(esp_hal::dma::DmaError::DescriptorError),
                    };
                    Err((err, unsafe { &mut *self.new_fb_ptr }))
                });
            }
            if SWAP_DONE.load(Ordering::Acquire) {
                break;
            }
            core::hint::spin_loop();
        }
        Ok(unsafe { &mut *self.old_fb_ptr })
    }

    /// Spin-loops until the DMA is guaranteed to no longer be reading from
    /// the old framebuffer, then returns it for reuse.
    ///
    /// If [`wait_for_done()`](Self::wait_for_done) was already awaited, this
    /// returns immediately.
    #[cfg(feature = "circular-dma")]
    pub fn wait(self) -> Result<&'static mut FB, (Hub75Error, &'static mut FB)> {
        while !SWAP_DONE.load(Ordering::Acquire) {
            core::hint::spin_loop();
        }
        Ok(unsafe { &mut *self.old_fb_ptr })
    }

    /// Yields to the executor until the swap is complete.
    ///
    /// After this resolves, call [`wait()`](Self::wait) to obtain the old
    /// framebuffer. This mirrors the esp-hal transfer pattern:
    ///
    /// ```rust,ignore
    /// let mut xfer = hub75.swap(fb)?;
    /// xfer.wait_for_done().await;
    /// fb = xfer.wait()?;
    /// ```
    #[cfg(not(feature = "circular-dma"))]
    pub async fn wait_for_done(&mut self) {
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
    }

    /// Yields to the executor until the swap is complete.
    ///
    /// After this resolves, call [`wait()`](Self::wait) to obtain the old
    /// framebuffer.
    #[cfg(feature = "circular-dma")]
    pub async fn wait_for_done(&mut self) {
        core::future::poll_fn(|cx| {
            if SWAP_DONE.load(Ordering::Acquire) {
                return core::task::Poll::Ready(());
            }
            critical_section::with(|cs| {
                if SWAP_DONE.load(Ordering::Relaxed) {
                    return core::task::Poll::Ready(());
                }
                *SWAP_WAKER.borrow_ref_mut(cs) = Some(cx.waker().clone());
                core::task::Poll::Pending
            })
        })
        .await;
    }
}

// ---------------------------------------------------------------------------
// Swap (non-circular)
// ---------------------------------------------------------------------------

#[cfg(not(feature = "circular-dma"))]
impl<DM: esp_hal::DriverMode, FB: FrameBuffer + 'static> Hub75<DM, FB> {
    /// Initiate a framebuffer swap.
    ///
    /// Registers `new_fb` as the pending buffer. The actual swap occurs at
    /// the next frame boundary (handled by the ISR). Returns a [`Hub75Swap`]
    /// transfer object — call [`.wait_for_done()`](Hub75Swap::wait_for_done)
    /// then [`.wait()`](Hub75Swap::wait), or just `.wait()` directly for
    /// blocking.
    ///
    /// # Panics
    ///
    /// Panics if the driver has not been initialised (no `Hub75` instance
    /// was created).
    pub fn swap(&self, new_fb: &'static mut FB) -> Hub75Swap<FB> {
        let new_cache = segments_from_fb(new_fb);
        let new_fb_ptr = new_fb as *mut FB;

        let old_fb_ptr = critical_section::with(|cs| {
            let mut borrow = ISR_STATE.borrow_ref_mut(cs);
            let state = borrow.as_mut().expect("Hub75 not initialised");
            let old = state.current_fb_ptr;
            state.pending_segments = Some(new_cache);
            state.pending_fb_ptr = new_fb_ptr as *const ();
            SWAP_DONE.store(false, Ordering::Relaxed);
            old
        });

        Hub75Swap {
            old_fb_ptr: old_fb_ptr as *mut FB,
            new_fb_ptr,
        }
    }
}

// ---------------------------------------------------------------------------
// Swap (circular-dma)
// ---------------------------------------------------------------------------

#[cfg(feature = "circular-dma")]
impl<DM: esp_hal::DriverMode, FB: FrameBuffer + 'static> Hub75<DM, FB> {
    /// Initiate a framebuffer swap (circular-DMA mode).
    ///
    /// Updates all DMA descriptor buffer pointers immediately and returns a
    /// [`Hub75Swap`] transfer object. The DMA engine's internal register may
    /// still be pointing into the old buffer for the currently in-flight
    /// descriptor. Call [`.wait_for_done()`](Hub75Swap::wait_for_done) then
    /// [`.wait()`](Hub75Swap::wait), or just `.wait()` directly for blocking.
    ///
    /// # Panics
    ///
    /// Panics if the driver has not been initialised (no `Hub75` instance
    /// was created).
    pub fn swap(&self, new_fb: &'static mut FB) -> Hub75Swap<FB> {
        let new_fb_ptr = new_fb as *mut FB;
        let old_fb_ptr = critical_section::with(|cs| {
            let mut borrow = CIRCULAR_STATE.borrow_ref_mut(cs);
            let state = borrow.as_mut().expect("Hub75 not initialised");
            let delta = new_fb_ptr as isize - state.current_fb_ptr as isize;
            // SAFETY: `descriptors` points to a `&'static mut` descriptor
            // array that outlives everything. The DMA engine reads descriptor
            // fields concurrently while we update `buffer` pointers here.
            // This is safe because:
            //  1. Each `buffer` field is a naturally-aligned 32-bit pointer. On Xtensa
            //     (ESP32/S3) aligned 32-bit stores are atomic with respect to the DMA bus
            //     master, so the DMA never observes a half-written pointer value.
            //  2. `circular-dma` is blocked at compile time on RISC-V targets (ESP32-C5/C6)
            //     where PARL_IO does not support circular chains.
            //  3. The pointer delta is valid because all plane data resides within a single
            //     contiguous `FB` allocation — both old and new framebuffers have identical
            //     internal layout.
            //  4. The worst-case visual artifact is one partially-mixed frame (tearing),
            //     which is acceptable for a display use-case.
            unsafe {
                for i in 0..state.desc_count {
                    let desc = &mut *state.descriptors.add(i);
                    desc.buffer = desc.buffer.wrapping_byte_offset(delta);
                }
            }
            let old_ptr = state.current_fb_ptr;
            state.current_fb_ptr = new_fb_ptr as *const ();
            SWAP_DONE.store(false, Ordering::Release);
            old_ptr
        });
        Hub75Swap {
            old_fb_ptr: old_fb_ptr as *mut FB,
        }
    }
}
