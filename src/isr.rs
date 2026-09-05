//! Shared ISR-driven BCM refresh infrastructure.
//!
//! Compiled for every platform that uses an interrupt-driven refresh loop
//! (ESP32 via I2S Parallel, ESP32-S3 via `LCD_CAM`, ESP32-C5/C6 via
//! `PARL_IO`). Holds the transfer state machine, the interrupt handler, and
//! the public [`Hub75`] driver handle with its `swap()` API.

use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;
use core::task::Waker;

use esp_hal::Blocking;
#[cfg(all(feature = "circular-dma", hub75_use_lcd_cam))]
use esp_hal::dma::DmaTxInterrupt;
use esp_hal::handler;
use esp_hal::interrupt::InterruptHandler;
use esp_hal::interrupt::Priority;
#[cfg(feature = "iram")]
use esp_hal::ram;
use esp_sync::NonReentrantMutex;

use crate::Hub75Error;
#[cfg(not(feature = "circular-dma"))]
use crate::bcm::cache_ptr;
#[cfg(feature = "circular-dma")]
use crate::bcm::circular::CircularBcmBuf;
#[cfg(not(feature = "circular-dma"))]
use crate::bcm::linear::BcmBuf;
#[cfg(not(feature = "circular-dma"))]
use crate::bcm::segments_from_fb_into;
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
// Frame-boundary interrupt handling (circular-dma only)
// ---------------------------------------------------------------------------
//
// The circular descriptor chain carries no `suc_eof` in steady state (a
// `suc_eof` descriptor would halt the DMA channel on ESP32-C5). The boundary
// detector is *armed* by setting `suc_eof` on the last descriptor and
// enabling the backend's frame-boundary interrupt, and *disarmed* again by
// the ISR. This is done only around a swap; in steady state no interrupts
// are enabled on any backend.

/// Per-backend frame-boundary interrupt management, through the transfer
/// kept alive in [`CircularState`], so no `DMA::steal()` or stored closure is
/// needed on any backend.
#[cfg(feature = "circular-dma")]
pub(crate) trait FrameInterrupt {
    /// Clear a pending frame-boundary flag. Called when arming (to drain
    /// stale flags) and from the frame-count ISR.
    fn clear_frame_interrupt(&self);

    /// Enable the frame-boundary source.
    fn listen_frame_interrupt(&self);

    /// Disable the frame-boundary source.
    fn unlisten_frame_interrupt(&self);
}

#[cfg(all(feature = "circular-dma", hub75_use_i2s_parallel))]
impl FrameInterrupt for TxTransfer {
    fn clear_frame_interrupt(&self) {
        use esp_hal::i2s::parallel::I2sParallelInterrupt;
        self.clear_interrupts(I2sParallelInterrupt::Eof);
    }

    fn listen_frame_interrupt(&self) {
        use esp_hal::i2s::parallel::I2sParallelInterrupt;
        self.listen(I2sParallelInterrupt::Eof);
    }

    fn unlisten_frame_interrupt(&self) {
        use esp_hal::i2s::parallel::I2sParallelInterrupt;
        self.unlisten(I2sParallelInterrupt::Eof);
    }
}

#[cfg(all(feature = "circular-dma", hub75_use_lcd_cam))]
impl FrameInterrupt for TxTransfer {
    fn clear_frame_interrupt(&self) {
        self.clear_interrupts_dma(DmaTxInterrupt::Eof);
    }

    fn listen_frame_interrupt(&self) {
        self.listen_dma(DmaTxInterrupt::Eof);
    }

    fn unlisten_frame_interrupt(&self) {
        self.unlisten_dma(DmaTxInterrupt::Eof);
    }
}

/// PARL_IO (ESP32-C5): the boundary source is the peripheral's `TxEof`
/// interrupt, which fires when the GDMA signals `suc_eof`. A consumed
/// `suc_eof` also *halts* the DMA channel, so the PARL_IO ISR must restart
/// the transfer after every boundary.
#[cfg(all(feature = "circular-dma", hub75_use_parl_io))]
impl FrameInterrupt for TxTransfer {
    fn clear_frame_interrupt(&self) {
        use esp_hal::parl_io::ParlIoInterrupt;
        self.clear_interrupts(ParlIoInterrupt::TxEof);
    }

    fn listen_frame_interrupt(&self) {
        use esp_hal::parl_io::ParlIoInterrupt;
        self.listen(ParlIoInterrupt::TxEof);
    }

    fn unlisten_frame_interrupt(&self) {
        use esp_hal::parl_io::ParlIoInterrupt;
        self.unlisten(ParlIoInterrupt::TxEof);
    }
}

// ---------------------------------------------------------------------------
// ISR shared state
// ---------------------------------------------------------------------------

/// A mutex-protected cell, parl_io-style.
///
/// `esp_sync::NonReentrantMutex` — the same `esp_sync::RawMutex` primitive
/// esp-hal's `PARL_IO` driver uses for its interrupt-enable register RMWs
/// (and which backs esp-hal's `critical_section` implementation), fused with
/// the interior-mutable data it guards. Access goes through
/// [`NonReentrantMutex::with`], which hands the closure a `&mut T`; calling
/// it reentrantly panics, turning any accidental lock-nesting bug into a
/// loud crash instead of silent corruption.
///
/// Why a dedicated mutex instead of the process-global `critical_section`:
/// the HUB75 refresh ISR runs at a very high rate (for example 64×32 @ 8
/// planes @ 200 Hz ⇒ ~12.8k ISR/s), and on multi-core chips (ESP32,
/// ESP32-S3) the global critical section is a single lock shared with *every*
/// other critical-section user in the firmware (esp-radio, Embassy, other
/// drivers). Taking the global lock in the ISR would spin behind unrelated
/// work on the other core — and make that work spin behind the ISR —
/// directly adding BCM timing jitter. A dedicated mutex only contends with
/// code that actually touches the ISR state.
///
/// Lock ordering: these mutexes are **leaf** locks. Inside them we only touch
/// the shared state, our peripheral registers, and esp-hal transfer methods
/// (which may acquire esp-hal-internal locks, but never ours), so no lock
/// cycle is possible.
type Shared<T> = NonReentrantMutex<T>;

/// Rebuilds an `InterruptHandler` with a runtime-configured priority.
///
/// `#[handler]` bakes the priority into the emitted const at compile time
/// (defaulting to `Priority::min()`); the handler's entry point is recovered
/// via `InterruptHandler::handler().callback()` and re-wrapped with the
/// requested priority.
pub(crate) fn handler_with_priority(
    handler: InterruptHandler,
    priority: Option<Priority>,
) -> InterruptHandler {
    match priority {
        Some(prio) => InterruptHandler::new(handler.handler().callback(), prio),
        None => handler,
    }
}

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
    /// Byte offset from the current to the pending framebuffer, set by
    /// `swap()`. The ISR applies this delta to every cached segment
    /// pointer at the frame boundary, then clears it to `None`.
    pub(crate) pending_delta: Option<isize>,
    pub(crate) pending_fb_ptr: *const (),
}

#[cfg(not(feature = "circular-dma"))]
// SAFETY (`Send`): required so the `Mutex<RefCell<Option<_>>>` statics below
// are `Sync`. All access to the inner value is serialised by the embassy
// mutex (which disables interrupts and CAS-spins on an owner word on
// multi-core chips). The raw pointers (`current_fb_ptr`, `pending_fb_ptr`)
// are only dereferenced inside lock closures, so they are never accessed
// concurrently from multiple cores.
unsafe impl Send for IsrState {}

#[cfg(not(feature = "circular-dma"))]
type SharedIsrState = Shared<Option<IsrState>>;

#[cfg(not(feature = "circular-dma"))]
static ISR_STATE: SharedIsrState = Shared::new(None);

// ---------------------------------------------------------------------------
// Circular-DMA shared state
// ---------------------------------------------------------------------------

#[cfg(feature = "circular-dma")]
pub(crate) struct CircularState {
    // Kept alive to prevent the DMA from stopping. All backends also use
    // the stored transfer to manage the frame-boundary interrupt (see
    // `FrameInterrupt`).
    pub(crate) transfer: Option<TxTransfer>,
    pub(crate) descriptors: *mut esp_hal::dma::DmaDescriptor,
    pub(crate) desc_count: usize,
    pub(crate) current_fb_ptr: *const (),
    /// Pointer delta for a pending swap, applied by the ISR at the next pass
    /// boundary (see `Hub75::swap`).
    pub(crate) pending_delta: Option<isize>,
    /// Set by `swap()` and cleared by the frame-boundary ISR; prevents
    /// a second `swap()` while one is still in-flight.
    pub(crate) swap_in_flight: bool,
}

#[cfg(feature = "circular-dma")]
// SAFETY (`Send`): same justification as `IsrState` — required so the
// `Mutex<RefCell<Option<_>>>` static is `Sync`; all access is serialised by
// the embassy mutex.
unsafe impl Send for CircularState {}

#[cfg(feature = "circular-dma")]
type SharedCircularState = Shared<Option<CircularState>>;

#[cfg(feature = "circular-dma")]
static CIRCULAR_STATE: SharedCircularState = Shared::new(None);

// ---------------------------------------------------------------------------
// Swap signalling
// ---------------------------------------------------------------------------

/// Swap-completion flag.
///
/// Payload protocol: all state establishing or retiring a swap is written
/// before a `Release` store to this flag, and every consumer reads it with
/// `Acquire`. `true` means the swap has been applied at a pass boundary and
/// it is safe to reclaim the old framebuffer via [`Hub75Swap::wait`].
/// Note (ESP32/S3): the first chunk of the just-started pass may still be
/// clocked out from the old buffer for up to one descriptor transfer time
/// (~200us at 4KiB/10MHz) after this flag is set; reclaim is still sound
/// because the old buffer remains valid, just briefly shared.
static SWAP_DONE: AtomicBool = AtomicBool::new(false);
#[cfg(not(feature = "circular-dma"))]
static HAS_ERROR: AtomicBool = AtomicBool::new(false);
/// Set to true once a Hub75 instance has been created. Prevents a second
/// constructor from overwriting the ISR state statics. Hub75 has no Drop
/// (run-forever design), so this is never reset.
static DRIVER_TAKEN: AtomicBool = AtomicBool::new(false);
static SWAP_WAKER: Shared<Option<Waker>> = Shared::new(None);

fn signal_swap_done() {
    SWAP_DONE.store(true, Ordering::Release);
    // Take the waker under the lock but wake outside of it: `wake()` runs
    // arbitrary executor code that must not run while a mutex is held.
    let waker = SWAP_WAKER.with(Option::take);
    if let Some(waker) = waker {
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
/// than the `PARL_IO` byte counter, so the transfer-length field is unused.
#[cfg(all(hub75_use_parl_io, esp32c5))]
pub(crate) const PARL_IO_DUMMY_TRANSFER_LEN: usize = 0;

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
    ISR_STATE.with(|state| {
        let Some(state) = state.as_mut() else { return };

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
            signal_swap_done();
            return;
        }

        let frame_boundary = buf.advance();

        if frame_boundary && let Some(delta) = state.pending_delta.take() {
            state.current_fb_ptr = state.pending_fb_ptr;
            state.pending_fb_ptr = core::ptr::null();
            buf.apply_delta(delta);
            signal_swap_done();
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
                signal_swap_done();
            }
        }
    });
}

// ---------------------------------------------------------------------------
// Pass-boundary ISR (circular-dma)
// ---------------------------------------------------------------------------

/// Apply a pending framebuffer pointer delta to every descriptor, at a pass
/// boundary. Shared by the per-backend ISRs.
#[cfg(feature = "circular-dma")]
#[cfg_attr(feature = "iram", ram)]
fn apply_pending_delta(state: &mut CircularState, delta: isize) {
    // SAFETY: `descriptors` points to a `&'static mut` descriptor
    // array that outlives everything. The DMA engine may be reading
    // descriptor fields while we rewrite the `buffer` pointers here;
    // that race is fine:
    //  1. Each `buffer` field is a naturally aligned 32-bit pointer; aligned 32-bit
    //     stores are atomic with respect to the DMA bus master, so DMA never sees a
    //     half-written pointer.
    //  2. At a pass boundary the DMA has consumed the whole chain, so the pointers
    //     take effect from the start of the next pass. On chips where `suc_eof`
    //     does not halt the DMA (ESP32/S3), the engine has already fetched
    //     descriptor 0's `buffer` at the wrap, before this ISR runs — plus one
    //     extra chunk per descriptor-transfer-time (`chunk_bytes / bus_hz`, ~200us
    //     at 4KiB/10MHz) of ISR entry latency. Those head chunks of the post-swap
    //     pass are sourced from the old framebuffer. This is a worst-case one-pass
    //     visual artifact confined to the LSB-first head of the BCM sequence; it is
    //     not memory-unsafe. The rewrite loop itself is orders of magnitude faster
    //     than the DMA's per-descriptor advance, so iteration order cannot race the
    //     engine.
    //  3. The delta stays valid because all plane data lives in one contiguous `FB`
    //     allocation and old and new framebuffers have identical layout.
    unsafe {
        for i in 0..state.desc_count {
            let desc = &mut *state.descriptors.add(i);
            desc.buffer = desc.buffer.wrapping_byte_offset(delta);
        }
    }
}

/// Swap-boundary ISR (I2S / `LCD_CAM`, circular-dma).
///
/// The boundary detector is armed only around a swap (see [`Hub75::swap`]);
/// this handler applies the pending buffer delta at the pass boundary and
/// disarms the detector again, leaving the chain free-running with no
/// interrupts enabled in steady state.
#[cfg(all(
    feature = "circular-dma",
    any(hub75_use_lcd_cam, hub75_use_i2s_parallel)
))]
#[handler]
#[cfg_attr(feature = "iram", ram)]
pub(crate) fn hub75_boundary_isr() {
    CIRCULAR_STATE.with(|state| {
        let Some(state) = state.as_mut() else {
            return;
        };

        // Scope the transfer borrow so the pending delta can be applied to
        // the state below.
        {
            let Some(xfer) = state.transfer.as_ref() else {
                return;
            };
            xfer.clear_frame_interrupt();
        }

        if let Some(delta) = state.pending_delta.take() {
            apply_pending_delta(state, delta);
            state.swap_in_flight = false;
            signal_swap_done();
        }

        // Swap-armed boundary handled: disarm until the next swap. The
        // DMA does not halt on `suc_eof` on these chips, so the chain
        // keeps running.
        crate::bcm::circular::set_last_suc_eof(state.descriptors, state.desc_count, false);
        if let Some(xfer) = state.transfer.as_ref() {
            xfer.unlisten_frame_interrupt();
        }
    });
}

/// Swap-boundary ISR (PARL_IO / ESP32-C5, circular-dma).
///
/// A consumed `suc_eof` halts the DMA channel, so the ISR must restart the
/// transfer after applying the pending delta. The detector is armed only
/// around a swap (see [`Hub75::swap`]); in steady state the chain runs
/// `suc_eof`-free and no interrupts are enabled.
#[cfg(all(feature = "circular-dma", hub75_use_parl_io))]
#[handler]
#[cfg_attr(feature = "iram", ram)]
pub(crate) fn hub75_boundary_isr() {
    CIRCULAR_STATE.with(|state| {
        let Some(state) = state.as_mut() else {
            return;
        };
        // Take the transfer: `wait()` consumes it and returns the driver and
        // buffer for the restart. `wait()` returns instantly — the boundary
        // interrupt has already fired — and clears the `tx_eof` flag.
        let Some(xfer) = state.transfer.take() else {
            return;
        };

        if let Some(delta) = state.pending_delta.take() {
            apply_pending_delta(state, delta);
            state.swap_in_flight = false;
            signal_swap_done();
        }

        // Swap-armed boundary handled: disarm until the next swap; the chain
        // runs suc_eof-free again until the next swap arms it.
        crate::bcm::circular::set_last_suc_eof(state.descriptors, state.desc_count, false);
        xfer.unlisten_frame_interrupt();

        let (_, tx, buf) = xfer.wait();
        // Dummy transfer length (`tx_bytelen = 0`): with `TxEofSource::DmaEof`
        // the frame ends at the armed `suc_eof` descriptor regardless of the
        // bit-length counter, so there is no size limit.
        match tx.write(PARL_IO_DUMMY_TRANSFER_LEN, buf) {
            Ok(xfer) => state.transfer = Some(xfer),
            // The driver handle and buffer are dropped; the display stops.
            Err((_, _tx, _buf)) => {}
        }
    });
}

// ---------------------------------------------------------------------------
// ISR state initialisation (called by platform constructors)
// ---------------------------------------------------------------------------

#[cfg(all(not(hub75_use_lcd_cam), not(feature = "circular-dma")))]
pub(crate) fn init_isr_state(tx: TxDriver, buf: BcmBuf) {
    ISR_STATE.with(|state| {
        *state = Some(IsrState {
            transfer: TransferPhase::Idle(tx, buf),
            current_fb_ptr: core::ptr::null(),
            pending_delta: None,
            pending_fb_ptr: core::ptr::null(),
        });
    });
}

#[cfg(all(hub75_use_lcd_cam, not(feature = "circular-dma")))]
pub(crate) fn init_isr_state(tx: TxDriver, buf: BcmBuf, word_size: WordSize) {
    ISR_STATE.with(|state| {
        *state = Some(IsrState {
            transfer: TransferPhase::Idle(tx, buf),
            word_size,
            current_fb_ptr: core::ptr::null(),
            pending_delta: None,
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
    CIRCULAR_STATE.with(|state| {
        *state = Some(CircularState {
            transfer: Some(xfer),
            descriptors: desc_ptr,
            desc_count,
            current_fb_ptr: fb_ptr,
            pending_delta: None,
            swap_in_flight: false,
        });

        // The transfer is stored; from this point on every interrupt can be
        // serviced through it. Drain any stale frame-boundary flag. The
        // detector is armed by `swap()` and disarmed by the ISR; no
        // interrupts are enabled in steady state.
        {
            let state = state.as_mut().expect("just stored");
            let transfer = state.transfer.as_ref().expect("transfer kept alive");
            transfer.clear_frame_interrupt();
        }
    });
}

/// Attempt to claim the singleton driver slot.
///
/// Returns `Ok(())` if this is the first initialisation, or
/// `Err(Hub75Error::AlreadyInitialised)` if a driver already exists.
/// Called at the top of every constructor's `new_internal` before any
/// hardware configuration so that a second call fails cleanly without
/// overwriting static state.
pub(crate) fn claim_driver() -> Result<(), Hub75Error> {
    if DRIVER_TAKEN.swap(true, Ordering::AcqRel) {
        Err(Hub75Error::AlreadyInitialised)
    } else {
        Ok(())
    }
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
/// [`FrameBuffer::Word`](crate::framebuffer::FrameBuffer::Word); mismatches
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
/// **Framebuffer data must reside in internal DRAM, not PSRAM.** PSRAM
/// needs cache writeback before DMA reads, and this driver's custom DMA
/// buffer paths don't do that. A debug assertion checks this at init.
///
/// `Hub75` does not implement [`Drop`]. The ISR-driven refresh runs for the
/// lifetime of the program.
pub struct Hub75<DM: esp_hal::DriverMode, FB> {
    _dm: core::marker::PhantomData<DM>,
    _fb: core::marker::PhantomData<fn() -> FB>,
    _not_sync: core::marker::PhantomData<*const ()>,
}

// SAFETY: Hub75 is a zero-sized handle; all mutable state lives in statics
// guarded by the ISR `STATE_LOCK` (see `Shared`), so it is safe to send
// across threads.
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
    /// # Errors
    ///
    /// Returns [`Hub75Error::AlreadyRunning`](crate::Hub75Error::AlreadyRunning)
    /// if called while a transfer is already in flight. Call
    /// [`Hub75Swap::wait`](crate::Hub75Swap::wait) on the outstanding swap
    /// first.
    #[cfg(not(feature = "circular-dma"))]
    pub fn restart(&self, fb: &'static FB) -> Result<(), Hub75Error> {
        start_internal(fb)
    }
}

#[cfg(not(feature = "circular-dma"))]
pub(crate) fn start_internal(fb: &'static impl FrameBuffer) -> Result<(), Hub75Error> {
    crate::bcm::validate_fb_internal_ram(fb);

    ISR_STATE.with(|state| {
        let state = state.as_mut().expect("Hub75 not initialised");

        let (tx, mut buf) =
            match core::mem::replace(&mut state.transfer, TransferPhase::Transitioning) {
                TransferPhase::Idle(tx, buf) | TransferPhase::Error(_, tx, buf) => (tx, buf),
                other => {
                    state.transfer = other;
                    return Err(Hub75Error::AlreadyRunning);
                }
            };

        // Resolve any stale Hub75Swap that was waiting on a previous error
        // state before it was consumed. Without this, a swap left un-waited
        // when restart() is called would spin forever because HAS_ERROR and
        // SWAP_DONE get cleared below, and no new pending_delta exists to
        // drive a fresh completion signal.
        signal_swap_done();

        // Build the cache in-place (restart/re-init path).
        segments_from_fb_into(fb, unsafe { &mut *cache_ptr().cast_mut() });
        buf.reset_with_cache();
        state.current_fb_ptr = core::ptr::from_ref(fb).cast::<()>();
        state.pending_delta = None;
        state.pending_fb_ptr = core::ptr::null();

        #[cfg(hub75_use_lcd_cam)]
        let xfer_result = start_transfer(tx, buf, state.word_size);
        #[cfg(not(hub75_use_lcd_cam))]
        let xfer_result = start_transfer(tx, buf);

        match xfer_result {
            Ok(xfer) => {
                state.transfer = TransferPhase::InFlight(xfer);
                HAS_ERROR.store(false, Ordering::Release);
                SWAP_DONE.store(false, Ordering::Release);
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
#[must_use = "call .wait() to reclaim the old framebuffer, or the buffer is leaked"]
pub struct Hub75Swap<FB: 'static> {
    old_fb_ptr: *mut FB,
    #[cfg(not(feature = "circular-dma"))]
    new_fb_ptr: *mut FB,
}

// SAFETY: The raw pointer always originates from a `&'static mut FB`. Only
// one `Hub75Swap` exists at a time: `Hub75::swap()` returns
// `Err(Hub75Error::SwapInFlight, _)` if called while a previous swap is still
// in-flight, and `Hub75` is `!Sync`, so concurrent `swap()` calls from
// multiple threads are impossible.
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
    ///
    /// # Errors
    ///
    /// Returns [`Hub75Error::Dma`] if the ISR has recorded a DMA error from
    /// a prior transfer.
    ///
    /// # Panics
    ///
    /// Panics if the driver has not been initialised (no `Hub75` instance
    /// was created).
    #[cfg(not(feature = "circular-dma"))]
    pub fn wait(self) -> Result<&'static mut FB, (Hub75Error, &'static mut FB)> {
        loop {
            if HAS_ERROR.load(Ordering::Acquire) {
                return ISR_STATE.with(|state| {
                    let state = state.as_mut().unwrap();
                    state.pending_delta = None;
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
    ///
    /// # Errors
    ///
    /// The error variant is never returned in circular-DMA mode; it exists
    /// for API consistency with the non-circular [`wait()`](Self::wait).
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
            SWAP_WAKER.with(|waker| {
                if SWAP_DONE.load(Ordering::Acquire) || HAS_ERROR.load(Ordering::Acquire) {
                    return core::task::Poll::Ready(());
                }
                *waker = Some(cx.waker().clone());
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
            SWAP_WAKER.with(|waker| {
                if SWAP_DONE.load(Ordering::Acquire) {
                    return core::task::Poll::Ready(());
                }
                *waker = Some(cx.waker().clone());
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
    /// transfer object; call [`.wait_for_done()`](Hub75Swap::wait_for_done)
    /// then [`.wait()`](Hub75Swap::wait), or just `.wait()` directly for
    /// blocking.
    ///
    /// # Errors
    ///
    /// Returns [`Hub75Error::SwapInFlight`] along with ownership of `new_fb`
    /// if a previous [`Hub75Swap`] is still outstanding. Only one swap may be
    /// in-flight at a time; call `.wait()` (or `.wait_for_done().await` then
    /// `.wait()`) on the previous [`Hub75Swap`] before calling `swap()` again.
    ///
    /// # Panics
    ///
    /// Panics if the driver has not been initialised (no `Hub75` instance
    /// was created).
    pub fn swap(
        &self,
        new_fb: &'static mut FB,
    ) -> Result<Hub75Swap<FB>, (Hub75Error, &'static mut FB)> {
        // Pre-validate the framebuffer contract so any panic happens outside
        // the critical section (with interrupts enabled).
        let count = new_fb.bcm_segment_count();
        let spg = new_fb.bcm_segments_per_group();
        assert!(
            count <= crate::bcm::MAX_SEGMENTS,
            "bcm_segment_count {count} exceeds MAX_SEGMENTS"
        );
        assert!(
            spg > 0 && count.is_multiple_of(spg),
            "bcm_segment_count {count} not divisible by segments_per_group {spg}"
        );
        let new_fb_ptr = core::ptr::from_mut::<FB>(new_fb);

        let old_fb_ptr = ISR_STATE.with(|state| {
            let state = state.as_mut().expect("Hub75 not initialised");
            if state.pending_delta.is_some() {
                return Err(new_fb_ptr as *const ());
            }
            let old = state.current_fb_ptr;
            // Both framebuffers are the same type with identical internal
            // layout; this is the same invariant circular-DMA mode relies
            // on. The delta shifts every cached segment pointer to the new
            // FB.
            let delta = new_fb_ptr as isize - old as isize;
            state.pending_delta = Some(delta);
            state.pending_fb_ptr = new_fb_ptr as *const ();
            SWAP_DONE.store(false, Ordering::Release);
            Ok(old)
        });

        match old_fb_ptr {
            Ok(old) => Ok(Hub75Swap {
                old_fb_ptr: old as *mut FB,
                new_fb_ptr,
            }),
            Err(_) => Err((Hub75Error::SwapInFlight, new_fb)),
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
    /// # Swap granularity
    ///
    /// On ESP32-C5 (`PARL_IO`) the DMA halts at the boundary and the switch
    /// is exact. On ESP32/ESP32-S3 the DMA wraps without halting, so the
    /// first DMA chunk (~4KiB, the LSB end of the BCM sequence) of the pass
    /// in which the swap completes may still come from the previous
    /// framebuffer. The affected slice carries at most a few percent of
    /// frame brightness and the display is transitioning to new content
    /// anyway, so this is not visible in practice.
    ///
    /// # Errors
    ///
    /// Returns [`Hub75Error::SwapInFlight`] along with ownership of `new_fb`
    /// if a previous [`Hub75Swap`] is still outstanding. Only one swap may be
    /// in-flight at a time; call `.wait()` (or `.wait_for_done().await` then
    /// `.wait()`) on the previous [`Hub75Swap`] before calling `swap()` again.
    ///
    /// # Panics
    ///
    /// Panics if the driver has not been initialised (no `Hub75` instance
    /// was created).
    pub fn swap(
        &self,
        new_fb: &'static mut FB,
    ) -> Result<Hub75Swap<FB>, (Hub75Error, &'static mut FB)> {
        let new_fb_ptr = core::ptr::from_mut::<FB>(new_fb);
        let old_fb_ptr = CIRCULAR_STATE.with(|state| {
            let state = state.as_mut().expect("Hub75 not initialised");
            if state.swap_in_flight {
                return Err(new_fb_ptr as *const ());
            }
            let delta = new_fb_ptr as isize - state.current_fb_ptr as isize;
            // The delta is applied by the frame-boundary ISR at the next pass
            // boundary. On ESP32/S3 the wrap-time fetch of descriptor 0 (plus
            // ISR latency) means the first chunk(s) of that pass still source
            // the old framebuffer — a benign, LSB-weighted, one-pass artifact
            // (see `apply_pending_delta`).
            state.pending_delta = Some(delta);
            state.swap_in_flight = true;
            // Arm the boundary detector: mark the last descriptor with
            // `suc_eof` and enable the backend's frame-boundary interrupt.
            //
            // On ESP32/S3 the DMA wraps without halting, so the bit is
            // purely a marker for the next pass boundary; the ISR clears it
            // and disarms. On ESP32-C5 the consumed `suc_eof` *halts* the
            // DMA channel; the ISR restarts the transfer (see
            // `hub75_boundary_isr`).
            crate::bcm::circular::set_last_suc_eof(state.descriptors, state.desc_count, true);
            if let Some(xfer) = state.transfer.as_ref() {
                // Drain any frame-boundary flag latched before the update,
                // so the ISR cannot attribute a pre-swap boundary to the new
                // buffer and release the old framebuffer while the DMA may
                // still be reading from it.
                xfer.clear_frame_interrupt();
                xfer.listen_frame_interrupt();
            }
            let old_ptr = state.current_fb_ptr;
            state.current_fb_ptr = new_fb_ptr as *const ();
            SWAP_DONE.store(false, Ordering::Release);
            Ok(old_ptr)
        });

        match old_fb_ptr {
            Ok(old) => Ok(Hub75Swap {
                old_fb_ptr: old as *mut FB,
            }),
            Err(_) => Err((Hub75Error::SwapInFlight, new_fb)),
        }
    }
}
