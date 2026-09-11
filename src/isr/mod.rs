//! Shared ISR-driven BCM refresh infrastructure.
//!
//! Compiled for every platform that uses an interrupt-driven refresh loop
//! (ESP32 via I2S Parallel, ESP32-S3 via `LCD_CAM`, ESP32-C5/C6 via
//! `PARL_IO`). This module holds the pieces shared by both refresh modes —
//! the swap-signalling core and the public [`Hub75`]/[`Hub75Swap`] API —
//! while the two mutually-exclusive refresh state machines live in
//! submodules:
//!
//! - [`linear`]: one DMA transfer per BCM segment group, restarted by the ISR
//!   (compiled when `circular-dma` is **not** enabled).
//! - [`circular`]: a single free-running circular descriptor chain with
//!   pointer-delta swaps at pass boundaries (compiled with `circular-dma`).
//!
//! Both modes share a single unified refresh [`isr`]: the mode differences
//! (stale-interrupt gating, group advancement, delta target, disarm, and
//! completion signalling) are compile-time selected inside the handler.

use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;
use core::task::Waker;

use esp_hal::Blocking;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::handler;
use esp_hal::interrupt::InterruptHandler;
use esp_hal::interrupt::Priority;
#[cfg(feature = "iram")]
use esp_hal::ram;
use esp_sync::NonReentrantMutex;

use crate::Hub75Error;
use crate::framebuffer::FrameBuffer;
#[cfg(hub75_use_lcd_cam)]
use crate::framebuffer::WordSize;

// Compile in exactly one refresh mode, and re-export its items so callers
// (`i2s_parallel.rs`, `lcd_cam.rs`, `parl_io.rs`) can refer to them uniformly
// as `isr::{isr, init_state, start_internal, ...}` regardless of the active
// refresh mode. `start_internal` itself is shared (defined below), so it is
// not re-exported per mode.
cfg_select! {
    feature = "circular-dma" => {
        pub(crate) mod circular;
        pub(crate) use circular::init_state;
        pub(crate) use crate::bcm::circular::BcmBuf;
    }
    _ => {
        pub(crate) mod linear;
        pub(crate) use linear::init_state;
        pub(crate) use crate::bcm::linear::BcmBuf;
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
pub(crate) type Shared<T> = NonReentrantMutex<T>;

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

// ---------------------------------------------------------------------------
// Swap signalling
// ---------------------------------------------------------------------------

/// Swap-completion flag.
///
/// Payload protocol: all state establishing or retiring a swap is written
/// before a `Release` store to this flag, and every consumer reads it with
/// `Acquire`. `true` means the swap has been applied at a pass boundary and
/// it is safe to reclaim the old framebuffer via [`Hub75Swap::wait`].
///
/// Reclaim-safety note: once this flag is set the transfer that reached the
/// boundary has fully finished (the ISR consumed it via [`finish_transfer`],
/// whose `wait()` confirms the peripheral is idle), the pointer delta has been
/// applied, and the DMA has been restarted sourcing the new framebuffer — so
/// it will never read the old buffer again. Reclaiming the old buffer via
/// [`Hub75Swap::wait`] is therefore sound.
pub(crate) static SWAP_DONE: AtomicBool = AtomicBool::new(false);
/// Set to true once a Hub75 instance has been created. Prevents a second
/// constructor from overwriting the ISR state statics. Hub75 has no Drop
/// (run-forever design), so this is never reset.
static DRIVER_TAKEN: AtomicBool = AtomicBool::new(false);
pub(crate) static SWAP_WAKER: Shared<Option<Waker>> = Shared::new(None);

/// DMA error flag, set by the ISR (either refresh mode) when a transfer
/// fails. Cleared by `start_internal` (either refresh mode) when refresh is
/// (re)started. Consumers treat it as an alternate completion condition for
/// a pending swap. Circular mode only sets it on a (theoretically
/// impossible) boundary-restart failure.
pub(crate) static HAS_ERROR: AtomicBool = AtomicBool::new(false);

// ---------------------------------------------------------------------------
// Platform transfer plumbing (shared by both refresh modes)
// ---------------------------------------------------------------------------

// Per-backend driver and transfer types, generic over the DMA buffer type.
//
// Only the transfer types are buffer-parameterised; the driver handles are
// not. [`TxTransfer`] fixes `B` to the active-mode [`BcmBuf`].
cfg_select! {
    hub75_use_i2s_parallel => {
        use esp_hal::i2s::parallel::{I2sParallel, I2sParallelTransfer};
        pub(crate) type TxDriver = I2sParallel<'static, Blocking>;
        pub(crate) type TxXfer<B> = I2sParallelTransfer<'static, B, Blocking>;
    }
    hub75_use_parl_io => {
        use esp_hal::parl_io::{ParlIoTx, ParlIoTxTransfer};
        pub(crate) type TxDriver = ParlIoTx<'static, Blocking>;
        pub(crate) type TxXfer<B> = ParlIoTxTransfer<'static, B, Blocking>;
    }
    hub75_use_lcd_cam => {
        use esp_hal::lcd_cam::lcd::i8080::{I8080, I8080Transfer};

        pub(crate) type TxDriver = I8080<'static, Blocking>;
        pub(crate) type TxXfer<B> = I8080Transfer<'static, B, Blocking>;
    }
    _ => {
        compile_error!("no HUB75 backend selected: enable exactly one chip feature");
    }
}

/// Transfer type for the active refresh mode's buffer.
pub(crate) type TxTransfer = TxXfer<BcmBuf>;

/// On ESP32-C5, the GDMA EOF signal is generated by the DMA channel rather
/// than the `PARL_IO` byte counter, so the transfer-length field is unused.
/// Circular `PARL_IO` (ESP32-C5 only — the `DmaEof` EOF source does not
/// exist on the C6) always uses it; linear mode uses it only on the C5.
#[cfg(all(hub75_use_parl_io, esp32c5))]
pub(crate) const PARL_IO_DUMMY_TRANSFER_LEN: usize = 0;

// ---------------------------------------------------------------------------
// Driver singleton
// ---------------------------------------------------------------------------

// Start a transfer on the active backend, mapping errors to
// [`Hub75Error`]. Shared by both refresh modes and every per-backend ISR
// path; the only backend-specific parameter is `transfer_len`
// (`PARL_IO`: the bit-length the peripheral's EOF counter is set to;
// ignored on the other backends).
cfg_select! {
    hub75_use_i2s_parallel => {
        #[cfg_attr(feature = "iram", ram)]
        pub(crate) fn start_transfer<B: DmaTxBuffer>(
            tx: TxDriver,
            buf: B,
        ) -> Result<TxXfer<B>, (Hub75Error, TxDriver, B)> {
            tx.send(buf)
                .map_err(|(err, tx, buf)| (Hub75Error::Dma(err), tx, buf))
        }
    }
    hub75_use_parl_io => {
        #[cfg_attr(feature = "iram", ram)]
        pub(crate) fn start_transfer<B: DmaTxBuffer>(
            tx: TxDriver,
            buf: B,
            transfer_len: usize,
        ) -> Result<TxXfer<B>, (Hub75Error, TxDriver, B)> {
            tx.write(transfer_len, buf)
                .map_err(|(err, tx, buf)| (Hub75Error::ParlIo(err), tx, buf))
        }
    }
    hub75_use_lcd_cam => {
        #[cfg_attr(feature = "iram", ram)]
        pub(crate) fn start_transfer<B: DmaTxBuffer>(
            tx: TxDriver,
            buf: B,
            word_size: WordSize,
        ) -> Result<TxXfer<B>, (Hub75Error, TxDriver, B)> {
            use esp_hal::lcd_cam::lcd::i8080::Command;

            let result = match word_size {
                WordSize::Eight => tx.send(Command::<u8>::None, 0, buf),
                WordSize::Sixteen => tx.send(Command::<u16>::None, 0, buf),
            };
            result.map_err(|(err, tx, buf)| (Hub75Error::Dma(err), tx, buf))
        }
    }
    _ => {}
}

/// Consume a completed transfer, returning `(result, driver, buffer)`.
///
/// `.wait()` returns instantly — the interrupt already fired — but the
/// `wait()` calls still check peripheral state, so this is safe from ISR
/// context on every backend.
///
/// Per-backend completion/flag semantics:
///
/// | Backend | `is_done()` polls | `wait()` polls & clears |
/// |---|---|---|
/// | I2S (ESP32) | `state.tx_idle` | polls `tx_idle`; clears `out_done`/`out_total_eof` in `INT_CLR` |
/// | `LCD_CAM` (S3) | `lcd_start == 0` | polls `lcd_start`; clears `lcd_trans_done` in `LC_DMA_INT_CLR` |
/// | `PARL_IO` (C5) | (via `wait`) | polls `INT_RAW.tx_eof` and clears it itself |
///
/// The `PARL_IO` case is the reason the circular ISR must **not** clear
/// `INT_RAW.tx_eof` beforehand: `clear_frame_interrupt` writes `INT_CLR`, which
/// write-clears `INT_RAW` and would leave `wait()` spinning on a flag that has
/// already been cleared.
///
/// Circular mode additionally (compiled only with `circular-dma`) asserts
/// `is_done()`: the armed chain ends on the spare boundary descriptor, so the
/// DMA must have halted before the ISR runs; busy-waiting in an interrupt
/// handler is not acceptable.
#[cfg_attr(feature = "iram", ram)]
pub(crate) fn finish_transfer<B: DmaTxBuffer<Final = B>>(
    xfer: TxXfer<B>,
) -> (Result<(), Hub75Error>, TxDriver, B) {
    // The DMA must have halted at the pass boundary (see
    // `circular::assert_transfer_done`).
    #[cfg(feature = "circular-dma")]
    circular::assert_transfer_done(&xfer);

    cfg_select! {
        hub75_use_i2s_parallel => {
            let (tx, buf) = xfer.wait();
            (Ok(()), tx, buf)
        }
        _ => {
            let (result, tx, buf) = xfer.wait();
            (result.map_err(Hub75Error::Dma), tx, buf)
        }
    }
}

/// Bind the buffer to `fb`, start the first DMA transfer, and store the
/// in-flight state. Shared by both refresh modes; the only mode-specific steps
/// are how the buffer is bound to the framebuffer (linear builds the segment
/// cache, circular builds the descriptor ring) and the `PARL_IO` transfer
/// length.
pub(crate) fn start_internal(fb: &'static impl FrameBuffer) -> Result<(), Hub75Error> {
    crate::bcm::validate_fb_internal_ram(fb);

    // Collect the swap waker under the lock and wake it after release (see
    // the swap-completion protocol in `isr`). `return` inside the closure
    // only exits the closure, so the wake below always runs.
    let mut wake = None;
    let result = STATE.with(|state| {
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
        SWAP_DONE.store(true, Ordering::Release);
        if wake.is_none() {
            wake = SWAP_WAKER.with(Option::take);
        }

        // Bind the buffer to the framebuffer.
        cfg_select! {
            feature = "circular-dma" => {
                buf.build(fb);
                state.descriptors = buf.descriptors_ptr();
                state.descriptor_count = buf.descriptor_count();
            }
            _ => {
                crate::bcm::fill_segment_cache(
                    fb,
                    unsafe { &mut *crate::bcm::cache_ptr().cast_mut() },
                );
                buf.reset_with_cache();
            }
        }
        state.current_fb_ptr = core::ptr::from_ref(fb).cast::<()>();
        state.pending_delta = None;

        // PARL_IO only: the peripheral's EOF bit-length counter. On the C5 the
        // EOF comes from the DMA channel, so the field is a dummy; circular-dma
        // on PARL_IO is C5-only, so only C6 linear reads the real counter.
        #[cfg(hub75_use_parl_io)]
        let transfer_len = cfg_select! {
            any(feature = "circular-dma", esp32c5) => PARL_IO_DUMMY_TRANSFER_LEN,
            _ => buf.current_transfer_len(),
        };

        let xfer_result = start_transfer(
            tx,
            buf,
            #[cfg(hub75_use_parl_io)]
            transfer_len,
            #[cfg(hub75_use_lcd_cam)]
            state.word_size,
        );

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
    });

    if let Some(waker) = wake {
        waker.wake();
    }
    result
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
// ISR state (shared by both refresh modes)
// ---------------------------------------------------------------------------

/// Transfer lifecycle, shared by both refresh modes.
///
/// - **Linear**: `Idle` until `start_internal()` kicks off the first transfer;
///   `Error` parks the driver and buffer until `restart()`.
/// - **Circular**: `Idle` until `start_internal()` builds the descriptor ring
///   and starts the free-running chain; `Error` records a failed
///   initial/restart transfer (theoretically impossible; surfaced to swap
///   waiters).
pub(crate) enum TransferPhase {
    Idle(TxDriver, BcmBuf),
    InFlight(TxTransfer),
    Error(Hub75Error, TxDriver, BcmBuf),
    Transitioning,
}

/// Refresh-mode ISR state.
///
/// Both modes share the pointer-delta swap protocol (`current_fb_ptr` +
/// `pending_delta`) and the same in-flight rule: exactly one outstanding
/// swap, tracked by `pending_delta.is_some()`. Mode-specific fields:
///
/// - `descriptors` / `descriptor_count`: circular only — the descriptor ring
///   the boundary ISR applies the pending delta to.
/// - `word_size`: `LCD_CAM` only — reconstructs the `I8080::send()` call when
///   the ISR restarts a transfer.
pub(crate) struct State {
    pub(crate) transfer: TransferPhase,
    /// Circular only: descriptor ring for the pending-delta application and
    /// the boundary-detector arm/disarm.
    #[cfg(feature = "circular-dma")]
    pub(crate) descriptors: *mut esp_hal::dma::DmaDescriptor,
    #[cfg(feature = "circular-dma")]
    pub(crate) descriptor_count: usize,
    /// `LCD_CAM` only: transfer word width for ISR-driven restarts.
    #[cfg(hub75_use_lcd_cam)]
    pub(crate) word_size: WordSize,
    pub(crate) current_fb_ptr: *const (),
    /// Byte offset from the current to the pending framebuffer, set by
    /// `swap()`. The ISR applies this delta at the pass/frame boundary,
    /// then clears it to `None` (which also serves as the swap-in-flight
    /// flag). Mirrors the circular-DMA invariant: both framebuffers are the
    /// same type with identical internal layout.
    pub(crate) pending_delta: Option<isize>,
}

// SAFETY (`Send`): required so the `Mutex<RefCell<Option<_>>>` static below
// is `Sync`. All access to the inner value is serialised by the embassy
// mutex (which disables interrupts and CAS-spins on an owner word on
// multi-core chips). The raw pointers are only dereferenced inside lock
// closures, so they are never accessed concurrently from multiple cores.
unsafe impl Send for State {}

pub(crate) type SharedState = Shared<Option<State>>;

pub(crate) static STATE: SharedState = Shared::new(None);

// ---------------------------------------------------------------------------
// Refresh ISR (shared by both refresh modes)
// ---------------------------------------------------------------------------

/// Refresh ISR — single handler for all backends and both refresh modes.
///
/// - **Linear**: fires at every BCM segment-group transfer completion. The
///   handler consumes the transfer, advances the buffer's group state machine,
///   applies the pending pointer delta to the cached segments at a full-frame
///   boundary, and restarts the next group transfer.
/// - **Circular**: fires only while a swap is armed (the boundary detector
///   relinks the second-to-last ring descriptor to the spare boundary
///   descriptor, `suc_eof` + `NULL` next — see [`Hub75::swap`](Hub75::swap)).
///   The chain has therefore ended on the boundary descriptor and the DMA is
///   stopped: the handler consumes the ended transfer, applies the pending
///   pointer delta to the descriptor ring while the engine is stopped, relinks
///   the ring, and restarts the transfer from the head — the swap is exact,
///   with no pass boundary ever sourced from the old buffer after the delta is
///   applied. Stale interrupts (no swap armed) are ignored; the display is
///   never touched otherwise.
#[handler]
#[cfg_attr(feature = "iram", ram)]
pub(crate) fn isr() {
    // The swap waker is collected under the lock and woken after it is
    // released (`wake()` runs executor code that must not run while `STATE`
    // is held). `return` inside the closure only exits the closure, so the
    // wake below always runs.
    let mut wake = None;
    STATE.with(|state| {
        let Some(state) = state.as_mut() else { return };

        // Circular only: gate on the armed swap. The interrupt source is
        // enabled for the driver's lifetime (see the platform constructor),
        // but the disarmed ring carries no `suc_eof`, so in steady state
        // nothing fires. Should a stale boundary flag ever appear, clear it
        // (safe here: no transfer is being waited on, even on `PARL_IO`) so
        // it cannot re-fire, and leave the display untouched.
        #[cfg(feature = "circular-dma")]
        if state.pending_delta.is_none() {
            circular::clear_frame_interrupt();
            return;
        }

        let xfer = match core::mem::replace(&mut state.transfer, TransferPhase::Transitioning) {
            TransferPhase::InFlight(xfer) => xfer,
            other => {
                state.transfer = other;
                return;
            }
        };

        // Circular only: drain the boundary flag that fired this interrupt.
        // Safe before `wait()` on I2S/LCD_CAM, whose `wait()` polls peripheral
        // *state* registers, not interrupt flags. `PARL_IO` must not clear
        // here — its `wait()` polls `INT_RAW.tx_eof`, the very flag that fired
        // this ISR — but `wait()` itself clears the flag on completion, so the
        // flag is clean after `finish_transfer` returns on every backend (and
        // therefore on every error path below too, which cannot re-fire it).
        #[cfg(feature = "circular-dma")]
        cfg_select! {
            hub75_use_parl_io => {}
            _ => {
                circular::clear_frame_interrupt();
            }
        }

        // `.wait()` returns instantly — the interrupt already fired.
        #[cfg_attr(feature = "circular-dma", allow(unused_mut))]
        let (result, tx, mut buf) = finish_transfer(xfer);
        if let Err(err) = result {
            state.transfer = TransferPhase::Error(err, tx, buf);
            HAS_ERROR.store(true, Ordering::Release);
            SWAP_DONE.store(true, Ordering::Release);
            if wake.is_none() {
                wake = SWAP_WAKER.with(Option::take);
            }
            return;
        }

        // Is this a full-frame boundary? Linear advances the group state
        // machine (one BCM group per transfer; `full-chain-dma` transfers
        // always cover a full frame). Circular only ever runs at a pass
        // boundary, so every interrupt is a boundary.
        #[cfg(not(feature = "circular-dma"))]
        let frame_boundary = buf.advance();
        #[cfg(feature = "circular-dma")]
        let frame_boundary = true;

        if frame_boundary && let Some(delta) = state.pending_delta.take() {
            // Apply the delta while the DMA is not reading the affected
            // pointers: linear shifts the cached segment pointers; circular
            // rewrites the descriptor ring while the engine is halted at the
            // pass boundary (the restart below happens only afterwards).
            #[cfg(not(feature = "circular-dma"))]
            buf.apply_delta(delta);
            #[cfg(feature = "circular-dma")]
            circular::apply_pending_delta(state, delta);
            // Linear: the swap completes at this frame boundary.
            #[cfg(not(feature = "circular-dma"))]
            {
                SWAP_DONE.store(true, Ordering::Release);
                if wake.is_none() {
                    wake = SWAP_WAKER.with(Option::take);
                }
            }
        }

        // Circular only: swap-armed boundary handled — disarm until the next
        // swap by relinking the ring (`next` back to the head).
        #[cfg(feature = "circular-dma")]
        crate::bcm::circular::disarm_boundary(state.descriptors, state.descriptor_count);

        // PARL_IO only: the peripheral's EOF bit-length counter. On the C5
        // the EOF comes from the DMA channel, so the field is a dummy (both
        // refresh modes; the `DmaEof` EOF source does not exist on the C6,
        // where linear mode computes the real length and circular mode is
        // unsupported).
        #[cfg(hub75_use_parl_io)]
        let transfer_len = cfg_select! {
            esp32c5 => PARL_IO_DUMMY_TRANSFER_LEN,
            _ => buf.current_transfer_len(),
        };

        let xfer_result = start_transfer(
            tx,
            buf,
            #[cfg(hub75_use_parl_io)]
            transfer_len,
            #[cfg(hub75_use_lcd_cam)]
            state.word_size,
        );

        match xfer_result {
            Ok(xfer) => state.transfer = TransferPhase::InFlight(xfer),
            Err((hub_err, tx, buf)) => {
                state.transfer = TransferPhase::Error(hub_err, tx, buf);
                HAS_ERROR.store(true, Ordering::Release);
                // Linear: resolve a pending swap as failed. Circular signals
                // unconditionally below.
                #[cfg(not(feature = "circular-dma"))]
                {
                    SWAP_DONE.store(true, Ordering::Release);
                    if wake.is_none() {
                        wake = SWAP_WAKER.with(Option::take);
                    }
                }
            }
        }

        // Circular only: the DMA is provably no longer reading the old
        // framebuffer now that the restart is issued (or has failed).
        #[cfg(feature = "circular-dma")]
        {
            SWAP_DONE.store(true, Ordering::Release);
            if wake.is_none() {
                wake = SWAP_WAKER.with(Option::take);
            }
        }
    });

    // Wake outside of `STATE`: `wake()` runs arbitrary executor code that
    // must not run while a mutex is held.
    if let Some(waker) = wake {
        waker.wake();
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

// `Send` is derived automatically: every field is a `PhantomData` over a
// `Send` type (`*const ()` is `Send`, `fn() -> FB` is `Send`, and `DM` is
// always `Blocking` or `Async`, both `Send`), so no manual `unsafe impl Send`
// is needed.
//
// Hub75 is intentionally `!Sync` via the `_not_sync: PhantomData<*const ()>`
// field. Even though `swap()` takes `&self` and `STATE` would serialise
// concurrent callers, sharing a `&Hub75` across cores would let two threads
// race to be the one outstanding swap and would make the single-waker-slot
// protocol in `SWAP_WAKER` ambiguous. Requiring ownership (`Send` but not
// `Sync`) keeps the driver single-owner by construction.

impl<DM: esp_hal::DriverMode, FB> Hub75<DM, FB> {
    pub(crate) fn from_phantom() -> Self {
        Self {
            _dm: core::marker::PhantomData,
            _fb: core::marker::PhantomData,
            _not_sync: core::marker::PhantomData,
        }
    }
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
    pub(crate) old_fb_ptr: *mut FB,
    pub(crate) new_fb_ptr: *mut FB,
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
    ///
    /// Returns `true` (early) if the ISR has recorded a DMA error from a
    /// prior transfer; [`wait()`](Self::wait) then resolves to the error.
    pub fn is_done(&self) -> bool {
        SWAP_DONE.load(Ordering::Acquire) || HAS_ERROR.load(Ordering::Acquire)
    }

    /// Spin-loops until the DMA is guaranteed to no longer be reading from
    /// the old framebuffer, then returns it for reuse.
    ///
    /// In non-circular mode this waits for the next frame boundary. In
    /// circular-DMA mode this waits for the armed pass boundary (the ISR
    /// applies the pointer delta while the DMA is stopped and restarts the
    /// chain from the head).
    ///
    /// If [`wait_for_done()`](Self::wait_for_done) was already awaited, this
    /// returns immediately.
    ///
    /// # Errors
    ///
    /// Returns the recorded DMA error if the ISR has failed a transfer (or,
    /// in circular mode, a boundary restart). On error the pending swap is
    /// abandoned and `new_fb` is returned for reuse.
    ///
    /// # Panics
    ///
    /// Panics if the driver has not been initialised (no `Hub75` instance
    /// was created).
    pub fn wait(self) -> Result<&'static mut FB, (Hub75Error, &'static mut FB)> {
        loop {
            if HAS_ERROR.load(Ordering::Acquire) {
                return STATE.with(|state| {
                    let state = state.as_mut().unwrap();
                    state.pending_delta = None;
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
}

// ---------------------------------------------------------------------------
// Swap (shared by both refresh modes)
// ---------------------------------------------------------------------------

impl<DM: esp_hal::DriverMode, FB: FrameBuffer + 'static> Hub75<DM, FB> {
    /// Initiate a framebuffer swap.
    ///
    /// Computes the byte offset between the old and new framebuffer
    /// allocations and registers it as the pending delta. Returns a
    /// [`Hub75Swap`](crate::Hub75Swap) transfer object; call
    /// [`.wait_for_done()`](crate::Hub75Swap::wait_for_done) then
    /// [`.wait()`](crate::Hub75Swap::wait), or just `.wait()` directly for
    /// blocking.
    ///
    /// # Swap granularity
    ///
    /// - **Circular**: the delta is applied by the boundary ISR at the next
    ///   pass boundary, while the DMA is stopped. Arming relinks the
    ///   second-to-last ring descriptor to the spare boundary descriptor
    ///   (`suc_eof` + `NULL` next) with a single atomic write, so the chain
    ///   ends at the next pass boundary; the ISR applies the pointer delta,
    ///   relinks the ring, and restarts the transfer from the head sourcing the
    ///   new framebuffer. The swap is exact, with no pass boundary ever sourced
    ///   from the old buffer after the delta is applied, and takes up to one
    ///   pass period to complete. The output is blanked for the (very brief)
    ///   halt at the pass boundary.
    /// - **Linear**: the delta is applied by the ISR to every cached segment
    ///   pointer at the next frame boundary. Both framebuffers are the same
    ///   type with identical internal layout, so the single delta shifts every
    ///   cached segment pointer.
    ///
    /// # Errors
    ///
    /// Returns [`Hub75Error::SwapInFlight`] along with ownership of `new_fb`
    /// if a previous [`Hub75Swap`](crate::Hub75Swap) is still outstanding. Only
    /// one swap may be in-flight at a time; call `.wait()` (or
    /// `.wait_for_done().await` then `.wait()`) on the previous
    /// [`Hub75Swap`](crate::Hub75Swap) before calling `swap()` again.
    ///
    /// # Panics
    ///
    /// Panics if the driver has not been initialised (no `Hub75` instance
    /// was created).
    pub fn swap(
        &self,
        new_fb: &'static mut FB,
    ) -> Result<Hub75Swap<FB>, (Hub75Error, &'static mut FB)> {
        // Linear only: pre-validate the framebuffer contract against the
        // segment cache so any panic happens outside the critical section
        // (with interrupts enabled). Circular mode streams segments straight
        // from the framebuffer and has no cache to overflow.
        #[cfg(not(feature = "circular-dma"))]
        {
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
        }

        let new_fb_ptr = core::ptr::from_mut::<FB>(new_fb);

        let old_fb_ptr = STATE.with(|state| {
            let state = state.as_mut().expect("Hub75 not initialised");
            if state.pending_delta.is_some() {
                return Err(new_fb_ptr as *const ());
            }
            let old = state.current_fb_ptr;
            // The delta shifts every descriptor/cache pointer from the old
            // to the new framebuffer; the ISR applies it at the boundary.
            let delta = new_fb_ptr as isize - old as isize;
            state.pending_delta = Some(delta);
            // Circular only: arm the boundary detector.
            //
            // `arm_boundary` copies the last ring descriptor's buffer/length
            // into the spare boundary descriptor (`suc_eof` + `NULL` next)
            // and relinks the second-to-last ring descriptor to it with a
            // single atomic write — the chain ends at the next pass boundary
            // on every backend, and the ISR restores the ring and restarts
            // the transfer. No interrupt management is needed here: the
            // frame-boundary source was enabled once at init and stays
            // enabled, and the disarmed ring produces no boundary flags, so
            // no stale flag can be pending when the detector is armed.
            #[cfg(feature = "circular-dma")]
            circular::arm_swap_detector(state);
            // `current_fb_ptr` is updated immediately; the ISR applies only
            // the delta to the descriptors/cache.
            state.current_fb_ptr = new_fb_ptr as *const ();
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
