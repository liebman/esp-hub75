//! Shared ISR-driven BCM refresh infrastructure.
//!
//! Compiled for every platform that uses an interrupt-driven refresh loop
//! (ESP32 via I2S Parallel, ESP32-S3 via `LCD_CAM`, ESP32-C5/C6 via
//! `PARL_IO`). This module holds the swap-signalling core, the [`Transfer`]
//! plumbing, the refresh [`isr`], and the public [`Hub75`](crate::Hub75) /
//! [`Hub75Swap`](crate::Hub75Swap) driver API.
//!
//! The two refresh modes are compile-time selected (`circular-dma`):
//!
//! - **Linear**: one DMA transfer per BCM segment group, restarted by the ISR.
//! - **Circular**: a single free-running circular descriptor chain with
//!   pointer-delta swaps at pass boundaries.
//!
//! Both modes share one refresh [`isr`]: the mode differences (stale-interrupt
//! gating, group advancement, delta target, disarm, and completion signalling)
//! are `#[cfg]` branches inside the handler and the swap path. The
//! descriptor-chain and segment-cache mechanics live in [`crate::bcm`].

use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;
use core::task::Waker;

use esp_hal::handler;
use esp_hal::interrupt::InterruptHandler;
use esp_hal::interrupt::Priority;
#[cfg(feature = "iram")]
use esp_hal::ram;
use esp_sync::NonReentrantMutex;

use crate::Hub75;
use crate::Hub75Error;
use crate::Hub75Swap;
// The active refresh mode's BCM buffer. The mode differences themselves are
// compiled into [`isr`], [`Hub75::swap`] and [`start_internal`] below as
// `#[cfg]` branches; the descriptor-chain and segment-cache mechanics live in
// [`crate::bcm`].
#[cfg(feature = "circular-dma")]
pub(crate) use crate::bcm::circular::BcmBuf;
#[cfg(not(feature = "circular-dma"))]
pub(crate) use crate::bcm::linear::BcmBuf;
use crate::framebuffer::FrameBuffer;
#[cfg(hub75_use_lcd_cam)]
use crate::framebuffer::WordSize;

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
/// boundary has fully finished (the ISR consumed it via [`Transfer::finish`],
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

// Platform transfer plumbing (the per-backend driver/transfer types and the
// [`Transfer`] lifecycle) lives in [`transfer`], re-exported here for the rest
// of the engine.
mod transfer;
pub(crate) use transfer::Transfer;
pub(crate) use transfer::TxDriver;

/// Bind the buffer to `fb`, start the first DMA transfer, and park the
/// in-flight state. Shared by both refresh modes; the only mode-specific steps
/// are how the buffer is bound to the framebuffer (linear builds the segment
/// cache, circular builds the descriptor ring) — the transfer itself is
/// started through [`Transfer::start`].
pub(crate) fn start_internal(fb: &'static impl FrameBuffer) -> Result<(), Hub75Error> {
    crate::bcm::validate_fb_internal_ram(fb);

    // Collect the swap waker under the lock and wake it after release (see
    // the swap-completion protocol in `isr`). `return` inside the closure
    // only exits the closure, so the wake below always runs.
    let mut wake = None;
    let result = STATE.with(|state| {
        let state = state.as_mut().expect("Hub75 not initialised");

        if !state.transfer.is_idle() {
            return Err(Hub75Error::AlreadyRunning);
        }

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
                let (descriptors, descriptor_count) = {
                    let buf = state.transfer.buf_mut();
                    buf.build(fb);
                    (buf.descriptors_ptr(), buf.descriptor_count())
                };
                state.descriptors = descriptors;
                state.descriptor_count = descriptor_count;
            }
            _ => {
                state.transfer.buf_mut().bind_cache(fb);
            }
        }
        state.current_fb_ptr = core::ptr::from_ref(fb).cast::<()>();
        state.pending_delta = None;

        match state.transfer.start() {
            Ok(()) => {
                HAS_ERROR.store(false, Ordering::Release);
                SWAP_DONE.store(false, Ordering::Release);
                Ok(())
            }
            Err(err) => {
                HAS_ERROR.store(true, Ordering::Release);
                Err(err)
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

/// Refresh-mode ISR state.
///
/// Both modes share the pointer-delta swap protocol (`current_fb_ptr` +
/// `pending_delta`) and the same in-flight rule: exactly one outstanding
/// swap, tracked by `pending_delta.is_some()`. Mode-specific fields:
///
/// - `descriptors` / `descriptor_count`: circular only — the descriptor ring
///   the boundary ISR applies the pending delta to.
///
/// The transfer plumbing (driver handle, BCM buffer, transfer phase and the
/// `LCD_CAM` `word_size`) lives in [`Transfer`].
pub(crate) struct State {
    pub(crate) transfer: Transfer,
    /// Circular only: descriptor ring for the pending-delta application and
    /// the boundary-detector arm/disarm.
    #[cfg(feature = "circular-dma")]
    pub(crate) descriptors: *mut esp_hal::dma::DmaDescriptor,
    #[cfg(feature = "circular-dma")]
    pub(crate) descriptor_count: usize,
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

/// Store the ISR state before the platform constructor starts the DMA.
///
/// Both refresh modes share the same two-step boot: the `(driver, buffer)`
/// pair is parked as `Idle` inside [`Transfer`], and [`start_internal`] then
/// binds the buffer to the framebuffer and kicks off the first transfer.
/// `word_size` is the transfer's word width — only `LCD_CAM` passes it.
pub(crate) fn init_state(tx: TxDriver, buf: BcmBuf, #[cfg(hub75_use_lcd_cam)] word_size: WordSize) {
    STATE.with(|state| {
        *state = Some(State {
            transfer: Transfer::new(
                tx,
                buf,
                #[cfg(hub75_use_lcd_cam)]
                word_size,
            ),
            #[cfg(feature = "circular-dma")]
            descriptors: core::ptr::null_mut(),
            #[cfg(feature = "circular-dma")]
            descriptor_count: 0,
            current_fb_ptr: core::ptr::null(),
            pending_delta: None,
        });
    });
}

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
            Transfer::clear_frame_interrupt();
            return;
        }

        // Spurious interrupt (no transfer in flight): nothing to consume.
        if !state.transfer.is_in_flight() {
            return;
        }

        // Consume the completed transfer and park the engine. The per-backend
        // pre-`wait()` flag handling and the circular boundary assertion live
        // in `Transfer::finish`.
        if state.transfer.finish().is_err() {
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
        let frame_boundary = state.transfer.buf_mut().advance();
        #[cfg(feature = "circular-dma")]
        let frame_boundary = true;

        if frame_boundary && let Some(delta) = state.pending_delta.take() {
            // Apply the delta while the DMA is not reading the affected
            // pointers: linear shifts the cached segment pointers; circular
            // rewrites the descriptor ring while the engine is halted at the
            // pass boundary (the restart below happens only afterwards).
            #[cfg(not(feature = "circular-dma"))]
            state.transfer.buf_mut().apply_delta(delta);
            #[cfg(feature = "circular-dma")]
            crate::bcm::circular::apply_delta(state.descriptors, state.descriptor_count, delta);
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

        // Restart: linear kicks off the next group transfer, circular restarts
        // the ring from the head. The backend parameters (including the
        // `PARL_IO` transfer length) are derived inside `Transfer::start`.
        if state.transfer.start().is_err() {
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
// Hub75Swap implementation (the type itself lives in `lib.rs`)
// ---------------------------------------------------------------------------

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
                    let err = state
                        .transfer
                        .error()
                        .unwrap_or(Hub75Error::Dma(esp_hal::dma::DmaError::DescriptorError));
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
                count <= crate::bcm::linear::MAX_SEGMENTS,
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
            crate::bcm::circular::arm_boundary(state.descriptors, state.descriptor_count);
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

// ---------------------------------------------------------------------------
// Hub75::restart (linear only — circular never stops, so has no restart)
// ---------------------------------------------------------------------------

#[cfg(not(feature = "circular-dma"))]
impl<DM: esp_hal::DriverMode, FB: FrameBuffer + 'static> Hub75<DM, FB> {
    /// Restart display refresh after an error.
    ///
    /// Callable after [`Hub75::swap`](Hub75::swap) returned an error.
    /// Sets up the BCM segment data and kicks off the first DMA transfer with
    /// the same framebuffer type.
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
    pub fn restart(&self, fb: &'static FB) -> Result<(), Hub75Error> {
        start_internal(fb)
    }
}
