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

use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;
use core::task::Waker;

use esp_hal::interrupt::InterruptHandler;
use esp_hal::interrupt::Priority;
use esp_sync::NonReentrantMutex;

use crate::Hub75Error;
use crate::framebuffer::FrameBuffer;

#[cfg(feature = "circular-dma")]
pub(crate) mod circular;
#[cfg(not(feature = "circular-dma"))]
pub(crate) mod linear;

// Re-export the active mode's items under their original paths so callers
// (`i2s_parallel.rs`, `lcd_cam.rs`, `parl_io.rs`) are unaffected by the
// module split.
#[cfg(all(feature = "circular-dma", hub75_use_parl_io, esp32c5))]
pub(crate) use circular::PARL_IO_DUMMY_TRANSFER_LEN;
#[cfg(feature = "circular-dma")]
pub(crate) use circular::hub75_boundary_isr;
#[cfg(feature = "circular-dma")]
pub(crate) use circular::store_circular_state;
#[cfg(not(feature = "circular-dma"))]
pub(crate) use linear::hub75_isr;
#[cfg(not(feature = "circular-dma"))]
pub(crate) use linear::init_isr_state;
#[cfg(not(feature = "circular-dma"))]
pub(crate) use linear::start_internal;

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
/// Note (ESP32/S3): the first chunk of the just-started pass may still be
/// clocked out from the old buffer for up to one descriptor transfer time
/// (~200us at 4KiB/10MHz) after this flag is set; reclaim is still sound
/// because the old buffer remains valid, just briefly shared.
pub(crate) static SWAP_DONE: AtomicBool = AtomicBool::new(false);
/// Set to true once a Hub75 instance has been created. Prevents a second
/// constructor from overwriting the ISR state statics. Hub75 has no Drop
/// (run-forever design), so this is never reset.
static DRIVER_TAKEN: AtomicBool = AtomicBool::new(false);
pub(crate) static SWAP_WAKER: Shared<Option<Waker>> = Shared::new(None);

pub(crate) fn signal_swap_done() {
    SWAP_DONE.store(true, Ordering::Release);
    // Take the waker under the lock but wake outside of it: `wake()` runs
    // arbitrary executor code that must not run while a mutex is held.
    let waker = SWAP_WAKER.with(Option::take);
    if let Some(waker) = waker {
        waker.wake();
    }
}

// ---------------------------------------------------------------------------
// Driver singleton
// ---------------------------------------------------------------------------

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
    #[cfg(not(feature = "circular-dma"))]
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
    pub fn is_done(&self) -> bool {
        #[cfg(not(feature = "circular-dma"))]
        {
            SWAP_DONE.load(Ordering::Acquire) || linear::HAS_ERROR.load(Ordering::Acquire)
        }
        #[cfg(feature = "circular-dma")]
        {
            SWAP_DONE.load(Ordering::Acquire)
        }
    }
}
