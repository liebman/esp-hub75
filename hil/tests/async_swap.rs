//! Async swap completion: is the `wait_for_done()` future actually woken?
//!
//! `Hub75Swap::wait_for_done()` is the async half of the swap contract. The
//! blocking `wait()` spins until the ISR has retired the old framebuffer;
//! `wait_for_done()` returns a future that the same ISR has to wake at the
//! frame boundary. A bug there is invisible to every blocking test in this
//! suite -- the swap still completes, the completion flag is still set, the old
//! buffer still comes back -- and surfaces in an application as a task that
//! never runs again.
//!
//! So this file runs the future on an executor that re-polls it *only* after
//! the ISR calls the waker that the future registered ([`block_on`]). Each test
//! is guarded by `#[timeout]`: a lost wakeup leaves the future pending forever,
//! and the timeout reports that as a failure instead of a hang.
//!
//! Only the ESP32-S3 wiring has been validated on a board; `new_async` itself
//! is not chip-gated, so the file compiles for every backend.

#![no_std]
#![no_main]

// `hil` is force-linked below so that its app descriptor, defmt logger and
// `#[embedded_test::setup]` watchdog hook end up in the image.

use core::future::Future;
use core::pin::pin;
use core::ptr;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;
use core::task::Context;
use core::task::Poll;
use core::task::RawWaker;
use core::task::RawWakerVTable;
use core::task::Waker;

use hil as _;
use hil::support::AsyncFixture;
use hil::support::bring_up_async;
use hil::target::FrameBuffer;

// ---------------------------------------------------------------------------
// Executor
// ---------------------------------------------------------------------------

/// Set by the waker, cleared by the executor before each re-poll.
static WOKEN: AtomicBool = AtomicBool::new(false);

fn wake() {
    WOKEN.store(true, Ordering::Release);
}

unsafe fn clone(_: *const ()) -> RawWaker {
    RawWaker::new(ptr::null(), &WAKER_VTABLE)
}

unsafe fn wake_once(_: *const ()) {
    wake();
}

unsafe fn wake_by_ref(_: *const ()) {
    wake();
}

unsafe fn drop_waker(_: *const ()) {}

/// A waker with a null data pointer: this executor has exactly one task, so
/// "which task was woken" is not a question it has to answer.
static WAKER_VTABLE: RawWakerVTable =
    RawWakerVTable::new(clone, wake_once, wake_by_ref, drop_waker);

fn waker() -> Waker {
    // SAFETY: every function in `WAKER_VTABLE` ignores its data pointer (which
    // is null), so the waker owns nothing, and cloning or dropping it is
    // trivially sound.
    unsafe { Waker::from_raw(RawWaker::new(ptr::null(), &WAKER_VTABLE)) }
}

/// Runs `future` to completion, re-polling only after [`WOKEN`] is set.
///
/// Parking on the flag is what turns a lost wakeup into a failure rather than a
/// short delay: nothing else re-polls the future, so an ISR that sets its
/// completion flag but never calls the waker leaves this loop spinning until
/// the test's `#[timeout]` fires.
fn block_on<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let waker = waker();
    let mut cx = Context::from_waker(&waker);

    loop {
        if let Poll::Ready(output) = future.as_mut().poll(&mut cx) {
            return output;
        }
        while !WOKEN.swap(false, Ordering::AcqRel) {
            core::hint::spin_loop();
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[embedded_test::tests]
mod tests {
    use super::*;

    /// A pending swap's completion reaches an async waiter.
    ///
    /// Both halves of the contract are checked: that the wake arrives (without
    /// it `block_on` never returns), and that what `wait()` then hands back is
    /// the buffer that was displayed when the swap was requested -- the same
    /// identity `tests/lifecycle.rs` checks through the blocking API.
    #[test]
    #[timeout(30)]
    fn async_wait_for_done_is_woken() -> Result<(), &'static str> {
        let AsyncFixture {
            hub75,
            displayed,
            spare,
        } = bring_up_async()?;

        let mut pending = hub75
            .swap(spare)
            .map_err(|_| "the async swap was rejected")?;

        // No negative `is_done()` check before this: the frame boundary can
        // land between `swap()` and the check, so the assertion would be racy
        // rather than informative.
        block_on(pending.wait_for_done());

        if !pending.is_done() {
            return Err("wait_for_done() resolved before the completion flag was set");
        }

        let previous = pending
            .wait()
            .map_err(|_| "the async swap completed but then reported an error")?;
        if !ptr::eq(ptr::from_ref(previous), displayed) {
            return Err("the async swap returned a buffer other than the displayed one");
        }

        Ok(())
    }

    /// Successive async swaps each get woken: the waker slot is re-armed for
    /// every pending swap instead of being consumed by the first one.
    ///
    /// `wait_for_done()` stores its waker in a single ISR-side slot, so a
    /// driver that took it out once would pass the first test above and hang on
    /// the second swap -- the shape of failure this test is here for.
    #[test]
    #[timeout(30)]
    fn async_swaps_repeat() -> Result<(), &'static str> {
        let AsyncFixture {
            hub75,
            displayed,
            spare,
        } = bring_up_async()?;

        // What the driver displays now, and what the next round hands it.
        let mut expected: *const FrameBuffer = displayed;
        let mut buffer = spare;

        for _ in 0..3 {
            let handed_over = ptr::from_ref(&*buffer);

            let mut pending = hub75
                .swap(buffer)
                .map_err(|_| "an async swap was rejected")?;
            block_on(pending.wait_for_done());

            let previous = pending
                .wait()
                .map_err(|_| "an async swap completed but then reported an error")?;
            if !ptr::eq(ptr::from_ref(previous), expected) {
                return Err("an async swap returned a buffer other than the displayed one");
            }

            // The pair alternates: the buffer just handed over is displayed
            // once the swap lands, and the one that comes back is next round's
            // input.
            expected = handed_over;
            buffer = previous;
        }

        Ok(())
    }
}
