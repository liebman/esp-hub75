//! Swap lifecycle: what the driver promises about the one in-flight swap.
//!
//! `Hub75::swap` takes ownership of the buffer that is about to be displayed
//! and only hands the previous one back once the DMA is guaranteed to be done
//! reading it. That contract has three parts, and each one is a way for a user
//! to corrupt the display or the heap if the driver gets it wrong:
//!
//! * only one swap may be in flight, and a rejected `swap()` must return the
//!   buffer it was given rather than dropping it (this file's first test),
//! * a completed swap must clear the state that made `swap()` refuse the next
//!   call, and the buffers must alternate exactly (second test),
//! * a refused `restart()` must not resolve or disturb the swap that is already
//!   pending (third test, non-circular mode only).
//!
//! Every test constructs its own driver: `tests/reset.rs` proves that
//! `embedded-test` resets the chip -- and therefore the driver's statics --
//! before each test case.

#![no_std]
#![no_main]

// `hil` is force-linked below so that its app descriptor, defmt logger and
// `#[embedded_test::setup]` watchdog hook end up in the image.

use core::ptr;

use esp_hal::time::Instant;
use esp_hub75::Hub75Error;
use hil as _;
use hil::support::Fixture;
use hil::support::bring_up;

/// How long `is_done()` may take to report a completed swap.
///
/// One frame is `1_000_000 / REFRESH_HZ` microseconds (768 us on the 16-bit
/// wiring, 864 on the latched one), so this allows for ~60 frames. It is a
/// guard against a completion signal that never arrives, not a timing
/// assertion: without it the failure would only surface as the test timeout,
/// with nothing to say about which flag went missing.
const DONE_DEADLINE_US: u64 = 50_000;

#[embedded_test::tests]
mod tests {
    use super::*;

    /// A second `swap()` while one is in flight is refused, the buffer it was
    /// given comes back, and the swap that was already pending still lands.
    ///
    /// The returned-identity check is the one that matters for memory safety:
    /// if the driver dropped the buffer instead of returning it, a caller that
    /// trusts it would lose a framebuffer per rejected swap; if it returned the
    /// wrong one, the caller would hand the DMA a buffer the driver still owns.
    #[test]
    #[timeout(30)]
    fn swap_in_flight_is_rejected() -> Result<(), &'static str> {
        let Fixture {
            hub75,
            displayed,
            spare,
            spare2,
            ..
        } = bring_up()?;

        let pending = hub75
            .swap(spare)
            .map_err(|_| "the first swap was rejected")?;

        let spare2_addr = ptr::from_ref(&*spare2);
        match hub75.swap(spare2) {
            Ok(_) => return Err("a second swap was accepted while one was in flight"),
            Err((err, returned)) => {
                if !matches!(err, Hub75Error::SwapInFlight) {
                    return Err("the second swap failed with something other than SwapInFlight");
                }
                if !ptr::eq(ptr::from_ref(returned), spare2_addr) {
                    return Err("the rejected swap did not return the buffer it was given");
                }
            }
        }

        // The rejected swap must not have resolved, or otherwise disturbed, the
        // one that is still pending.
        let deadline = Instant::now();
        while !pending.is_done() {
            if deadline.elapsed().as_micros() > DONE_DEADLINE_US {
                return Err("is_done() never reported the pending swap as complete");
            }
            core::hint::spin_loop();
        }

        let previous = pending
            .wait()
            .map_err(|_| "the pending swap failed after a rejected second swap")?;
        if !ptr::eq(ptr::from_ref(previous), displayed) {
            return Err("wait() returned a buffer other than the one being displayed");
        }

        Ok(())
    }

    /// A swap right after a completed swap is accepted, and the two buffers
    /// alternate exactly across three swaps.
    ///
    /// This is what catches a completion path that forgets to clear the pending
    /// delta or the swap-done flag: the driver would either refuse every
    /// subsequent swap (`SwapInFlight`) or return the buffer it is currently
    /// displaying, which a caller would then overwrite while the DMA reads it.
    #[test]
    #[timeout(30)]
    fn swaps_repeat_across_the_pair() -> Result<(), &'static str> {
        let Fixture {
            hub75,
            displayed,
            spare,
            spare2,
            ..
        } = bring_up()?;

        let spare_addr = ptr::from_ref(&*spare);
        let spare2_addr = ptr::from_ref(&*spare2);

        let previous = hub75
            .swap(spare)
            .map_err(|_| "the first swap was rejected")?
            .wait()
            .map_err(|_| "the first swap failed")?;
        if !ptr::eq(ptr::from_ref(previous), displayed) {
            return Err("the first swap returned a buffer other than the displayed one");
        }

        // `previous` is the buffer that was on screen a moment ago and is now
        // reusable -- which is exactly what the next swap depends on.
        let previous2 = hub75
            .swap(spare2)
            .map_err(|_| "a swap right after a completed swap was rejected")?
            .wait()
            .map_err(|_| "the second swap failed")?;
        if !ptr::eq(ptr::from_ref(previous2), spare_addr) {
            return Err("the second swap returned the wrong framebuffer");
        }

        // And the pair keeps alternating instead of drifting to one buffer.
        let previous3 = hub75
            .swap(previous)
            .map_err(|_| "the third swap was rejected")?
            .wait()
            .map_err(|_| "the third swap failed")?;
        if !ptr::eq(ptr::from_ref(previous3), spare2_addr) {
            return Err("the third swap did not return the buffer the second swap displayed");
        }

        Ok(())
    }

    /// `restart()` is refused while refresh is in flight, and its refusal
    /// leaves the pending swap and the driver working.
    ///
    /// The refusal is deterministic rather than a race: the ISR holds the
    /// driver's state lock across consuming a finished transfer, applying the
    /// pending delta and starting the next one, so a caller can never observe
    /// the idle window in between. `restart()` therefore always sees a running
    /// transfer while refresh is healthy.
    ///
    /// Not available in circular-DMA mode: the ring never stops, so the driver
    /// has no `restart()` there.
    #[cfg(not(feature = "circular-dma"))]
    #[test]
    #[timeout(30)]
    fn restart_while_swap_is_pending_is_rejected() -> Result<(), &'static str> {
        let Fixture {
            hub75,
            displayed,
            spare,
            spare2,
        } = bring_up()?;

        let spare_addr = ptr::from_ref(&*spare);

        let pending = hub75
            .swap(spare)
            .map_err(|_| "the first swap was rejected")?;

        // `restart()` takes a `&'static` reference, so offering it `spare2`
        // borrows that buffer for the rest of the test -- the swaps below
        // therefore ping-pong the buffers that come back rather than naming it
        // again. The rejection happens before `restart()` touches any state, so
        // the pending swap must still resolve normally, and the driver must
        // never start displaying `spare2`.
        match hub75.restart(&*spare2) {
            Ok(()) => return Err("restart() was accepted while a transfer was in flight"),
            Err(Hub75Error::AlreadyRunning) => {}
            Err(_) => return Err("restart() failed with something other than AlreadyRunning"),
        }

        let previous = pending
            .wait()
            .map_err(|_| "the pending swap did not survive a rejected restart")?;
        if !ptr::eq(ptr::from_ref(previous), displayed) {
            return Err("the pending swap returned a buffer other than the displayed one");
        }

        // Refresh is still healthy: the pair keeps alternating, starting from
        // the buffer the pending swap just handed back.
        let previous2 = hub75
            .swap(previous)
            .map_err(|_| "the driver stopped accepting swaps after a rejected restart")?
            .wait()
            .map_err(|_| "the swap after a rejected restart failed")?;
        if !ptr::eq(ptr::from_ref(previous2), spare_addr) {
            return Err("the swap after a rejected restart returned the wrong framebuffer");
        }

        let previous3 = hub75
            .swap(previous2)
            .map_err(|_| "the driver stopped accepting swaps after a rejected restart")?
            .wait()
            .map_err(|_| "the second swap after a rejected restart failed")?;
        if !ptr::eq(ptr::from_ref(previous3), displayed) {
            return Err("the second swap after a rejected restart returned the wrong framebuffer");
        }

        Ok(())
    }
}
