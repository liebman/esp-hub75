//! Sustained swap throughput: does the swap path stay correct under load, and
//! does refresh keep its speed while it is being hammered?
//!
//! A single swap only exercises one frame boundary. The failure modes this file
//! looks for need repetition: a completion path that clears the pending delta
//! slightly too late shows up as a `SwapInFlight` refusal a few dozen swaps in;
//! one that swaps the pointers the wrong way round shows up as a returned
//! buffer that is not the one that was displayed; a bug that only appears in
//! the `full-chain-dma` / `circular-dma` refresh modes shows up only when the
//! test is built with those features.
//!
//! The timing check is a ceiling, not a benchmark: `REFRESH_HZ` says how many
//! frame boundaries pass per second, and every swap has to reach one, so a
//! storm of `SWAPS` swaps cannot be faster than `SWAPS` frames -- and if it is
//! *much* slower, the refresh loop is stalling between swaps. The ceiling is
//! deliberately loose; it is there to catch a stall, not to measure jitter.
//! The frame rate itself is measured in `tests/refresh.rs`, which times the
//! very same round trip against the model and knows what each DMA mode's
//! overhead is.

#![no_std]
#![no_main]

// `hil` is force-linked below so that its app descriptor, defmt logger and
// `#[embedded_test::setup]` watchdog hook end up in the image.

use core::ptr;

use esp_hal::time::Instant;
use hil as _;
use hil::support::Fixture;
use hil::support::bring_up;
use hil::target::REFRESH_HZ;

/// Swaps to complete in one test.
///
/// At ~1.3 kHz that is a few hundred milliseconds, far inside the test
/// timeout, but enough boundaries to expose an off-by-one in the completion
/// state machine: the pair has to alternate and every check below runs on
/// every iteration.
const SWAPS: u32 = 300;

/// Microseconds in one refresh period, from the compile-time model.
const FRAME_US: u64 = 1_000_000 / REFRESH_HZ as u64;

/// Ceiling for the whole storm.
///
/// Each swap completes at the first frame boundary after its `swap()` call, so
/// the run costs about one frame per swap. Measured on the ESP32-S3 with RTT
/// instrumentation enabled (which inflates the figure slightly):
///
/// | Wiring | Measured | Per swap | Ideal frame |
/// | --- | --- | --- | --- |
/// | `bus16` (direct drive) | 244,762 us | 815 us | 768 us |
/// | `bus8` (latched) | 272,680 us | 908 us | 864 us |
///
/// So a healthy storm costs ~1.06 frames per swap on both wirings. Twice the
/// ideal is therefore a real ceiling with ~90% headroom: it passes the
/// measured behaviour comfortably and fails if the refresh loop starts
/// stalling between swaps, which is what this checks.
const MAX_STORM_US: u64 = SWAPS as u64 * FRAME_US * 2;

#[embedded_test::tests]
mod tests {
    use super::*;

    /// Three hundred swaps across a two-buffer ping-pong, with the returned
    /// pointer and the swap result checked on every iteration.
    ///
    /// The identity check is what makes this more than a loop: `wait()` must
    /// return the buffer that was *displayed* a moment ago, every single time.
    /// A driver that returns the buffer it just started displaying would let
    /// the test overwrite the memory the DMA is reading, and the panel would
    /// show tearing instead of a test failure.
    #[test]
    #[timeout(30)]
    fn swap_storm_keeps_the_pair_alternating() -> Result<(), &'static str> {
        let Fixture {
            hub75,
            displayed,
            spare,
            ..
        } = bring_up()?;

        // `held` is the buffer we own and will hand over next; `expected` is
        // the address of the buffer the driver is displaying right now. They
        // start on opposite buffers and swap roles on every iteration.
        let mut held = spare;
        let mut expected = displayed;

        let start = Instant::now();
        for _ in 0..SWAPS {
            let handed_over = ptr::from_ref(&*held);

            let previous = hub75
                .swap(held)
                .map_err(|_| "a swap was refused while no other swap was pending")?
                .wait()
                .map_err(|_| "a transfer failed during the storm")?;

            if !ptr::eq(ptr::from_ref(previous), expected) {
                return Err("a swap returned a buffer other than the one being displayed");
            }

            held = previous;
            expected = handed_over;
        }
        let elapsed_us = start.elapsed().as_micros();

        // Compiles away unless the `defmt` feature is on; with it on, the
        // number appears in the RTT output of a run, which is how the ceiling
        // below was measured rather than guessed.
        defmt::info!(
            "swap storm: {} swaps in {} us ({} us/frame)",
            SWAPS,
            elapsed_us,
            elapsed_us / SWAPS as u64
        );

        // Each swap must reach a boundary *after* the previous one, so `SWAPS`
        // swaps span at least `SWAPS - 1` frame periods.
        let floor_us = (SWAPS as u64 - 1) * FRAME_US;
        if elapsed_us < floor_us {
            return Err("the swap storm completed faster than the refresh rate allows");
        }
        if elapsed_us > MAX_STORM_US {
            return Err("the swap storm took far longer than the refresh rate implies");
        }

        Ok(())
    }
}
