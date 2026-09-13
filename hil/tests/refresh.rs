//! Refresh rate on silicon: does the panel get the rate the model predicts?
//!
//! `hil::target` freezes the *model* at compile time -- `CYCLES` and
//! `REFRESH_HZ` are asserted equal to `frame_clock_cycles::<FrameBuffer>()` and
//! `refresh_hz::<FrameBuffer>(RATE)` -- but a model is not a measurement: a
//! wrong pixel-clock divisor, a PLL that does not reach the requested rate, or
//! an ISR gap between BCM groups all leave the model intact and change what the
//! panel actually receives. This file measures the other side of that
//! comparison.
//!
//! There is no frame counter in the driver, so the boundary counter here is the
//! swap path itself: `Hub75Swap::wait()` only returns once the ISR has reached
//! a frame boundary and applied the pending delta (`isr::SWAP_DONE` is set
//! there), so one `swap()`/`wait()` round trip is one *observed* boundary and
//! [`SWAPS`] of them are [`SWAPS`] boundaries over a wall-clock window. That is
//! the measurement, including its bias.
//!
//! The bias is small but stable, and it is a property of the ISR rather than
//! noise: at every interrupt the handler finishes the transfer, applies the
//! delta and restarts the engine with the DMA idle in between, which costs
//! ~10 us (measured 8-13). How many of those boundaries a frame *has* is the
//! DMA mode's business:
//!
//! * the default (group-based) mode interrupts once per BCM group -- 4 per
//!   frame on this panel -- and restarts a transfer each time, so a frame pays
//!   the cost four times: the measured ~5% (43 us at 16-bit, 44 at 8-bit);
//! * `full-chain-dma` and `circular-dma` move a whole frame (or pass) per
//!   transfer, so a frame pays it once: the measured ~1.5% (13 us at 16-bit, 8
//!   at 8-bit).
//!
//! The swap loop itself adds almost nothing: the two modes only agree if each
//! frame boundary costs about the same, and 4 x 10.75 us (the group mode's
//! 43 us over four boundaries) against 8-13 us (the frame modes' one) leaves a
//! couple of microseconds for the handoff between two loop iterations. So the
//! number to carry away is that a frame pays its ISR cost once per interrupt
//! and no part of `hil::target` can see it: the model knows the pixel clock and
//! the BCM sequence, not how the driver chops that sequence into transfers.
//! That is exactly why this belongs in a measurement.
//!
//! Measured on the ESP32-S3 (`--release`, `Instant`, `--features defmt` -- i.e.
//! with RTT polling enabled, which inflates every figure slightly): one swap in
//! permille of the ideal frame, and the refresh rate that implies.
//!
//! | Wiring | default (group) | `full-chain-dma` | `circular-dma` | model |
//! | --- | --- | --- | --- | --- |
//! | `bus16` | 1056 -- 811 us, 1232 Hz | 1017 -- 781 us, 1280 Hz | 1014 -- 779 us, 1283 Hz | 768 us, 1302 Hz |
//! | `bus8` | 1051 -- 908 us, 1100 Hz | 1010 -- 872 us, 1145 Hz | 1011 -- 873 us, 1144 Hz | 864 us, 1157 Hz |
//!
//! Read that as: the two DMA modes that interrupt once per frame refresh ~1%
//! slower than the model, and the default group-based mode refreshes ~5%
//! slower, because it interrupts four times per frame and pays the ISR's
//! restart cost each time. The model is a property of the pixel clock and the
//! BCM sequence; the gap is a property of the ISR, which is why it shows up
//! here and nowhere in `hil::target`.
//!
//! The bands below hold those numbers with margin on both sides, and are narrow
//! enough that a refresh loop really running at a different rate -- a doubled
//! or halved clock, a stalled or restarted group -- fails.

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

/// Frame boundaries to observe, i.e. swap round trips to time.
///
/// A frame is ~0.8 ms, so this is ~160 ms of wall clock: long enough that the
/// per-boundary jitter averages out, and far inside the `#[timeout]`.
const SWAPS: u32 = 200;

/// Microseconds in one frame, from the compile-time model.
const FRAME_US: u64 = 1_000_000 / REFRESH_HZ as u64;

/// Per-swap time accepted, in permille of `FRAME_US`.
///
/// The lower bound is the model itself, less 1% of slack. A frame boundary
/// cannot arrive before the ISR has consumed the transfer that ends it, and
/// that cost is always positive, so a frame period *shorter* than the model
/// means the pixel clock really is faster than `RATE` -- the overclock this
/// test exists to catch. The slack absorbs `Instant`'s granularity over the
/// measurement window.
const MIN_PERMILLE: u64 = 990;

/// The upper bound depends on the DMA mode, because the mode decides how much
/// ISR work sits *inside* the frame period:
///
/// * default (group-based): one interrupt per BCM group -- four per frame on
///   this panel -- and the DMA is restarted each time, so the engine is idle
///   for ~11 us per group: ~5% of a frame on the bench, on both wirings;
/// * `full-chain-dma` / `circular-dma`: one interrupt per frame, so a frame
///   pays the boundary cost once, ~1.5%.
///
/// Both figures are measured with margin rather than derived: the group gap is
/// a backend and BCM-sequence property, which the compile-time model knows
/// nothing about.
#[cfg(not(any(feature = "full-chain-dma", feature = "circular-dma")))]
const MAX_PERMILLE: u64 = 1300;

/// See [`MAX_PERMILLE`].
#[cfg(any(feature = "full-chain-dma", feature = "circular-dma"))]
const MAX_PERMILLE: u64 = 1100;

#[embedded_test::tests]
mod tests {
    use super::*;

    /// Times [`SWAPS`] frame boundaries through the swap path and compares the
    /// result with the compile-time refresh model.
    ///
    /// The identity check on every iteration keeps the measurement honest: a
    /// loop that returned the wrong buffer would be timing something other than
    /// the swap path (and would fail in `tests/stress.rs` anyway, but a timing
    /// test should not depend on another file having caught it).
    #[test]
    #[timeout(30)]
    fn measured_refresh_matches_the_model() -> Result<(), &'static str> {
        let Fixture {
            hub75,
            displayed,
            spare,
            ..
        } = bring_up()?;

        // `held` is the buffer this test owns and hands over next; `expected`
        // is the address of the buffer being displayed right now. Both start on
        // the state `bring_up` left behind.
        let mut held = spare;
        let mut expected = displayed;

        // Warm-up, untimed: it costs a frame like every other round trip, but
        // it starts from an unknown phase (`bring_up` finished somewhere inside
        // a frame) and it settles the pair into the alternation the timed loop
        // then assumes.
        let handed_over = ptr::from_ref(&*held);
        let previous = hub75
            .swap(held)
            .map_err(|_| "the warm-up swap was rejected")?
            .wait()
            .map_err(|_| "the warm-up swap failed")?;
        if !ptr::eq(ptr::from_ref(previous), expected) {
            return Err("the warm-up swap returned a buffer other than the displayed one");
        }
        held = previous;
        expected = handed_over;

        let start = Instant::now();
        for _ in 0..SWAPS {
            let handed_over = ptr::from_ref(&*held);

            let previous = hub75
                .swap(held)
                .map_err(|_| "a swap was refused while no other swap was pending")?
                .wait()
                .map_err(|_| "a transfer failed while measuring the refresh rate")?;

            if !ptr::eq(ptr::from_ref(previous), expected) {
                return Err("a timed swap returned a buffer other than the one being displayed");
            }

            held = previous;
            expected = handed_over;
        }
        let elapsed_us = start.elapsed().as_micros();

        // Per-swap time in permille of the model frame, and the refresh rate
        // that implies. Integer math only: a `no_std` image has no FPU path
        // worth taking for this.
        let ideal_us = SWAPS as u64 * FRAME_US;
        let permille = elapsed_us * 1000 / ideal_us;
        let measured_hz = 1_000_000 * SWAPS as u64 / elapsed_us;

        // Compiles away unless the `defmt` feature is on; with it on, the
        // numbers appear in the RTT output of a run, which is how the table in
        // the module docs was filled in.
        defmt::info!(
            "refresh: {} swaps in {} us, {} us/swap ({} permille of {} us), {} Hz vs {} Hz",
            SWAPS,
            elapsed_us,
            elapsed_us / SWAPS as u64,
            permille,
            FRAME_US,
            measured_hz,
            REFRESH_HZ
        );

        if permille < MIN_PERMILLE {
            return Err("frame boundaries arrived faster than the refresh model allows");
        }
        if permille > MAX_PERMILLE {
            return Err("frame boundaries arrived far slower than the refresh model allows");
        }

        Ok(())
    }
}
