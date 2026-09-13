//! Does `embedded-test` hand every test case a clean chip?
//!
//! Every result in this suite rests on the assumption that a test case starts
//! with cleared statics: the test images link with `-nostartfiles`, so the
//! C runtime's `.bss` zeroing is *not* obviously in play, and the driver keeps
//! its singleton guard (`DRIVER_TAKEN`), the swap state and the pending delta
//! in module-level statics. If those survived from one test case to the next,
//! a second `Hub75::new` would fail with `AlreadyInitialised` and any test that
//! constructs its own driver would report a failure that has nothing to do with
//! the driver.
//!
//! The two tests below prove the opposite, and they are deliberately
//! **identical**: a canary is stored in a static that is expected to read `0`
//! again, and the driver is brought up and swapped. Whichever of the two runs
//! second is the one doing the real check -- if statics persisted, its canary
//! assertion fails and its construction returns `AlreadyInitialised`. Making
//! the bodies identical keeps the proof independent of the order
//! `embedded-test` runs the cases in, which is not a documented property.
//!
//! This file is the reason the other test files may bring the driver up once
//! per test case instead of once per file.

#![no_std]
#![no_main]

// `hil` is force-linked below so that its app descriptor, defmt logger and
// `#[embedded_test::setup]` watchdog hook end up in the image.

use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;

use hil as _;
use hil::support::Fixture;
use hil::support::bring_up;

/// Written by one test case, expected to read zero in the next.
static CANARY: AtomicU32 = AtomicU32::new(0);

/// A value that cannot be confused with the cleared state.
const CANARY_SET: u32 = 0xC0FF_EE00;

/// Brings the driver up and completes one swap.
///
/// Sharing the bring-up with the other test files is deliberate: the driver's
/// own guard (`DRIVER_TAKEN`) is the state this file is really probing, and
/// running the same code as every other file is what makes the probe mean
/// something.
fn construct_and_swap() -> Result<(), &'static str> {
    let Fixture { hub75, spare, .. } = bring_up()?;

    // The driver owns the buffer it was constructed with, so only `spare` --
    // the one about to be handed over -- can be written to.
    spare.erase();

    // `wait()` only returns once the ISR has reached a frame boundary and
    // applied the swap, so a transfer that never completes fails this test by
    // timing out rather than by silently passing.
    hub75
        .swap(spare)
        .map_err(|_| "swap() rejected while another swap was in flight")?
        .wait()
        .map_err(|_| "DMA transfer failed during the swap")?;

    Ok(())
}

#[embedded_test::tests]
mod tests {
    use super::*;

    /// Plants the canary and constructs the driver.
    ///
    /// See the module docs: this and the test below are the same test twice on
    /// purpose, so that whichever runs second is the one that proves statics
    /// did not survive the reset.
    #[test]
    #[timeout(30)]
    fn canary_case_one() -> Result<(), &'static str> {
        assert_eq!(
            CANARY.load(Ordering::Relaxed),
            0,
            "canary was already set before this test case ran"
        );
        CANARY.store(CANARY_SET, Ordering::Relaxed);
        assert_eq!(
            CANARY.load(Ordering::Relaxed),
            CANARY_SET,
            "the canary did not survive its own store"
        );

        construct_and_swap()
    }

    /// Checks that the canary is gone and constructs the driver again.
    ///
    /// Both halves matter: the canary covers every static in the test image,
    /// and the second construction covers the driver's own guard, which is the
    /// one that would otherwise make a later test fail for the wrong reason.
    #[test]
    #[timeout(30)]
    fn canary_case_two() -> Result<(), &'static str> {
        assert_eq!(
            CANARY.load(Ordering::Relaxed),
            0,
            "the canary survived from the previous test case: statics are not cleared by the reset"
        );
        CANARY.store(CANARY_SET, Ordering::Relaxed);
        assert_eq!(
            CANARY.load(Ordering::Relaxed),
            CANARY_SET,
            "the canary did not survive its own store"
        );

        construct_and_swap()
    }
}
