//! Bring-up tests: do we build, flash, boot and report results on hardware?
//!
//! These are deliberately the smallest possible tests. The interesting ones
//! (feature-effect assertions, DMA modes, units) build on top of them.
//!
//! The bus width is a crate feature, not a property of this file: the
//! framebuffer (`hil::target`), the pin map and the backend (`hil::support`)
//! all follow from it, so the tests below run against either wiring unchanged.
//!
//! This file keeps the one test that constructs the driver twice inside a
//! single test case, to check the singleton guard. `tests/reset.rs` covers the
//! other half of the picture -- that every test *case* starts from cleared
//! statics -- which is what lets the later files construct their own driver.

#![no_std]
#![no_main]

// `hil` is force-linked below so that its app descriptor, defmt logger and
// `#[embedded_test::setup]` watchdog hook end up in the image.

use core::fmt::Write;

use esp_hal::interrupt::Priority;
use esp_hal::time::Rate;
use esp_hub75::Hub75;
use esp_hub75::Hub75Config;
use esp_hub75::Hub75Error;
use hil as _;
use hil::support::Fixture;
use hil::support::bring_up;
use hil::support::bring_up_with;
use hil::target::CYCLES;
use hil::target::FrameBuffer;
use hil::target::RATE;
use hil::target::REFRESH_HZ;

// The panel geometry, the framebuffer type and the compile-time expectations
// on them live in `hil::target` (`src/target.rs`), next to the pin maps, so
// that all test files agree on them and none of them has to know which wiring
// it runs on.

// A fixed-size `core::fmt::Write` sink, so that `Display` can be exercised
// without an allocator (`alloc::string::String` is not in the image).
struct Sink {
    buf: [u8; 128],
    len: usize,
}

impl Sink {
    const fn new() -> Self {
        Self {
            buf: [0; 128],
            len: 0,
        }
    }

    fn as_str(&self) -> &str {
        // Every byte came from a `&str` handed to `write_str`, so the buffer is
        // always valid UTF-8; the fallback only matters if the sink itself is
        // broken, and an empty string is the safe answer there.
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

impl core::fmt::Write for Sink {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        if self.len + bytes.len() > self.buf.len() {
            return Err(core::fmt::Error);
        }
        self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();
        Ok(())
    }
}

#[embedded_test::tests]
mod tests {
    use super::*;

    /// No hardware involved: proves the image boots, the runner finds the test
    /// case table and reports a pass, and that the compile-time refresh model
    /// agrees with the runtime one.
    #[test]
    #[timeout(10)]
    fn smoke() {
        assert!(core::mem::size_of::<FrameBuffer>() > 0);
        assert_eq!(REFRESH_HZ, RATE.as_hz() / CYCLES as u32);
    }

    /// Constructs the driver on the selected chip's backend and completes one
    /// framebuffer swap.
    ///
    /// This is the whole hardware path in one test: `esp_hal::init`, the
    /// backend's peripheral + DMA setup, ISR start, and a `swap()`/`wait()`
    /// round trip that only returns once the ISR has reached a frame boundary.
    /// A hung DMA shows up as the test timeout, not as a false pass.
    #[test]
    #[timeout(30)]
    fn construct_and_swap() -> Result<(), &'static str> {
        let Fixture { hub75, spare, .. } = bring_up()?;

        // The driver owns the buffer it was constructed with, so only `spare`
        // -- the one about to be handed over -- can be written to.
        spare.erase();

        // Hand `spare` to the driver and get the previously displayed buffer
        // back. `wait()` only returns once the ISR has reached a frame
        // boundary and applied the swap, so a stalled DMA fails this test by
        // timing out rather than by silently passing.
        let previous = hub75
            .swap(spare)
            .map_err(|_| "swap() rejected while another swap was in flight")?
            .wait()
            .map_err(|_| "DMA transfer failed during the swap")?;

        // A stolen `Peripherals` proves the singleton guard rejects a second
        // driver. This is safe to do: `Hub75::new` checks the guard before it
        // touches the peripheral, and peripheral singletons have no `Drop`
        // impl that could disturb the running driver.
        let stolen = unsafe { esp_hal::peripherals::Peripherals::steal() };
        let second_pins = hil::hub75_pins!(stolen);
        let (second_peripheral, second_dma) = hil::hub75_backend!(stolen);
        let second_descriptors = esp_hub75::hub75_dma_descriptors!(FrameBuffer);

        let second = Hub75::new(
            second_peripheral,
            second_pins,
            second_dma,
            second_descriptors,
            Hub75Config::new().with_frequency(RATE),
            &*previous,
        );

        assert!(matches!(second, Err(Hub75Error::AlreadyInitialised)));

        Ok(())
    }

    /// No hardware involved: the config builders and the error type are pure
    /// data, so their contracts can be checked on a freshly reset chip without
    /// touching a peripheral.
    ///
    /// `Hub75Config`'s builders only write public fields, which makes a
    /// round trip through them exact -- and it is the *only* thing a test can
    /// check about a field that no getter reports. `Hub75Error` is what a
    /// caller matches on and prints, so its `PartialEq` has to separate the
    /// variants and its `Display` has to say something useful about each.
    #[test]
    #[timeout(10)]
    fn config_and_error_round_trips() -> Result<(), &'static str> {
        let frequency = Rate::from_khz(2_500);
        let priority = Priority::max();

        let config = Hub75Config::new()
            .with_frequency(frequency)
            .with_interrupt_priority(priority);
        if config.frequency.as_hz() != frequency.as_hz() {
            return Err("with_frequency() did not reach the frequency field");
        }
        if config.interrupt_priority != Some(priority) {
            return Err("with_interrupt_priority() did not reach the priority field");
        }

        // `Default` has to agree with `new()`: a driver built from the default
        // config and one built from `new()` must behave identically.
        let default = Hub75Config::default();
        if default.frequency.as_hz() != Hub75Config::new().frequency.as_hz() {
            return Err("Default disagrees with new() on the frequency");
        }
        if default.interrupt_priority.is_some() {
            return Err("the default configuration overrides the interrupt priority");
        }
        if Hub75Config::new()
            .with_frequency(frequency)
            .interrupt_priority
            .is_some()
        {
            return Err("with_frequency() also set the interrupt priority");
        }

        // The unit variants, in the order the checks below assume.
        let errors = [
            Hub75Error::NotInitialised,
            Hub75Error::AlreadyInitialised,
            Hub75Error::SwapInFlight,
            Hub75Error::AlreadyRunning,
        ];

        for (i, a) in errors.iter().enumerate() {
            for (j, b) in errors.iter().enumerate() {
                if (i == j) != (a == b) {
                    return Err("Hub75Error::eq does not separate the error variants");
                }
            }
        }

        let mut sinks = [Sink::new(), Sink::new(), Sink::new(), Sink::new()];
        for (sink, error) in sinks.iter_mut().zip(errors.iter()) {
            write!(sink, "{error}").map_err(|_| "Display for Hub75Error did not fit the sink")?;
        }
        for (i, sink) in sinks.iter().enumerate() {
            if sink.as_str().is_empty() {
                return Err("Display for Hub75Error produced an empty message");
            }
            for other in sinks.iter().skip(i + 1) {
                if sink.as_str() == other.as_str() {
                    return Err("two Hub75Error variants Display the same message");
                }
            }
        }

        // The two errors the swap lifecycle hands back are the ones a caller
        // acts on, so their text has to name the call that was refused and the
        // call that unblocks it.
        if !sinks[2].as_str().contains("swap") {
            return Err("the SwapInFlight message does not name the failed call");
        }
        if !sinks[3].as_str().contains("wait") {
            return Err("the AlreadyRunning message does not name the recovery");
        }

        Ok(())
    }

    /// The documented anti-flicker lever, on silicon: constructs the driver
    /// with the refresh ISR at the highest priority the chip offers.
    ///
    /// `interrupt_priority` is the one [`Hub75Config`] field whose value does
    /// not end up in a peripheral register -- it reaches esp-hal's interrupt
    /// binding, which is chip-specific, so a mis-set priority is a failure mode
    /// that only shows up when the handler actually fires. The swap below is
    /// what makes that happen.
    #[test]
    #[timeout(30)]
    fn construct_with_raised_isr_priority_and_swap() -> Result<(), &'static str> {
        let Fixture { hub75, spare, .. } = bring_up_with(
            Hub75Config::new()
                .with_frequency(RATE)
                .with_interrupt_priority(Priority::max()),
        )?;

        // The driver owns the buffer it was constructed with, so only `spare`
        // -- the one about to be handed over -- can be written to.
        spare.erase();

        hub75
            .swap(spare)
            .map_err(|_| "swap() was rejected with the ISR at maximum priority")?
            .wait()
            .map_err(|_| "the transfer failed with the ISR at maximum priority")?;

        Ok(())
    }
}
