//! Shared bring-up for test files that drive the blocking API.
//!
//! The steps are always the same -- take `Peripherals`, place the framebuffers
//! and DMA descriptors in statics, build the pin map for the selected wiring,
//! construct the driver with one buffer displayed -- and only the assertions
//! differ between files. Keeping that here means a test file only has to know
//! [`Fixture`], not the bus width, the chip's backend or the pin macros.
//!
//! [`bring_up`] may be called **once per test case**, not once per file: the
//! framebuffers live in `StaticCell`s that panic on a second write. That works
//! because `embedded-test` resets the chip -- and with it `.bss` -- before each
//! case, which `tests/reset.rs` proves.

use core::ptr;

use esp_hal::clock::CpuClock;
use esp_hub75::Hub75;
use esp_hub75::Hub75Config;
use esp_hub75::Hub75Error;

use crate::target::FrameBuffer;
use crate::target::RATE;

/// A running driver plus the framebuffers a swap test needs.
///
/// Three buffers is the floor, not padding: the driver starts out owning one
/// (`fb0`, of which only the address is kept here), and the `restart()` test
/// needs both of the others -- the pending swap owns the buffer it was handed,
/// so it cannot be named again, and `restart()` takes a `&'static` reference,
/// which borrows whichever buffer is offered to it for the rest of the test.
///
/// A framebuffer is 4096 bytes on the 16-bit wiring and 2304 on the latched
/// one, so this fixture costs 12 KiB / 6.75 KiB of `.bss`. Keep it at two
/// spares unless a test genuinely needs more.
pub struct Fixture {
    /// The driver handle. `Hub75` is zero-sized; the state lives in the ISR
    /// statics, so this is `!Sync` and single-owner by construction.
    pub hub75: Hub75<esp_hal::Blocking, FrameBuffer>,
    /// Address of the framebuffer the driver is displaying at construction.
    pub displayed: *const FrameBuffer,
    /// A buffer that can be handed to `swap()`.
    pub spare: &'static mut FrameBuffer,
    /// A second spare, for a test that needs a free buffer while one is in
    /// flight: a rejected `swap()` hands the buffer it was given straight back,
    /// and the `restart()` test needs a buffer to offer that it is not already
    /// using.
    pub spare2: &'static mut FrameBuffer,
}

/// Constructs the driver with `fb0` displayed and two spare framebuffers.
///
/// Uses the default configuration: [`RATE`] and the peripheral's own interrupt
/// priority. See [`bring_up_with`] for a fixture that hands
/// [`Hub75::new`](esp_hub75::Hub75::new) a caller-supplied [`Hub75Config`].
///
/// The error strings are written for the harness rather than for the driver:
/// within a test case the only likely construction failure is a driver that
/// survived from a previous case, so that case is called out by name.
///
/// See the module docs about calling this once per test case.
pub fn bring_up() -> Result<Fixture, &'static str> {
    bring_up_with(Hub75Config::new().with_frequency(RATE))
}

/// [`bring_up`], with the [`Hub75Config`] chosen by the caller.
///
/// Same fixture, same error handling -- only the configuration handed to
/// `Hub75::new` differs, so a test can push a config builder through a real
/// construction (`with_interrupt_priority`, say) instead of only checking the
/// field it set.
///
/// See [`bring_up`] about calling this once per test case.
pub fn bring_up_with(config: Hub75Config) -> Result<Fixture, &'static str> {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    let fb0 = crate::mk_static!(FrameBuffer, FrameBuffer::new());
    let spare = crate::mk_static!(FrameBuffer, FrameBuffer::new());
    let spare2 = crate::mk_static!(FrameBuffer, FrameBuffer::new());
    let tx_descriptors = esp_hub75::hub75_dma_descriptors!(FrameBuffer);

    let displayed = ptr::from_ref(&*fb0);

    let pins = crate::hub75_pins!(peripherals);
    let (peripheral, dma) = crate::hub75_backend!(peripherals);

    // Latched boards scale panel brightness through an OE pin that is not
    // part of the driver's pin set; drive it high (full on) exactly like the
    // `gradient-latched` example. A direct-drive board has no such pin, where
    // the macro expands to `()`.
    let _oe = crate::oe_pin!(peripherals);

    let hub75 = Hub75::new(peripheral, pins, dma, tx_descriptors, config, &*fb0).map_err(|err| {
        match err {
            Hub75Error::AlreadyInitialised => {
                "Hub75::new() found an existing driver: the previous test case's state survived the reset"
            }
            _ => "Hub75::new() failed on a freshly reset chip",
        }
    })?;

    Ok(Fixture {
        hub75,
        displayed,
        spare,
        spare2,
    })
}

/// An async-mode driver plus the framebuffers an async swap test needs.
///
/// Same shape as [`Fixture`], minus the second spare: the async tests ping-pong
/// the pair back and forth, and there is no `restart()` variant of the async
/// test to need a buffer that is free while one is in flight.
pub struct AsyncFixture {
    /// The driver handle, in its [`esp_hal::Async`] form.
    pub hub75: Hub75<esp_hal::Async, FrameBuffer>,
    /// Address of the framebuffer the driver is displaying at construction.
    pub displayed: *const FrameBuffer,
    /// A buffer that can be handed to `swap()`.
    pub spare: &'static mut FrameBuffer,
}

/// Constructs the driver through [`Hub75::new_async`].
///
/// The arguments are identical to [`bring_up`]'s -- only the constructor and
/// the resulting handle type differ. Kept as a separate function rather than
/// folded into a generic one because the pin, backend and channel types are
/// all chip- and bus-dependent; naming them here would cost more than the few
/// lines it saves.
///
/// See [`bring_up`] about calling this once per test case.
///
/// [`Hub75::new_async`]: esp_hub75::Hub75::new_async
pub fn bring_up_async() -> Result<AsyncFixture, &'static str> {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    let fb0 = crate::mk_static!(FrameBuffer, FrameBuffer::new());
    let spare = crate::mk_static!(FrameBuffer, FrameBuffer::new());
    let tx_descriptors = esp_hub75::hub75_dma_descriptors!(FrameBuffer);

    let displayed = ptr::from_ref(&*fb0);

    let pins = crate::hub75_pins!(peripherals);
    let (peripheral, dma) = crate::hub75_backend!(peripherals);

    // See `bring_up` about the latched board's separate brightness pin.
    let _oe = crate::oe_pin!(peripherals);

    let hub75 = Hub75::new_async(
        peripheral,
        pins,
        dma,
        tx_descriptors,
        Hub75Config::new().with_frequency(RATE),
        &*fb0,
    )
    .map_err(|err| match err {
        Hub75Error::AlreadyInitialised => {
            "Hub75::new_async() found an existing driver: the previous test case's state survived the reset"
        }
        _ => "Hub75::new_async() failed on a freshly reset chip",
    })?;

    Ok(AsyncFixture {
        hub75,
        displayed,
        spare,
    })
}
