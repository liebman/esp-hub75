//! Shared support code for the esp-hub75 hardware-in-the-loop (HIL) tests.
//!
//! Every test binary links this crate (`use hil as _;`) so that the pieces
//! below exist exactly once per image:
//!
//! * the ESP app descriptor that the flasher looks for,
//! * a defmt logger (a null one by default -- see the `defmt` feature),
//! * the boot-time watchdog setup that `embedded-test` runs before it starts
//!   taking to the debug probe over semihosting.

#![no_std]

pub mod pins;
pub mod support;
pub mod target;

// ---------------------------------------------------------------------------
// Static allocation
// ---------------------------------------------------------------------------

/// Places `$val` in a `'static` cell and hands back the `&'static mut`
/// reference.
///
/// Framebuffers, DMA descriptors and pin sets all have to outlive the test body
/// (the driver keeps pointers into the framebuffer), so they cannot live on the
/// stack. `StaticCell` is an `unsafe`-free way to do that: it hands out the
/// reference exactly once and panics on a second `write`, which is the same
/// lifetime the driver wants.
///
/// Exported so that every test binary shares one definition instead of
/// re-declaring it.
#[macro_export]
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

// ---------------------------------------------------------------------------
// Configuration guards
// ---------------------------------------------------------------------------
//
// The bus width and the chip are both compile-time choices that `pins.rs` and
// `target.rs` implement with `#[cfg]`. Checking the feature set here first
// makes a wrong one fail with an explanation instead of an undefined macro or
// a trait error.

#[cfg(all(feature = "bus8", feature = "bus16"))]
compile_error!(
    "both bus widths are enabled; enable exactly one of: `bus16` (16-bit direct drive), `bus8` \
     (8-bit latched). The `-8` aliases pass `--no-default-features` to turn `bus16` off."
);

#[cfg(not(any(feature = "bus8", feature = "bus16")))]
compile_error!(
    "no bus width selected; enable exactly one of: `bus16` (16-bit direct drive), `bus8` (8-bit \
     latched)"
);

// The ESP32-C5's `PARL_IO` has no 16-bit parallel mode: esp-hal's C5 `PARL_IO`
// has no `TxSixteenBits` pin set, and the driver gates both of its
// `Hub75Pins16` backends on `not(esp32c5)`.
#[cfg(all(feature = "esp32c5", feature = "bus16"))]
compile_error!(
    "esp32c5 cannot drive a 16-bit panel; use `bus8` (8-bit latched) -- the chip's PARL_IO has no \
     16-bit parallel mode"
);

#[cfg(any(
    all(feature = "esp32", feature = "esp32s3"),
    all(feature = "esp32", feature = "esp32c6"),
    all(feature = "esp32", feature = "esp32c5"),
    all(feature = "esp32s3", feature = "esp32c6"),
    all(feature = "esp32s3", feature = "esp32c5"),
    all(feature = "esp32c6", feature = "esp32c5")
))]
compile_error!(
    "multiple chip features enabled; enable exactly one of: `esp32`, `esp32s3`, `esp32c6`, \
     `esp32c5`"
);

#[cfg(not(any(
    feature = "esp32",
    feature = "esp32s3",
    feature = "esp32c6",
    feature = "esp32c5"
)))]
compile_error!("no chip selected; enable exactly one of: `esp32`, `esp32s3`, `esp32c6`, `esp32c5`");

esp_bootloader_esp_idf::esp_app_desc!();

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------

// By default nothing is logged. `probe-rs` polls RTT by halting a core, which
// perturbs exactly the DMA/refresh timing these tests measure, and the RTT
// buffers cost RAM. The `defmt` feature swaps this null logger for
// `defmt-rtt` so a failing test can be investigated.
#[cfg(not(feature = "defmt"))]
#[defmt::global_logger]
struct Logger;

#[cfg(not(feature = "defmt"))]
unsafe impl defmt::Logger for Logger {
    fn acquire() {}
    unsafe fn flush() {}
    unsafe fn release() {}
    unsafe fn write(_bytes: &[u8]) {}
}

#[cfg(feature = "defmt")]
use defmt_rtt as _;

// ---------------------------------------------------------------------------
// Watchdogs
// ---------------------------------------------------------------------------

/// Disables every watchdog timer before `embedded-test` starts.
///
/// Without this the chip resets mid-run -- most often once a core is halted by
/// the debugger, or while a test leaves the CPU in a spin loop -- and the
/// runner reports a lost target instead of a test result.
///
/// The writes are raw register accesses on purpose: a half-initialized or
/// broken peripheral driver must not be able to stop the test suite from
/// running, and `::regs()` does not consume the peripheral singletons, so the
/// tests can still take `TIMG0`, `LP_WDT`, ... from `Peripherals` afterwards.
/// All writes are register-granular to stay independent of generated field
/// names.
#[embedded_test::setup]
fn disable_watchdogs_before_semihosting() {
    // The RTC watchdog lives in `LP_WDT` on chips where it owns a register
    // block, and is reached through `LPWR` on the older parts.
    #[cfg(soc_has_lp_wdt)]
    use esp_hal::peripherals::LP_WDT as RtcWdt;
    #[cfg(not(soc_has_lp_wdt))]
    use esp_hal::peripherals::LPWR as RtcWdt;

    // ---- RWDT: unlock, clear the config register (disabling it), relock.
    {
        let lp_wdt = RtcWdt::regs();
        lp_wdt
            .wdtwprotect()
            .write(|w| unsafe { w.bits(0x50D8_3AA1) });
        lp_wdt.wdtconfig0().write(|w| unsafe { w.bits(0) });
        lp_wdt.wdtwprotect().write(|w| unsafe { w.bits(0) });
    }

    // ---- SWD (super watchdog): unlock, let it feed itself, relock.
    #[cfg(soc_has_swd_watchdog)]
    {
        #[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))]
        const SWD_WKEY: u32 = 0x8F1D_312A;
        #[cfg(not(any(esp32c2, esp32c3, esp32s2, esp32s3)))]
        const SWD_WKEY: u32 = 0x50D8_3AA1;

        let lp_wdt = RtcWdt::regs();
        lp_wdt
            .swd_wprotect()
            .write(|w| unsafe { w.swd_wkey().bits(SWD_WKEY) });
        lp_wdt.swd_conf().write(|w| w.swd_auto_feed_en().set_bit());
        lp_wdt
            .swd_wprotect()
            .write(|w| unsafe { w.swd_wkey().bits(0) });
    }

    // ---- MWDT: the timer-group watchdogs, one register block each.
    #[cfg(timergroup_timg0)]
    {
        let timg0 = esp_hal::peripherals::TIMG0::regs();
        timg0
            .wdtwprotect()
            .write(|w| unsafe { w.wdt_wkey().bits(0x50D8_3AA1) });
        timg0.wdtconfig0().modify(|_, w| w.wdt_en().clear_bit());
        timg0
            .wdtwprotect()
            .write(|w| unsafe { w.wdt_wkey().bits(0) });
    }

    #[cfg(timergroup_timg1)]
    {
        let timg1 = esp_hal::peripherals::TIMG1::regs();
        timg1
            .wdtwprotect()
            .write(|w| unsafe { w.wdt_wkey().bits(0x50D8_3AA1) });
        timg1.wdtconfig0().modify(|_, w| w.wdt_en().clear_bit());
        timg1
            .wdtwprotect()
            .write(|w| unsafe { w.wdt_wkey().bits(0) });
    }
}
