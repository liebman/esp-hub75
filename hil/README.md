# esp-hub75 HIL tests

On-target (hardware-in-the-loop) tests for `esp-hub75`. They run on a real
chip: the image is flashed, booted and its results are reported back over
semihosting by [`embedded-test`] and [`probe-rs`]. There is no host-side
simulation of the parallel DMA path, so a board on a bench is the only way to
exercise it.

## Chip and bus width matrix

The same tests run against both panel wirings and every supported backend. A
**chip** feature picks the pin map and the parallel-output peripheral; a **bus
width** feature picks the wiring:

* `bus16` (the default) -- 16-bit direct drive, `gradient`-style wiring,
* `bus8` -- 8-bit latched, `gradient-latched`/SmartLEDShield-style wiring.

| Chip | 16-bit direct drive | 8-bit latched |
| --- | --- | --- |
| ESP32-S3 | `check-esp32s3` | `check-esp32s3-8` |
| ESP32 | `check-esp32` | `check-esp32-8` |
| ESP32-C6 | `check-esp32c6` | `check-esp32c6-8` |
| ESP32-C5 | *not supported by the driver* | `check-esp32c5-8` |

Each `check-` alias has a `test-` counterpart that flashes and runs it. ESP32-C5
is 8-bit only: its `PARL_IO` has no 16-bit parallel mode and the driver gates
its 16-bit backends on `not(esp32c5)`, so `esp32c5` with `bus16` is a
`compile_error!`.

Those aliases are the whole matrix, so there is nothing else to know:
compile-only needs no hardware, and `test-` needs the matching board on the
probe. When several boards are attached, a run has to name the probe as well as
the chip: [`run.sh`](#running) does that, and is the supported way to run the
suite. The aliases stay useful for a single board, where there is nothing to
choose.

## Running

```console
$ cd hil
$ cargo test-esp32s3                 # 16-bit direct drive (the validated board)
$ cargo test-esp32s3-8               # 8-bit latched, same tests
$ cargo test-esp32s3 --test construct -- --nocapture
$ cargo test-esp32s3 --test construct -- smoke   # a single test
```

`hil/run.sh` is how to run more than one thing, and the only way to run with more
than one board attached:

```console
$ ./run.sh 16 --list                   # resolve the bench + matrix, run nothing
$ ./run.sh 16                          # the whole matrix on the attached board
$ ./run.sh 16 --test lifecycle async_swap --mode full-chain-dma circular-dma
$ ./run.sh 8 --compile-only --chip esp32c6   # no board needed
```

It detects the chip with `probe-rs info`, refuses to run when the detected chip
disagrees with `--chip`, picks the probe (`--probe`/`--probe-index`, or the only
one attached), exports `PROBE_RS_PROBE`/`PROBE_RS_NON_INTERACTIVE` for the run,
skips `circular-dma` on the C6 with a printed reason, treats a run that printed no
`test result:` line as a runner failure and retries it once, and ends with a
per-run table. `--help` lists the flags; `--list` is the dry run.

`<width>` is the one thing it cannot detect: it describes the *wiring*, and
either wiring boots and passes on the wrong panel, so pass the one that is
plugged in. Probe IDs are printed per run and never recorded -- see section 10 of
[`PLAN.md`](PLAN.md).

Compile-only checks need no hardware:

```console
$ cargo check-esp32s3          # fastest syntax/type check of the test binaries
$ cargo test-esp32s3 --no-run  # links the ELFs: also validates the linker scripts

# all 7 chip x bus-width combinations, compile-only:
$ for alias in check-esp32 check-esp32-8 check-esp32s3 check-esp32s3-8 \
      check-esp32c6 check-esp32c6-8 check-esp32c5-8; do cargo "$alias"; done
```

Every alias is `<verb>-<chip>[-8]`; the chip and the bus width are also cargo
features, so the underlying commands are

```console
$ cargo test --release --config .cargo/config-<chip>.toml --features <chip>
$ cargo test --release --config .cargo/config-<chip>.toml \
    --no-default-features --features <chip>,bus8
```

`bus16` is the default feature, which is why the `-8` aliases have to turn the
defaults off.

## Adding a test

* `Hub75` is a process-wide singleton (its state lives in ISR statics) and
  `embedded-test` resets the chip between test cases, so **each test case may
  call `bring_up()` once**. Several test cases per file are fine -- with the
  reset between them the driver is genuinely fresh (see `tests/lifecycle.rs`);
  two constructions inside a *single* test case would panic on the second
  `StaticCell` write instead. A test that needs more than one driver for itself
  goes in its own `tests/*.rs` file (add the matching `[[test]]` entry with
  `harness = false`). `bring_up_with(config)` is the same fixture with a
  caller-chosen `Hub75Config`, for a test that has to push a builder through a
  real construction.
* `defmt` logging is off by default (see below), so report failures with
  `Result<(), &'static str>`, `#[should_panic]` or a `#[timeout(..)]` that
  trips, and keep the message in the return value.
* Prefer assertions that fail at compile time (`const _: () = assert!(..)`) for
  anything that is a property of a *type*, and `#[test]`s for behaviour that
  depends on a feature flag actually reaching the driver.
* Give anything that touches hardware a `#[timeout(..)]`: the runner then
  reports a timeout instead of hanging forever on a stalled DMA.
* Keep a test file bus-width agnostic: take the framebuffer type, the geometry
  and the refresh expectations from `hil::target` and the wiring from
  `hil::hub75_pins!`/`hil::hub75_backend!`/`hil::oe_pin!`, instead of naming
  `Hub75Pins16`/`Hub75Pins8` or a framebuffer family directly. A test that
  names one wiring is a test that silently skips the other.

## Layout

| Path | Purpose |
| --- | --- |
| `src/lib.rs` | App descriptor, defmt logger, the `#[embedded_test::setup]` watchdog hook and the feature-set guards. Linked by every test binary via `use hil as _;`. |
| `src/pins.rs` | Per-chip HUB75 pin maps, backend (peripheral + DMA channel) selection and the latched boards' OE/brightness pin. |
| `src/target.rs` | Panel geometry, the bus-width-dependent framebuffer type, the refresh model and the compile-time expectations on it. |
| `src/support.rs` | The shared fixture: pin map, backend, DMA descriptors, `esp_hal::init` and the one allowed `Hub75::new`, exposed as `bring_up()` (blocking, default config), `bring_up_with(config)` and `bring_up_async()`, plus the framebuffers a swap test needs. |
| `tests/construct.rs` | Bring-up: harness smoke test, one construct + `swap()`/`wait()` round trip, the `Hub75Config`/`Hub75Error` round trips (no hardware) and one construction with the refresh ISR at maximum priority. |
| `tests/reset.rs` | The reset canary: two test cases that each need a freshly initialized chip, so a per-test reset that quietly does not happen fails here first. |
| `tests/lifecycle.rs` | The swap contract: which buffer `wait()` returns, `SwapInFlight` and its handed-back buffer, `restart()` refused while a transfer is in flight and recovery after the refusal (that last case is `#[cfg]`-ed out under `circular-dma`, where the ring never stops). |
| `tests/stress.rs` | A few hundred swaps alternating two identity-checked buffers, under a wall-clock ceiling derived from the refresh model. |
| `tests/async_swap.rs` | The async half: `wait_for_done()` is woken by the ISR, and stays woken across successive swaps. Runs on a local flag-waker executor so that a lost wakeup is a `#[timeout]` failure rather than a slow pass. |
| `tests/refresh.rs` | The refresh rate on silicon: 200 frame boundaries timed through the swap path and compared against the compile-time model, per DMA mode (the group-based default really is ~5% slower than `full-chain-dma`/`circular-dma`; the numbers are in the file). |
| `run.sh` | The runner: bench detection (chip from the probe, wiring from the argument), the test/mode matrix, per-chip exclusions, the one-retry runner-failure handling and the summary table. |
| `.cargo/config-<chip>.toml` | Target, `probe-rs` runner and the embedded-test/defmt linker flags. |
| `PLAN.md` | The test plan these tests implement, annotated with what has landed, what is still open, and the bench/probe question. |

## Notes that are easy to get wrong

* **Watchdogs.** `embedded-test` runs `disable_watchdogs_before_semihosting`
  before the first test. It writes registers directly rather than going through
  drivers: a driver that is half-initialized must not be able to stop the suite,
  and `::regs()` does not consume the peripheral singletons the tests need.
* **Each test case starts from a chip reset, and the reset is load-bearing.**
  `embedded-test` resets the chip between test cases, which is what lets two
  tests in one file each call `bring_up()`: the driver's statics, the DMA
  registers and the peripheral state are all re-initialized from scratch. That
  also means `bring_up()`/`bring_up_async()` may be called **once per test
  case**, not once per call site -- they write `StaticCell`s, and a second call
  without an intervening reset panics rather than returning an error.
  `tests/reset.rs` is the canary for both halves of that claim.
* **No RTT polling by default.** `probe-rs` polls RTT by halting a core, which
  perturbs DMA/refresh timing (and costs RAM), so the default build installs a
  null `#[defmt::global_logger]`. Use `--features defmt` only while debugging a
  failing test; never for timings. The reference numbers in `tests/stress.rs` and
  `tests/refresh.rs` were taken with RTT *on* (noted where they are used), which
  is why the bands around them carry deliberate margin.
* **Xtensa needs `-nostartfiles`** together with `embedded-test/xtensa-semihosting`,
  and the RISC-V parts must *not* set it. The arch-specific flag sets are in the
  per-chip config files.
* **`--cfg embedded_test`** is set because the esp-hal HIL configuration sets
  it; nothing in the dependency tree currently reads it.
* **The bus width is a type, not a feature of the driver.** Both framebuffer
  families are compiled unconditionally, so covering both wirings costs no
  extra dependencies: 16-bit direct drive is `Hub75Pins16` (word `u16`) plus
  `plain::DmaFrameBuffer`, 8-bit latched is `Hub75Pins8` (word `u8`) plus
  `latched::DmaFrameBuffer`. The driver's bounds tie the pair together, so a
  mismatched combination is a compile error rather than a wrong-looking panel.
* `probe-rs` autodetects the chip from the ELF/probe, but each config passes
  `--chip` explicitly to avoid surprises when several boards are attached. That is
  only half the job: `--chip` and `--probe` are independent axes, so with more
  than one probe connected the run also has to say *which probe*, and `run.sh`
  does that for you (and refuses to continue when the probe's chip and `--chip`
  disagree). Doing it by hand means passing a *full* `PROBE_RS_PROBE=vid:pid:serial`
  -- a bare serial is rejected by probe-rs -- together with
  `PROBE_RS_NON_INTERACTIVE=true`, so that an unbound bench fails instead of
  prompting. That variable is parsed as a bool, so `=1` does not work; the `[env]`
  default in `.cargo/config.toml` covers the plain aliases either way (see section
  10 of [`PLAN.md`](PLAN.md)).
* **Probe IDs are logged per run and never recorded.** USB-Serial-JTAG gives every
  board its own ID, and two boards of the same chip differ only in that ID, so a
  recorded one is bound to whatever is on the bench that day -- the serial that
  used to be written down here is not the one the probe reports now. The chip is
  the identity the tests and the tooling use; the ID only tells two attached boards
  apart *right now*, which is why `run.sh` prints it and stores nothing.

## Board support

| Chip | 16-bit direct drive | 8-bit latched | Status |
| --- | --- | --- | --- |
| ESP32-S3 | `LCD_CAM`/`DMA_CH0` | `LCD_CAM`/`DMA_CH0` | validated on hardware: the whole suite (6 binaries, 13 tests) is green on both wirings in all three DMA modes -- `lifecycle` reports 2 rather than 3 tests under `circular-dma`, where its refused-`restart()` case is `#[cfg]`-ed out because the ring never stops |
| ESP32 | `I2S0`/`DMA_I2S0` | `I2S1`/`DMA_I2S1` | compiles; pin maps copied from the examples, needs a board |
| ESP32-C6 | `PARL_IO`/`DMA_CH0` | `PARL_IO`/`DMA_CH0` | compiles; pin maps copied from the examples, needs a board |
| ESP32-C5 | *not supported by the driver* | `PARL_IO`/`DMA_CH0` | compiles; pin map copied from the example, needs a board |

The ESP32 needs both I2S peripherals: `I2S0` for the 16-bit wiring (the only
16-bit-capable parallel peripheral) and `I2S1` for the 8-bit one, because
`I2S0` drops every odd byte.

ESP32 has only one JTAG-capable setup (external probe) and no USB-Serial-JTAG;
`probe-rs` needs the external probe's configuration, so its alias is expected to
need a `--chip esp32` probe setup before it can run.

[`embedded-test`]: https://github.com/probe-rs/embedded-test
[`probe-rs`]: https://probe.rs
