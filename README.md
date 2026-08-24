# ESP-HUB75

[![Crates.io](https://img.shields.io/crates/v/esp-hub75.svg)](https://crates.io/crates/esp-hub75)
[![Documentation](https://docs.rs/esp-hub75/badge.svg)](https://docs.rs/esp-hub75)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](README.md)

A `no-std` driver for HUB75-style LED matrix panels on ESP32-series
microcontrollers.

The panel is refreshed over DMA with almost no CPU involvement, using
whichever peripheral fits each chip best:

- **ESP32-S3**: Uses the LCD_CAM peripheral
- **ESP32-C6**: Uses the PARL_IO peripheral
- **ESP32-C5**: Uses the PARL_IO peripheral (8-bit mode only; requires a
  latch circuit and `Hub75Pins8`)
- **ESP32**: Uses the I2S peripheral in parallel mode

The framebuffers work with `embedded-graphics`, so you can draw shapes, text,
and images on the display, and they go to the DMA engine as-is; there is no
per-frame conversion step.

## Framebuffers

The `hub75-framebuffer` crate has two families of framebuffer: **standard**
and **bitplane**. Each family has a direct-drive variant (16-bit, no external
latch) and a latched variant (8-bit, needs an external address-latch
circuit). Both can be handed to the peripheral as-is; there is no extra
formatting step. They differ in how they do Binary Code Modulation (BCM):

- **Standard** framebuffers (`framebuffer::plain::DmaFrameBuffer` /
  `framebuffer::latched::DmaFrameBuffer`) pre-render a complete copy of the
  pixel data for every BCM bit-weight. DMA output is straightforward, but
  memory use multiplies by the number of frames (`frame_count`).
- **Bitplane** framebuffers (`framebuffer::bitplane::plain::DmaFrameBuffer` /
  `framebuffer::bitplane::latched::DmaFrameBuffer`) store only one bit per
  pixel per plane. The driver assembles the BCM output on the fly with DMA
  descriptors: same visual quality, much less RAM.

Unless you have a reason not to, use the bitplane framebuffers.

## Hardware Requirements

- An ESP32-series microcontroller (ESP32, ESP32-S3, ESP32-C5, or ESP32-C6)
- A HUB75 LED matrix panel
- A 5V power supply capable of providing several amps of current
- A 3.3V to 5V level shifter (e.g., 74HCT245) is highly recommended

**Note**: The ESP32 is a 3.3V chip but HUB75 panels expect 5V logic. It can
work without a level shifter, but don't count on it.

## Pin Configurations

Two pin configurations are supported, depending on your hardware:

- **`Hub75Pins16` (Direct Drive)**: the standard configuration, where the row
  address lines go out with every pixel. Needs more GPIO pins but works with
  any standard HUB75 panel.

- **`Hub75Pins8` (Latched)**: for controller boards with an external
  74HC574-style latch on the row address lines. Uses fewer GPIO pins and less
  memory. For more details on the
  required circuit, see the [`hub75-framebuffer` crate's documentation](https://crates.io/crates/hub75-framebuffer)
  or its [GitHub repository](https://github.com/liebman/hub75-framebuffer).

## Examples

The examples are standalone crates in the [`examples/`](examples) directory,
organized by display setup rather than by chip. Each one supports several
boards; pick a board and it builds for the right chip automatically.

- [`examples/gradient`](examples/gradient) - Blocking (non-async) 64x64
  color-gradient demo with refresh/render stats, using a 16-bit bitplane
  framebuffer (direct drive, no latch). For ESP32, ESP32-S3, ESP32-C6, and
  ESP32-Trinity.
- [`examples/gradient-embassy`](examples/gradient-embassy) - Embassy (async)
  variant of `gradient`. Same boards.
- [`examples/gradient-latched`](examples/gradient-latched) - Blocking 64x64
  color-gradient demo using an 8-bit latched bitplane framebuffer for
  controller boards with a SmartLEDShield-style address latch. For ESP32,
  ESP32-S3, ESP32-C6, and ESP32-C5.
- [`examples/gradient-quarter`](examples/gradient-quarter) - Blocking demo
  for quarter-scan (1/16-scan) 64x64 panels, using the `QuarterScan`
  remapper. Same boards as `gradient`.
- [`examples/gradient-tiled`](examples/gradient-tiled) - Blocking demo that
  tiles four 64x32 panels (2x2) into one 128x64 virtual canvas. Same boards
  as `gradient`.

**Note**: The ESP32-C5 does not support 16-bit mode, so only the latched
example (`gradient-latched`) can be used with it.

### Running the examples

Each example defines its own cargo aliases in its `.cargo/config.toml`; see
the `[alias]` section there for the full list. There are `run-*`, `build-*`,
and `clippy-*` aliases per board (e.g. `run-esp32`, `run-esp32s3`,
`run-esp32c6`), and the `run-*` aliases build, flash with `espflash`, and
monitor in one step:

    cd examples/gradient
    cargo run-esp32

Extra features can be appended to any alias with `-F`/`--features`:

    cargo run-esp32 -F circular-dma,trail-blank-2

The features each example accepts (board selection and pass-throughs for
every `esp-hub75` feature) are listed in its `Cargo.toml`; see
[Crate Features](#crate-features) for what the driver features do. Two
example-only features are available on top of those:

- `row`: Use the row-major framebuffer layout (all bit-planes of a row
  stored contiguously) instead of the default plane-major layout.
- `20mhz`: Drive the panel at 20 MHz instead of the default 10 MHz for a
  higher refresh rate. The ESP32's I2S peripheral tops out at 19 MHz, which
  the examples handle automatically.

**Note**: Other than `gradient-latched`, the examples use plain
(direct-drive) framebuffers, which by default insert no blanking before or
after the row address change. Adding at least `trail-blank-2` is recommended
to avoid row ghosting. The latched framebuffers have this blanking built in
(1 clock before and 2 clocks after the address change) due to the
framebuffer structure.

## Crate Features

- `esp32`: Enable support for the ESP32
- `esp32s3`: Enable support for the ESP32-S3
- `esp32c5`: Enable support for the ESP32-C5
- `esp32c6`: Enable support for the ESP32-C6
- `defmt`: Enable logging with `defmt`
- `log`: Enable logging with the `log` crate
- `invert-blank`: Invert the blank signal in hardware by enabling the GPIO
  output inverter on the blank pin. Applies to both 8-bit latched
  (`Hub75Pins8`) and 16-bit direct-drive (`Hub75Pins16`) configurations. Some
  latch controller boards include a hardware inverter on the blank line; enable
  this feature to compensate.
- `invert-clock`: Invert the clock signal. By default the driver outputs data
  that changes on the falling edge of CLK so that it is stable when the panel
  latches on the rising edge. Enable this feature if your panel requires the
  opposite polarity.
- `invert-oe`: Forwards to the `hub75-framebuffer` crate, inverting the
  output-enable (OE) signal in the generated data stream. Whereas `invert-blank`
  inverts the blank pin in hardware, this feature flips the OE polarity at the
  framebuffer level instead. The two features may seem redundant but are meant
  to be used together: where the peripheral drives all pins to 0 when a
  transfer completes, `invert-blank` turns that idle 0 into a 1 (blanked), and
  `invert-oe` compensates for the now-inverted pin.
- `full-chain-dma`: Build the entire BCM repetition chain in a single DMA
  transfer instead of one plane per interrupt. This reduces interrupt frequency
  at the cost of more DMA descriptor RAM. Note that the ESP32-C6 PARL_IO
  peripheral has a 65 535-byte per-transfer limit, which constrains the maximum
  panel size and plane count when this feature is enabled.
- `circular-dma`: Circular DMA descriptor chain (implies `full-chain-dma`).
  The DMA engine starts once and loops forever; buffer swaps are instant
  pointer-delta updates with no DMA stop/restart. A frame-boundary ISR is
  always active in this mode, providing both `frame_count()` and the
  completion signal for `Hub75Swap::wait()` / `Hub75Swap::wait_for_done()`.
  Only supported on ESP32 and ESP32-S3; on ESP32-C5/C6 this is a compile-time
  error because PARL_IO cannot do circular chains.
- `skip-black-pixels`: Forwards to the `hub75-framebuffer` crate, enabling an
  optimization that skips writing black pixels to the framebuffer.
- `tail-closes-latch`: Forwards to the `hub75-framebuffer` crate. Applies to
  `plain` and `bitplane::plain` framebuffers only (not the latched variants).
  Appends a single extra "tail" word at the end of each DMA buffer (`plain`)
  or at the end of each bit-plane (`bitplane::plain`) that parks the bus with
  LATCH=0 and OE=BLANK, cleanly terminating the transfer.
- `iram`: Place the driver's hot path (render / DMA wait functions) in
  Instruction RAM (IRAM) to avoid flash-cache stalls (for example during
  Wi-Fi, PSRAM, or SPI-flash activity) that can cause visible flicker.
  Costs roughly 5-10 KiB of IRAM.
- `lead-blank-1/2/4/8/16` / `trail-blank-1/2/4/8/16`: Forwards to
  `hub75-framebuffer`. Control the number of pixel-clock cycles of blanking
  (OE HIGH) inserted around row address changes. The lead blank controls
  blanking *before* the address change, and the trail blank controls blanking
  *after*. Higher values reduce ghosting at the cost of slightly less
  brightness.
- `inter-row-blank-4/8/16/32`: Forwards to `hub75-framebuffer`. Insert
  additional dead clock cycles at the end of each row. In plain framebuffers
  the gap defers the address change to the first pixel of the next row, giving
  slow panels more time to finish blanking. In latched framebuffers the gap
  adds extra blanked cycles after the address change.
- `reverse-row-order`: Forwards to `hub75-framebuffer`. Stores the rows of the
  framebuffer in reverse scan order so that the DMA stream renders the last
  panel row first and row 0 last.

## Known Working Panels

This library should work with any "normal" RGB matrix panels. The following
panels have been tested and confirmed to work:

| Panel | Scan Rate | Column Driver | Row Driver |
|-------|-----------|---------------|------------|
| Waveshare RGB-Matrix-P3-64x64 | 1/32 | SM5166 | SM16208 |
| Waveshare RGB-Matrix-P3-64x32 | 1/16 | ICN2037 | SM5166 |
| Generic 64x32 | 1/16 | DP5125D | RUC7258E |

**Note**: Help us grow this list! Please let us know of other working and non working panels/chips.

## License

This project is dual-licensed under either of the following:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

## Contributing

We welcome contributions! Please feel free to submit a Pull Request.

By contributing, you agree that your submissions will be licensed under both
the Apache-2.0 and MIT licenses.

## Support

If you need help, please open an issue on our GitHub repository.
