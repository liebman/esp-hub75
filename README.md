# ESP-HUB75

[![Crates.io](https://img.shields.io/crates/v/esp-hub75.svg)](https://crates.io/crates/esp-hub75)
[![Documentation](https://docs.rs/esp-hub75/badge.svg)](https://docs.rs/esp-hub75)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](README.md)

A `no-std` Rust driver for HUB75-style LED matrix panels on ESP32-series
microcontrollers. HUB75 is a standard interface for driving large, bright,
and colorful RGB LED displays, commonly used in digital signage and art
installations.

This library provides a high-performance implementation that uses Direct
Memory Access (DMA) to drive the display with minimal CPU overhead. It is
designed to work with a variety of ESP32 models, using the most efficient
peripheral available on each chip:

- **ESP32-S3**: Uses the LCD_CAM peripheral
- **ESP32-C6**: Uses the PARL_IO peripheral
- **ESP32-C5**: Uses the PARL_IO peripheral (8-bit mode only; requires a
  latch circuit and `Hub75Pins8`)
- **ESP32**: Uses the I2S peripheral in parallel mode

The driver is built on top of the `embedded-graphics` crate, allowing you to
easily draw shapes, text, and images on the display. It also uses a zero-copy
framebuffer for efficient memory usage.

## Framebuffers

The `hub75-framebuffer` crate provides two families of framebuffer: the
**standard** framebuffers and the **bitplane** framebuffers. Each family has a
direct-drive variant (16-bit, no external latch) and a latched variant (8-bit,
requires an external address-latch circuit). Both families can be sent directly
to the peripheral without any extra formatting step. The difference is how they
achieve Binary Code Modulation (BCM):

- **Standard** framebuffers (`framebuffer::plain::DmaFrameBuffer` /
  `framebuffer::latched::DmaFrameBuffer`) pre-render a complete copy of the
  pixel data for every BCM bit-weight. This makes DMA output straightforward
  but multiplies memory usage by the number of frames (`frame_count`).
- **Bitplane** framebuffers (`framebuffer::bitplane::plain::DmaFrameBuffer` /
  `framebuffer::bitplane::latched::DmaFrameBuffer`) store only one bit per
  pixel per plane. The driver uses DMA descriptors to assemble the BCM output
  on the fly, avoiding the duplicated memory. The result is significantly lower
  RAM usage with the same visual quality.

Bitplane framebuffers are strongly recommended for most applications.

## Hardware Requirements

- An ESP32-series microcontroller (ESP32, ESP32-S3, ESP32-C5, or ESP32-C6)
- A HUB75 LED matrix panel
- A 5V power supply capable of providing several amps of current
- A 3.3V to 5V level shifter (e.g., 74HCT245) is highly recommended

**Note**: The ESP32 operates at 3.3V, while HUB75 panels require 5V logic
signals. While it may sometimes work without one, using a level shifter
ensures reliable operation and prevents damage to your hardware.

## Pin Configurations

This driver supports two types of HUB75 pin configurations, which you can
select based on your hardware setup:

- **`Hub75Pins16` (Direct Drive)**: This is the standard configuration where
  the row address lines are sent with every pixel. It requires more GPIO
  pins but works with any standard HUB75 panel.

- **`Hub75Pins8` (Latched)**: This configuration is for controller boards that
  include an external 74HC574-style latch for the row address lines. This is
  more memory-efficient and requires fewer GPIO pins. For more details on the
  required circuit, see the [`hub75-framebuffer` crate's documentation](https://crates.io/crates/hub75-framebuffer)
  or its [GitHub repository](https://github.com/liebman/hub75-framebuffer).

## Examples

The following examples demonstrate how to use this crate with different ESP32
variants.

### ESP32-S3 (LCD_CAM Interface)

- [`examples/hello_lcd_cam.rs`](examples/hello_lcd_cam.rs) - Displays "Hello, World!" using blocking `start()`.
- [`examples/lcd_cam.rs`](examples/lcd_cam.rs) - Async `start()`/`swap()` with color gradient and stats.
- [`examples/lcd_cam_latch.rs`](examples/lcd_cam_latch.rs) - Latched (8-bit) variant with color gradient and stats.
- [`examples/rustacean_lcd_cam.rs`](examples/rustacean_lcd_cam.rs) - Renders a Rustacean image.

### ESP32-C6 / ESP32-C5 (PARL_IO Interface)

- [`examples/parl_io.rs`](examples/parl_io.rs) - Async `start()`/`swap()` with color gradient and stats.
- [`examples/parl_io_latch.rs`](examples/parl_io_latch.rs) - Latched (8-bit) variant.
- [`examples/rustacean_parl_io.rs`](examples/rustacean_parl_io.rs) - Renders a Rustacean image.

**Note**: The ESP32-C5 does not support 16-bit mode, so only the latched
examples (`parl_io_latch.rs`) can be used with it.

### ESP32 (I2S Parallel Interface)

- [`examples/hello_i2s_parallel.rs`](examples/hello_i2s_parallel.rs) - Displays "Hello, World!" using blocking `start()`.
- [`examples/i2s_parallel.rs`](examples/i2s_parallel.rs) - Async `start()`/`swap()` with color gradient and stats.
- [`examples/i2s_parallel_latch.rs`](examples/i2s_parallel_latch.rs) - Latched (8-bit) variant.
- [`examples/rustacean_i2s.rs`](examples/rustacean_i2s.rs) - Renders a Rustacean image.

## Crate Features

- `esp32`: Enable support for the ESP32
- `esp32s3`: Enable support for the ESP32-S3
- `esp32c5`: Enable support for the ESP32-C5
- `esp32c6`: Enable support for the ESP32-C6
- `defmt`: Enable logging with `defmt`
- `log`: Enable logging with the `log` crate
- `invert-blank`: Invert the blank signal. This only applies to 8-bit latched
  configurations (`Hub75Pins8`); in 16-bit direct-drive mode the blank signal is
  always active-low. Some latch controller boards include a hardware inverter on
  the blank line — enable this feature to compensate.
- `invert-clock`: Invert the clock signal. By default the driver outputs data
  that changes on the falling edge of CLK so that it is stable when the panel
  latches on the rising edge. Enable this feature if your panel requires the
  opposite polarity.
- `full-chain-dma`: Build the entire BCM repetition chain in a single DMA
  transfer instead of one plane per interrupt. This reduces interrupt frequency
  at the cost of more DMA descriptor RAM. Note that the ESP32-C6 PARL_IO
  peripheral has a 65 535-byte per-transfer limit, which constrains the maximum
  panel size and plane count when this feature is enabled.
- `skip-black-pixels`: Forwards to the `hub75-framebuffer` crate, enabling an
  optimization that skips writing black pixels to the framebuffer.
- `tail-closes-latch`: Forwards to the `hub75-framebuffer` crate. Applies to
  `plain` and `bitplane::plain` framebuffers only (not the latched variants).
  Enabling `tail-closes-latch` adds one extra 16-bit word per DMA buffer
  (`plain`) or one extra word per bit-plane (`bitplane::plain`) that parks the
  bus with LATCH=0 and OE=BLANK, cleanly terminating the transfer.
- `iram`: Place the driver’s hot-path (render / DMA wait functions) in
  Instruction RAM (IRAM) to avoid flash-cache stalls (for example during
  Wi-Fi, PSRAM, or SPI-flash activity) that can cause visible flicker.
  Enabling this feature consumes roughly 5–10 KiB of IRAM.
- `tail-closes-latch`: Forwards to `hub75-framebuffer` (plain framebuffers
  only). Appends a single extra "tail" word at the end of each DMA buffer that
  drives the LATCH signal LOW (de-asserted) on the final clock edge. Without
  this feature the last word in each row asserts LATCH HIGH to latch shifted
  data into the LED drivers, and the GPIO pins remain in that state after the
  DMA transfer completes. Some hardware configurations (e.g. free-running DMA
  loops or peripherals that continue clocking after the descriptor chain ends)
  can re-latch stale data or glitch if LATCH is left asserted.

  Enabling `tail-closes-latch` adds one 16-bit `Entry` (for `plain`) or one
  entry per bit-plane (for `bitplane::plain`) that parks the bus with LATCH=0
  and OE=BLANK, cleanly terminating the transfer. The cost is a single extra
  word per DMA chunk, which is negligible compared to the frame data.

- `blank-delay-1` / `blank-delay-2` / `blank-delay-4` / `blank-delay-8`:
  Forwards to `hub75-framebuffer`. Control the number of pixel-clock cycles of
  blanking (OE HIGH) inserted around row address changes in plain framebuffers
  (`plain` and `bitplane::plain`). The blanking delay gives the address lines
  time to settle before the new row is latched and lit, preventing ghosting or
  "bleeding" artifacts between rows.

  | Feature | Blanking cycles |
  |---------|-----------------|
  | *(none)* | 1 (default) |
  | `blank-delay-1` | 1 |
  | `blank-delay-2` | 2 |
  | `blank-delay-4` | 4 |
  | `blank-delay-8` | 8 |

  Higher values reduce ghosting at the cost of slightly less brightness (the
  LEDs are on for less time per scan line). Start with the default and increase
  only if you observe row-transition artifacts on your particular panel
  hardware.

##  Known Working Panels

This library should work with any "normal" RGB matrix panels. The following panels have been
tested and confirmed to work:

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
