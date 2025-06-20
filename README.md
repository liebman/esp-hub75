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
- **ESP32**: Uses the I2S peripheral in parallel mode

The driver is built on top of the `embedded-graphics` crate, allowing you to
easily draw shapes, text, and images on the display. It also uses a zero-copy
framebuffer for efficient memory usage.

## Hardware Requirements

- An ESP32-series microcontroller (ESP32, ESP32-S3, or ESP32-C6)
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

- [`examples/hello_lcd_cam`](examples/hello_lcd_cam.rs) - Displays "Hello, World!".
- [`examples/lcd_cam.rs`](examples/lcd_cam.rs) - Shows a color gradient and stats.

### ESP32-C6 (PARL_IO Interface)

- [`examples/hello_parl_io.rs`](examples/hello_parl_io.rs) - Displays "Hello, World!".
- [`examples/parl_io.rs`](examples/parl_io.rs) - Shows a color gradient and stats.

### ESP32 (I2S Parallel Interface)

- [`examples/hello_i2s_parallel.rs`](examples/hello_i2s_parallel.rs) - Displays
  "Hello, World!".
- [`examples/i2s_parallel.rs`](examples/i2s_parallel.rs) - Shows a color gradient
  and stats.

## Crate Features

- `esp32`: Enable support for the ESP32
- `esp32s3`: Enable support for the ESP32-S3
- `esp32c6`: Enable support for the ESP32-C6
- `defmt`: Enable logging with `defmt`
- `log`: Enable logging with the `log` crate
- `invert-blank`: Invert the blank signal, required for some controller boards.
- `skip-black-pixels`: Forwards to the `hub75-framebuffer` crate, enabling an
  optimization that skips writing black pixels to the framebuffer.

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
