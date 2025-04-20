# ESP-HUB75

A no-std Rust library for driving HUB75 LED matrix displays from ESP32 series microcontrollers. This library provides efficient implementations using DMA (Direct Memory Access) for optimal performance across all supported ESP32 variants.

## Features

- DMA-based implementations for supported ESP32 variants:
  - ESP32-S3: LCD_CAM DMA interface
  - ESP32-C6: PARL_IO DMA interface
  - ESP32: I2S Parallel DMA interface
- Zero-copy frame buffering with DMA transfers
- Support for embedded-graphics for drawing text and shapes

## Hardware Requirements

- ESP32 series microcontroller (ESP32, ESP32-S3, or ESP32-C6)
- HUB75 LED matrix display
- Level shifter/converter (recommended for reliable operation)
  - Recommended: 74HCT245 or similar 3.3V to 5V level shifter
  - Connect between ESP32 GPIOs and HUB75 input pins

## Examples

### ESP32-S3 (LCD_CAM Interface)

[`examples/lcd_cam.rs`](examples/lcd_cam.rs)

```rust
// Displays "Hello, World!" text on a 64x64 pixel matrix
// Uses embedded-graphics for text rendering
```

### ESP32-C6 (PARL_IO Interface)

[`examples/parl_io.rs`](examples/parl_io.rs)

```rust
// Displays a RGB gradient and performance statistics
// Compatible with 64x64 pixel matrices
```

### ESP32 (I2S Parallel Interface)

[`examples/i2s_parallel.rs`](examples/i2s_parallel.rs)

```rust
// Displays a RGB gradient and performance statistics
// Compatible with 64x64 pixel matrices
```

## Notes

**Note**: While not strictly required, using a 3.3V to 5V level shifter between your ESP32 and the HUB75 display is recommended for reliable operation. The ESP32 operates at 3.3V while HUB75 displays typically require 5V signals. A level shifter helps ensure proper signal levels and reliable display operation.

## License

This project is dual-licensed under:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. By contributing to this project, you agree that your contributions will be licensed under both the Apache 2.0 and MIT licenses, as specified above.

## Support

For support, please open an issue in the GitHub repository. We'll do our best to help you get your HUB75 display working with your ESP32.
