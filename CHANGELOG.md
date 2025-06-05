# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- next-header -->

## [Unreleased] - ReleaseDate

## [0.2.1] - 2025-06-05

### Added

- doc link in Cargo.toml
- badges in README

## [0.2.0] - 2025-06-03

### Added

- `parl_io` Hub75 driver
- `i2s_parallel` Hub75 driver for `esp32` (#5)
- `framebuffer::latched::DmaFrameBuffer` to support using a latch to [reduce pins/memory used](https://github.com/pixelmatix/SmartMatrix/blob/master/extras/hardware/ESP32/SmartLEDShield_ESP32_V0_sch.pdf)
- `i2s_parallel_latch` Hub75 driver for `esp32` with external latch
- `lcd_cam_latch` Hub75 driver for `esp32-s3` with external latch support
- Refactored framebuffer traits and interfaces (#13)
- Added `Hub75Pins` trait for unified pin configuration (#13)
- Added support for 8-bit and 16-bit data width in LCD_CAM driver (#13)

### Changed

- update `esp-hal` to 0.21.0
- parl_io, lcd_cam: frequency is required to be passed in
- refactor existing DmaFrameBuffer, changes signature!
- Merged latch implementations into main drivers (#13)
- Unified pin configuration through traits (#13)

### Fixed

- fix buffer size calculation (it was doubling the buffer size)

### Removed

- removed GPIO implementation
- removed separate latch implementations, they were merged into main drivers (#13)

## [0.1.0] - 2024-08-16

- initial version

<!-- next-url -->
[Unreleased]: https://github.com/liebman/esp-hub75/compare/v0.2.1...HEAD
[0.2.1]: https://github.com/liebman/esp-hub75/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/liebman/esp-hub75/compare/v0.1.0...v0.2.0
