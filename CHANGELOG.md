# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `parl_io` Hub75 driver
- `i2s_parallel` Hub75 driver for `esp32` (#5)
- `framebuffer::latched::DmaFrameBuffer` to support using a latch to [reduce pins/memory used](https://github.com/pixelmatix/SmartMatrix/blob/master/extras/hardware/ESP32/SmartLEDShield_ESP32_V0_sch.pdf)
- `i2s_parallel_latched` Hub75 driver for `esp32` with external latch

### Changed

- update `esp-hal` to 0.21.0
- parl_io, lcd_cam: frequency is required to be passed in
- refactor existing DmaFrameBuffer, changes signature!

### Fixed

- fix buffer size calculation (it was doubling the buffer size)

### Removed

- removed GPIO implementation

## [0.1.0] - 2024-08-16

- initial version
