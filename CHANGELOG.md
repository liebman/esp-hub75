# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- next-header -->

## [Unreleased] - ReleaseDate

### Added

* added `iram` feature to place the hot rendering function in ram
* new example i2s_parallel_dimming as an example of using a delay as overall
  brightness control. [#22](https://github.com/liebman/esp-hub75/pull/22)

### Changed

* bump `hub75-framebuffer` to `0.4.0` [#27](https://github.com/liebman/esp-hub75/pull/27)

## [0.5.0] - 2025-07-16

### Changed

* bump `esp-hal` to `1.0.0-rc.0` [#18](https://github.com/liebman/esp-hub75/pull/18)
* bump `hub-framebuffer` to 0.3.0 [#18](https://github.com/liebman/esp-hub75/pull/18)
* bump rust version to 1.88 [#20](https://github.com/liebman/esp-hub75/pull/20)

## [0.4.0] - 2025-06-20

### ⚠️ Breaking

* Renamed `DmaFrameBuffer::clear()` to `erase()`.  
  The new name avoids shadowing `embedded_graphics::DrawTarget::clear(Color)`.  
  Update your code: `fb.clear()` ➜ `fb.erase()`.  
  If you actually wanted the trait method, call `fb.clear(Color::BLACK)` instead.

### Added

* `skip-black-pixels` feature that gives a performance boot in some cases

### Changed

* bump `hub75-framebuffer` to 0.2.0

## [0.3.0] - 2025-06-14

### Changed

* pin `esp-hal` to =1.0.0-beta.1
* bump `esp-hal-embassy` to 0.8.1
* bump `bitfield` to 0.19.1
* bump `defmt` to 1.0.1
* bump `defmt-rtt` (dev-dependencies) to 1.0.0
* moved `static_cell` to dev-dependencies
* extracted framebuffer support to `hub75-framebuffer`

### Removed

* removed `critical-section` as its an indirect dependency not a direct dependency

## [0.2.3] - 2025-06-05

### Fixed

* correct targets for `doc.rs` config

## [0.2.2] - 2025-06-05

### Fixed

* cleanup some issues in README
* add `doc.rs` config to `Cargo.toml`

## [0.2.1] - 2025-06-05

### Added

* doc link in `Cargo.toml`
* badges in README

## [0.2.0] - 2025-06-03

### Added

* `parl_io` Hub75 driver
* `i2s_parallel` Hub75 driver for `esp32` (#5)
* `framebuffer::latched::DmaFrameBuffer` to support using a latch to
  [reduce pins/memory used](https://github.com/pixelmatix/SmartMatrix/blob/master/extras/hardware/ESP32/SmartLEDShield_ESP32_V0_sch.pdf)
* `i2s_parallel_latch` Hub75 driver for `esp32` with external latch
* `lcd_cam_latch` Hub75 driver for `esp32-s3` with external latch support
* Refactored framebuffer traits and interfaces (#13)
* Added `Hub75Pins` trait for unified pin configuration (#13)
* Added support for 8-bit and 16-bit data width in LCD_CAM driver (#13)

### Changed

* update `esp-hal` to 0.21.0
* `parl_io`, `lcd_cam`: frequency is required to be passed in
* refactor existing `DmaFrameBuffer`, changes signature!
* Merged latch implementations into main drivers (#13)
* Unified pin configuration through traits (#13)

### Fixed

* fix buffer size calculation (it was doubling the buffer size)

### Removed

* removed GPIO implementation
* removed separate latch implementations, they were merged into main drivers (#13)

## [0.1.0] - 2024-08-16

* initial version

<!-- next-url -->
[Unreleased]: https://github.com/liebman/esp-hub75/compare/v0.5.0...HEAD
[0.5.0]: https://github.com/liebman/esp-hub75/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/liebman/esp-hub75/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/liebman/esp-hub75/compare/v0.2.3...v0.3.0
[0.2.3]: https://github.com/liebman/esp-hub75/compare/v0.2.2...v0.2.3
[0.2.2]: https://github.com/liebman/esp-hub75/compare/v0.2.1...v0.2.2
[0.2.1]: https://github.com/liebman/esp-hub75/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/liebman/esp-hub75/compare/v0.1.0...v0.2.0
