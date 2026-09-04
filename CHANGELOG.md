# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- next-header -->

## [Unreleased] - ReleaseDate

### ⚠️ Breaking

* Removed the `GdmaChannelNum` trait. The S3 backend now binds and enables its
  interrupts through the new `esp-hal` `I8080` interrupt API
  (`set_interrupt_handler` / `set_dma_interrupt_handler` / `listen` /
  `listen_dma`), so the DMA channel number is no longer needed. This requires
  an `esp-hal` build that includes the `I8080` interrupt API.
* The `Hub75::new` / `Hub75::new_async` constructors now take a
  [`Hub75Config`] instead of a bare `Rate`. Use `Hub75Config::new(rate)` to
  preserve the previous behavior.

### Added

* `circular-dma` support for ESP32-C5 (PARL_IO). The circular chain carries no
  `suc_eof` (a `suc_eof` descriptor terminates the PARL_IO transfer); a swap
  arms the boundary detector (`suc_eof` on the last descriptor + the
  `PARL_IO` `TxEof` interrupt), the ISR applies the pending buffer delta at
  the pass boundary and restarts the halted transfer. Requires the local
  `esp-hal` with the PARL_IO transfer interrupt API.
* `Hub75Config::frame_counter` (circular-DMA mode): when `false`, no
  interrupts are enabled in steady state on any backend — the detector is
  armed only around a swap and disarmed again. When `true` (default), the
  frame-count ISR runs every frame as before; `frame_count()` then reports
  real frame counts, otherwise completed swaps.
* In circular-DMA mode the swap delta is now applied by the ISR at the pass
  boundary instead of immediately in `swap()`, eliminating the mid-frame
  tear the previous implementation could produce.


## [0.16.0] - 2026-09-02

### Changed

* bump `esp-hal` to `1.2.0`

## [0.15.0] - 2026-08-26

### ⚠️ Breaking

* Removed the deprecated free function
  `dma_descriptor_count(bcm_chunk_count, bcm_chunk_bytes)` (deprecated since
  0.7.0). Use the new generic const fn
  `esp_hub75::dma_descriptor_count::<FB>(MAX_DMA_CHUNK_SIZE)` or the
  `hub75_dma_descriptors!` macro instead.
* Requires a `hub75-framebuffer` release exposing
  `FrameBuffer::BCM_SEGMENT_SHAPES`/`BCM_SEQUENCE_LEN`/`BCM_SEQUENCE_COUNT`.

### Added

* `reverse-row-order` feature (forwards to `hub75-framebuffer`) storing the
  rows of every framebuffer layout in reverse scan order, so the DMA stream
  renders the last panel row first and row 0 last. Logical coordinates are
  unchanged, only the scan order changes.
* `lead-blank-32` and `trail-blank-32` features extending blanking delay options to 32 pixel-clock cycles.
* New generic const fn `esp_hub75::dma_descriptor_count::<FB>(max_chunk)`
  computes the required DMA descriptor count from the framebuffer type's
  static `FrameBuffer::BCM_SEGMENT_SHAPES`; no framebuffer instance
  needed, so descriptor tables stay statically allocated.
* `hub75_dma_descriptors!` now works with any `FrameBuffer` type,
  including `tiling::RemappedFrameBuffer` (previously only framebuffer
  types with an inherent `dma_descriptor_count()` could be used).
* Compile-time check that a framebuffer's BCM segment count fits the
  driver's segment cache (`MAX_SEGMENTS`).
* `invert-oe` feature forwarding to `hub75-framebuffer`, inverting the
  output-enable (OE) signal in the generated data stream.

### Changed

* DMA descriptor sizing moved out of `hub75-framebuffer` into this crate:
  the framebuffer crate now exposes only the platform-neutral BCM scan
  sequence; all ESP DMA descriptor logic lives here.
* `hub75_dma_descriptors!` no longer over-allocates in the default
  group-based DMA mode: it now allocates only the largest single transfer
  group's descriptors instead of the whole frame's chain (e.g. 33 instead
  of 1056 descriptors for a 64×64 row-major 6-plane configuration, saving
  ~12 KiB of RAM). `full-chain-dma`/`circular-dma` still allocate the full
  chain as required.
* `MAX_SEGMENTS` raised from 288 to 320 to cover the true worst case
  (32 row-pairs × (8 planes + inter-row gap + trailer)).
* `invert-blank` now applies to 16-bit direct-drive (`Hub75Pins16`)
  configurations in addition to 8-bit latched (`Hub75Pins8`); previously the
  16-bit blank pin was always inverted in hardware.

## [0.14.0] - 2026-08-02

### ⚠️ Breaking — Major refactor [#49](https://github.com/liebman/esp-hub75/pull/49)

* **ISR-driven continuous refresh replaces user render loop.** Previously the
  user had to run a dedicated task calling `hub75.render(fb)` in a tight loop —
  each call sent one frame via DMA, returned a `Hub75Transfer` (consuming
  `Hub75`), and the display was only alive while the loop ran. Now `new()` /
  `new_async()` take an initial `&'static FB` and start an internal ISR that
  refreshes the display continuously. To update the image, call `hub75.swap(fb)?`
  which returns a `Hub75Swap` transfer object; wait on it to reclaim the old
  buffer.

  ```rust
  // Before: dedicated render task looping forever
  let mut hub75 = Hub75::new_async(parl_io, pins, channel, tx_descriptors, freq)?;
  loop {
      let mut xfer = hub75.render(fb)?;
      xfer.wait_for_done().await?;
      let (res, h) = xfer.wait(); hub75 = h;
  }

  // After: ISR refreshes continuously; swap buffers when ready
  let hub75 = Hub75::new_async(parl_io, pins, channel, tx_descriptors, freq, &*fb0)?;
  loop {
      // ... draw into fb1 ...
      let mut xfer = hub75.swap(fb1)?;
      xfer.wait_for_done().await;
      fb1 = xfer.wait()?;
  }
  ```

* **`Hub75Pins` trait gains associated `Word` type** — word-size mismatch with
  `FrameBuffer::Word` is now a compile-time error.

### Added

* `Hub75Swap<FB>` transfer object returned by `swap()` — allows callers to
  do other work between initiating the swap and waiting for completion.
* `full-chain-dma` feature — single DMA transfer per full BCM chain.
* `circular-dma` feature — circular DMA descriptor chain for ESP32 and
  ESP32-S3. The DMA engine starts once and loops forever; buffer swaps are
  instant pointer-delta updates with no DMA stop/restart. Compile-time error
  if enabled on ESP32-C5/C6 (PARL_IO does not support circular chains).
* `frame_count()` method on `Hub75` to query the number of completed BCM
  frames since driver creation.
* `inter-row-blank-4`, `inter-row-blank-8`, `inter-row-blank-16`, and `inter-row-blank-32` features that insert additional dead clock
  cycles between the latch and the address change at the end of each row. Forwarded to `hub75-framebuffer`; in plain framebuffers
  the gap defers the address change to the first pixel of the next row, giving slow panels more time to finish blanking.
* New examples: `rustacean_i2s`, `rustacean_lcd_cam`, `rustacean_parl_io`, `esp32-trinity-tiled`.


### Removed

* `Hub75Transfer`, `render()`, `render_buf()`.
* Several old examples replaced by new ones above.

## [0.13.0] - 2026-07-25

### ⚠️ Breaking

* Renamed blank delay features from `blank-delay-1/2/4/8` to separate `lead-blank-1/2/4/8/16` and `trail-blank-1/2/4/8/16` features. The lead blank delay controls how many clock cycles the output is blanked before the row address is changed, and the trail blank delay controls blanking after the row address is changed. The new `16` value is also available. Default is 1 for plain framebuffers and 0 for latched framebuffers (which handle timing via extra `Address` entries to manage the address change).

### Added

* add esp32-trinity example

### Changed

* bump `hub75-franebuffer` to 0.10.0 for lead/trail blank support

## [0.12.0] - 2026-07-19

### Added

* `blank-delay-1` / `blank-delay-2` / `blank-delay-4` / `blank-delay-8` features
  to control blanking cycles around row address changes (forwarded to
  `hub75-framebuffer`).

* feature `tail-closes-latch` (pass thru to `hub75-framebuffer`)  will include an
  extra entry in `plain` and `bitplane/plain` implementations to close the latch
  at the end of the the buffer (`plain`) and at the end of each plane (`bitplane/plain`)

### Changed

* bump `hub75-framebuffer` to `0.9.0`

## [0.11.0] - 2026-05-02

### ⚠️ Breaking

* **new macro to allocate dma descriptors:** The new bitplane framebuffers require more DMA descriptors
  to implement BCM by randering the same frame multiple times.
  ```
  -    let (_, tx_descriptors) = esp_hal::dma_descriptors!(0, FBType::dma_buffer_size_bytes());
  +    let tx_descriptors = esp_hub75::hub75_dma_descriptors!(FBType);
  ```

### Added

* add support for `ESP32-C5`
* add bitplane framebuffer support by using dma descripters to render bitplanes for BCM using less memory

### Changed

* bump `hub75-framebuffer` to `0.8.0`
* Removed unnessassary const generics from FrameBuffer, MutableFrameBuffer, FrameBufferOperations

## [0.10.0] - 2026-04-25

* bump `esp-hal` to `1.1.0`
* bump `hub75-framebuffer` to `0.7.0`

## [0.9.0] - 2026-03-21

### ⚠️ Breaking

* **ESP32 (I2S parallel) and ESP32-C6:** Fixed the default clock polarity.
  Both were using the wrong polarity — data was changing on the rising edge of
  CLK, but most HUB75 panels latch data on the rising edge so data must be
  stable at that point. The default is now correct for most panels, with data
  changing on the falling edge of CLK. If your panel relied on the old
  (incorrect) polarity, enable the `invert-clock` feature.

### Added

* Added an `invert-clock` feature. By default, data changes on the falling
  edge of CLK so that it is stable when the panel latches on the rising edge
  (the standard for most HUB75 panels). Enable `invert-clock` for panels that
  require the opposite polarity. [#44](https://github.com/liebman/esp-hub75/pull/44)

## [0.8.0] - 2025-11-01

### Changed

* examples: add esp_bootloader_esp_idf::esp_app_desc!() for latest espflash [#37](https://github.com/liebman/esp-hub75/pull/37)
* bump `esp-hal` to `1.0.0` [#39](https://github.com/liebman/esp-hub75/pull/39)

## [0.7.0] - 2025-10-16

### Changed

* bump `esp-hal` to `1.0.0-rc.1` [#34](https://github.com/liebman/esp-hub75/pull/34)
* bump `hub75-framebuffer` to `0.5.0` [#34](https://github.com/liebman/esp-hub75/pull/34)

## [0.6.0] - 2025-08-17

### Added

* added `iram` feature to place the hot rendering function in ram
* new example `i2s_parallel_dimming` as an example of using a delay as overall
  brightness control. [#22](https://github.com/liebman/esp-hub75/pull/22)
* new example `lcd_cam_tiled` as an example of tiling multiple panels into a
  larger display [#29](https://github.com/liebman/esp-hub75/pull/29)

### Changed

* bump `hub75-framebuffer` to `0.4.2` [#29](https://github.com/liebman/esp-hub75/pull/29)
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
[Unreleased]: https://github.com/liebman/esp-hub75/compare/v0.16.0...HEAD
[0.16.0]: https://github.com/liebman/esp-hub75/compare/v0.15.0...v0.16.0
[0.15.0]: https://github.com/liebman/esp-hub75/compare/v0.14.0...v0.15.0
[0.14.0]: https://github.com/liebman/esp-hub75/compare/v0.13.0...v0.14.0
[0.13.0]: https://github.com/liebman/esp-hub75/compare/v0.12.0...v0.13.0
[0.12.0]: https://github.com/liebman/esp-hub75/compare/v0.11.0...v0.12.0
[0.11.0]: https://github.com/liebman/esp-hub75/compare/v0.10.0...v0.11.0
[0.10.0]: https://github.com/liebman/esp-hub75/compare/v0.9.0...v0.10.0
[0.9.0]: https://github.com/liebman/esp-hub75/compare/v0.8.0...v0.9.0
[0.8.0]: https://github.com/liebman/esp-hub75/compare/v0.7.0...v0.8.0
[0.7.0]: https://github.com/liebman/esp-hub75/compare/v0.6.0...v0.7.0
[0.6.0]: https://github.com/liebman/esp-hub75/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/liebman/esp-hub75/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/liebman/esp-hub75/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/liebman/esp-hub75/compare/v0.2.3...v0.3.0
[0.2.3]: https://github.com/liebman/esp-hub75/compare/v0.2.2...v0.2.3
[0.2.2]: https://github.com/liebman/esp-hub75/compare/v0.2.1...v0.2.2
[0.2.1]: https://github.com/liebman/esp-hub75/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/liebman/esp-hub75/compare/v0.1.0...v0.2.0
