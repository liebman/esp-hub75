# esp-hub75

Drive HUB75 displays from ESP32 series SOCs.

All implementations use DMA where supported.

- [ ] - documentation!
- [x] - LCD peripheral  (async) for: `esp32s3`
  - [ ] - sync support for LCD peripheral
- [x] - I2S peripheral in LCD mode: `esp32`
- [ ] - sync support for the i2s peripheral
- [x] - PARL_IO peripheral: `esp32c6`
  - [ ] - sync support for PARL_IO peripheral

## Notes

- To drive HUB75 displays reliably you need a level converter between the GPIOs and the HUB75!

## Examples

### [esp32s3: lcd_cam](examples/lcd_cam.rs)

Will display  a red/green/blue gradient on the top 24 lines and some rendering and refresh stats at the bottom.
Expects a 64x64 matrix.

### [esp32c6: parl_io](examples/parl_io.rs)

Will display  a red/green/blue gradient on the top 24 lines and some rendering and refresh stats at the bottom.
Expects a 64x64 matrix.

### [esp32: i2s](examples/i2s_parallel.rs)

Will display  a red/green/blue gradient on the top 24 lines and some rendering and refresh stats at the bottom.
Expects a 64x64 matrix.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
