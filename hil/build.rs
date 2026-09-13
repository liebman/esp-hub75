//! Emits the chip `cfg`s (`esp32s3`, `soc_has_lcd_cam`, `timergroup_timg0`,
//! ...) that the sources of this crate use.
//!
//! A build script's `rustc-cfg` output only applies to its own package, so the
//! test binaries need their own copy of this even though `esp-hub75` does the
//! same thing.

use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    esp_metadata_generated::assert_unique_used_features!("esp32", "esp32c5", "esp32c6", "esp32s3");

    esp_metadata_generated::Chip::from_cargo_feature()?.define_cfgs();

    // Used by rust-analyzer-only code paths; keeps `unexpected_cfgs` quiet.
    println!("cargo::rustc-check-cfg=cfg(rust_analyzer)");

    Ok(())
}
