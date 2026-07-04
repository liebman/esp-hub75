use esp_metadata_generated::Chip;

fn main() {
    esp_metadata_generated::assert_unique_used_features!(
        "esp32", "esp32c5", "esp32c6", "esp32s3"
    );

    let chip =
        Chip::from_cargo_feature().expect("exactly one chip feature must be enabled");
    chip.define_cfgs();

    println!("cargo:rustc-check-cfg=cfg(hub75_use_parl_io)");
    println!("cargo:rustc-check-cfg=cfg(hub75_use_lcd_cam)");
    println!("cargo:rustc-check-cfg=cfg(hub75_use_i2s_parallel)");

    if chip.contains("parl_io_driver_supported") {
        println!("cargo:rustc-cfg=hub75_use_parl_io");
    } else if chip.contains("soc_has_lcd_cam") {
        println!("cargo:rustc-cfg=hub75_use_lcd_cam");
    } else if chip.contains("esp32") {
        println!("cargo:rustc-cfg=hub75_use_i2s_parallel");
    } else {
        panic!(
            "No supported parallel output peripheral found for {}. \
             esp-hub75 requires PARL_IO, LCD_CAM, or ESP32 I2S parallel.",
            chip.name()
        );
    }
}
