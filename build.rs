use esp_build::assert_unique_used_features;

fn main() {
    // NOTE: update when adding new device support!
    // Ensure that exactly one chip has been specified:
    assert_unique_used_features!("esp32", "esp32c6", "esp32s3");

    let target = std::env::var("TARGET").unwrap();

    #[cfg(feature = "esp32")]
    {
        assert!(
            target == "xtensa-esp32-none-elf",
            "feature esp32 does not match target {}",
            target
        );
        println!("cargo:rustc-cfg=esp32");
    }

    #[cfg(feature = "esp32s3")]
    {
        assert!(
            target == "xtensa-esp32s3-none-elf",
            "feature esp32s3 does not match target {}",
            target
        );
        println!("cargo:rustc-cfg=esp32s3");
    }

    #[cfg(feature = "esp32c6")]
    {
        assert!(
            target == "riscv32imac-unknown-none-elf",
            "feature esp32c6 does not match target {}",
            target
        );
        println!("cargo:rustc-cfg=esp32c6");
    }
}
