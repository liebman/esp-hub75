#[macro_export]
macro_rules! assert_unique_used_features {
    ($($feature:literal),+ $(,)?) => {
        assert!(
            (0 $(+ cfg!(feature = $feature) as usize)+ ) == 1,
            "Exactly one of the following features must be enabled: {}",
            [$($feature),+].join(", ")
        );
    };
}

fn main() {
    // NOTE: update when adding new device support!
    // Ensure that exactly one chip has been specified:
    assert_unique_used_features!("esp32", "esp32c6", "esp32s3", "esp32s3-idf", "esp32p4-idf");

    let target = std::env::var("TARGET").unwrap();

    #[cfg(feature = "esp32")]
    {
        assert!(
            target == "xtensa-esp32-none-elf",
            "feature esp32 does not match target {target}"
        );
        println!("cargo:rustc-cfg=esp32");
    }

    #[cfg(feature = "esp32s3")]
    {
        assert!(
            target == "xtensa-esp32s3-none-elf",
            "feature esp32s3 does not match target {target}"
        );
        println!("cargo:rustc-cfg=esp32s3");
    }

    #[cfg(feature = "esp32s3-idf")]
    {
        assert!(
            target == "xtensa-esp32s3-espidf",
            "feature esp32s3-idf requires target xtensa-esp32s3-espidf, got {target}"
        );
        println!("cargo:rustc-cfg=esp32s3_idf");
        println!("cargo:rustc-cfg=idf_backend");
        // Emit ESP-IDF link arguments (library paths, linker scripts, etc.)
        embuild::espidf::sysenv::output();
    }

    #[cfg(feature = "esp32p4-idf")]
    {
        assert!(
            target == "riscv32imafc-esp-espidf",
            "feature esp32p4-idf requires target riscv32imafc-esp-espidf, got {target}"
        );
        println!("cargo:rustc-cfg=esp32p4_idf");
        println!("cargo:rustc-cfg=idf_backend");
        // Emit ESP-IDF link arguments (library paths, linker scripts, etc.)
        embuild::espidf::sysenv::output();
    }

    #[cfg(feature = "esp32c6")]
    {
        assert!(
            target == "riscv32imac-unknown-none-elf",
            "feature esp32c6 does not match target {target}"
        );
        println!("cargo:rustc-cfg=esp32c6");
    }
}
