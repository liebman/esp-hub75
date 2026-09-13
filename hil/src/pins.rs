//! Per-chip HUB75 pin maps and backend selection for the HIL rig.
//!
//! The pin assignments mirror the `gradient` (16-bit direct drive) and
//! `gradient-latched` (8-bit latched) examples, so a panel wired for either
//! example works here unchanged.
//!
//! Which wiring the tests exercise is chosen by the `bus16` (default) and
//! `bus8` features; the macros below then resolve to the matching pin struct
//! and DMA channel, so a test never has to mention the bus width:
//!
//! ```ignore
//! let pins = hil::hub75_pins!(peripherals);
//! let (peripheral, dma) = hil::hub75_backend!(peripherals);
//! let _oe = hil::oe_pin!(peripherals); // `()` on direct-drive boards
//! ```
//!
//! These are macros rather than functions because `Peripherals` can only be
//! dismantled by partial move, which a function cannot do for its caller.
//!
//! | Chip | 16-bit direct drive (`bus16`) | 8-bit latched (`bus8`) |
//! | --- | --- | --- |
//! | ESP32 | `I2S0` / `DMA_I2S0` | `I2S1` / `DMA_I2S1` |
//! | ESP32-S3 | `LCD_CAM` / `DMA_CH0` | `LCD_CAM` / `DMA_CH0` |
//! | ESP32-C6 | `PARL_IO` / `DMA_CH0` | `PARL_IO` / `DMA_CH0` |
//! | ESP32-C5 | not supported by the driver | `PARL_IO` / `DMA_CH0` |
//!
//! The ESP32's 8-bit wiring needs `I2S1`: `I2S0` drops every odd byte.
//!
//! Only ESP32-S3 has been validated on hardware. The other maps are copies of
//! the corresponding example and are expected to work once a board is
//! connected; they exist so a new board only needs a wiring check.

// ---------------------------------------------------------------------------
// 16-bit direct drive: `Hub75Pins16` (bus16)
// ---------------------------------------------------------------------------

/// Builds the pin struct for the selected bus width from an owned
/// `Peripherals`.
///
/// Expands to [`Hub75Pins16`] (direct drive) with `bus16`, or [`Hub75Pins8`]
/// (latched) with `bus8`, using the selected chip's pin map.
///
/// The OE / brightness pin of a latched board is not part of the driver's pin
/// set and is taken separately (see `oe_pin!`).
///
/// [`Hub75Pins16`]: esp_hub75::Hub75Pins16
/// [`Hub75Pins8`]: esp_hub75::Hub75Pins8
#[cfg(all(esp32s3, feature = "bus16"))]
#[macro_export]
macro_rules! hub75_pins {
    ($p:expr) => {{
        use ::esp_hal::gpio::Pin as _;

        ::esp_hub75::Hub75Pins16 {
            red1: ($p).GPIO38.degrade(),
            grn1: ($p).GPIO42.degrade(),
            blu1: ($p).GPIO48.degrade(),
            red2: ($p).GPIO47.degrade(),
            grn2: ($p).GPIO2.degrade(),
            blu2: ($p).GPIO21.degrade(),
            addr0: ($p).GPIO14.degrade(),
            addr1: ($p).GPIO46.degrade(),
            addr2: ($p).GPIO13.degrade(),
            addr3: ($p).GPIO9.degrade(),
            addr4: ($p).GPIO3.degrade(),
            blank: ($p).GPIO11.degrade(),
            clock: ($p).GPIO12.degrade(),
            latch: ($p).GPIO10.degrade(),
        }
    }};
}

#[cfg(all(esp32, feature = "bus16"))]
#[macro_export]
macro_rules! hub75_pins {
    ($p:expr) => {{
        use ::esp_hal::gpio::Pin as _;

        ::esp_hub75::Hub75Pins16 {
            red1: ($p).GPIO16.degrade(),
            grn1: ($p).GPIO4.degrade(),
            blu1: ($p).GPIO17.degrade(),
            red2: ($p).GPIO18.degrade(),
            grn2: ($p).GPIO5.degrade(),
            blu2: ($p).GPIO19.degrade(),
            addr0: ($p).GPIO15.degrade(),
            addr1: ($p).GPIO13.degrade(),
            addr2: ($p).GPIO12.degrade(),
            addr3: ($p).GPIO14.degrade(),
            addr4: ($p).GPIO2.degrade(),
            blank: ($p).GPIO25.degrade(),
            clock: ($p).GPIO27.degrade(),
            latch: ($p).GPIO26.degrade(),
        }
    }};
}

#[cfg(all(esp32c6, feature = "bus16"))]
#[macro_export]
macro_rules! hub75_pins {
    ($p:expr) => {{
        use ::esp_hal::gpio::Pin as _;

        ::esp_hub75::Hub75Pins16 {
            red1: ($p).GPIO19.degrade(),
            grn1: ($p).GPIO20.degrade(),
            blu1: ($p).GPIO21.degrade(),
            red2: ($p).GPIO22.degrade(),
            grn2: ($p).GPIO23.degrade(),
            blu2: ($p).GPIO15.degrade(),
            addr0: ($p).GPIO10.degrade(),
            addr1: ($p).GPIO8.degrade(),
            addr2: ($p).GPIO1.degrade(),
            addr3: ($p).GPIO0.degrade(),
            addr4: ($p).GPIO11.degrade(),
            blank: ($p).GPIO5.degrade(),
            clock: ($p).GPIO7.degrade(),
            latch: ($p).GPIO6.degrade(),
        }
    }};
}

// ---------------------------------------------------------------------------
// 8-bit latched: `Hub75Pins8` (bus8)
// ---------------------------------------------------------------------------

/// Builds the 8-bit latched pin struct ([`Hub75Pins8`]) for the selected chip.
///
/// These are the pin assignments of the `gradient-latched` example.
///
/// [`Hub75Pins8`]: esp_hub75::Hub75Pins8
#[cfg(all(esp32s3, feature = "bus8"))]
#[macro_export]
macro_rules! hub75_pins {
    ($p:expr) => {{
        use ::esp_hal::gpio::Pin as _;

        ::esp_hub75::Hub75Pins8 {
            red1: ($p).GPIO10.degrade(),
            grn1: ($p).GPIO11.degrade(),
            blu1: ($p).GPIO12.degrade(),
            red2: ($p).GPIO13.degrade(),
            grn2: ($p).GPIO9.degrade(),
            blu2: ($p).GPIO14.degrade(),
            blank: ($p).GPIO45.degrade(),
            clock: ($p).GPIO47.degrade(),
            latch: ($p).GPIO21.degrade(),
        }
    }};
}

#[cfg(all(esp32, feature = "bus8"))]
#[macro_export]
macro_rules! hub75_pins {
    ($p:expr) => {{
        use ::esp_hal::gpio::Pin as _;

        ::esp_hub75::Hub75Pins8 {
            red1: ($p).GPIO16.degrade(),
            grn1: ($p).GPIO4.degrade(),
            blu1: ($p).GPIO17.degrade(),
            red2: ($p).GPIO18.degrade(),
            grn2: ($p).GPIO5.degrade(),
            blu2: ($p).GPIO19.degrade(),
            blank: ($p).GPIO26.degrade(),
            clock: ($p).GPIO25.degrade(),
            latch: ($p).GPIO2.degrade(),
        }
    }};
}

#[cfg(all(esp32c6, feature = "bus8"))]
#[macro_export]
macro_rules! hub75_pins {
    ($p:expr) => {{
        use ::esp_hal::gpio::Pin as _;

        ::esp_hub75::Hub75Pins8 {
            red1: ($p).GPIO10.degrade(),
            grn1: ($p).GPIO8.degrade(),
            blu1: ($p).GPIO1.degrade(),
            red2: ($p).GPIO0.degrade(),
            grn2: ($p).GPIO11.degrade(),
            blu2: ($p).GPIO7.degrade(),
            blank: ($p).GPIO21.degrade(),
            clock: ($p).GPIO19.degrade(),
            latch: ($p).GPIO18.degrade(),
        }
    }};
}

#[cfg(all(esp32c5, feature = "bus8"))]
#[macro_export]
macro_rules! hub75_pins {
    ($p:expr) => {{
        use ::esp_hal::gpio::Pin as _;

        ::esp_hub75::Hub75Pins8 {
            red1: ($p).GPIO9.degrade(),
            grn1: ($p).GPIO8.degrade(),
            blu1: ($p).GPIO7.degrade(),
            red2: ($p).GPIO6.degrade(),
            grn2: ($p).GPIO10.degrade(),
            blu2: ($p).GPIO1.degrade(),
            blank: ($p).GPIO27.degrade(),
            clock: ($p).GPIO5.degrade(),
            latch: ($p).GPIO26.degrade(),
        }
    }};
}

// ---------------------------------------------------------------------------
// Backend selection
// ---------------------------------------------------------------------------

/// Picks the parallel-output peripheral and its DMA channel.
///
/// ESP32-S3 drives the panel from `LCD_CAM`; both RISC-V parts from
/// `PARL_IO`, each with its single parallel-output DMA channel. The ESP32 has
/// two I2S peripherals and needs both: `I2S0` for the 16-bit wiring (its only
/// 16-bit-capable parallel peripheral) and `I2S1` for the 8-bit one, because
/// `I2S0` drops every odd byte.
///
/// Exactly one bus width is always enabled (`src/lib.rs` enforces it), so the
/// arms that do not depend on it are not split by feature.
#[cfg(esp32s3)]
#[macro_export]
macro_rules! hub75_backend {
    ($p:expr) => {
        (($p).LCD_CAM, ($p).DMA_CH0)
    };
}

#[cfg(all(esp32, feature = "bus16"))]
#[macro_export]
macro_rules! hub75_backend {
    ($p:expr) => {
        (($p).I2S0, ($p).DMA_I2S0)
    };
}

#[cfg(all(esp32, feature = "bus8"))]
#[macro_export]
macro_rules! hub75_backend {
    ($p:expr) => {
        (($p).I2S1, ($p).DMA_I2S1)
    };
}

#[cfg(esp32c6)]
#[macro_export]
macro_rules! hub75_backend {
    ($p:expr) => {
        (($p).PARL_IO, ($p).DMA_CH0)
    };
}

/// `bus16` on the ESP32-C5 is rejected in `src/lib.rs`, so this arm only ever
/// sees the 8-bit latched configuration.
#[cfg(esp32c5)]
#[macro_export]
macro_rules! hub75_backend {
    ($p:expr) => {
        (($p).PARL_IO, ($p).DMA_CH0)
    };
}

// ---------------------------------------------------------------------------
// Latched boards' brightness input (bus8)
// ---------------------------------------------------------------------------

/// Drives the latched boards' OE/brightness input high, the level the
/// `gradient-latched` example uses and the one such panels are wired for.
///
/// The pin is not an input to the driver -- it only scales panel brightness --
/// so no assertion depends on it; it is driven anyway so that the bench
/// behaves like the example. Direct-drive boards have no such pin, where this
/// expands to `()`.
#[cfg(all(esp32, feature = "bus8"))]
#[macro_export]
macro_rules! oe_pin {
    ($p:expr) => {{
        ::esp_hal::gpio::Output::new(
            ($p).GPIO27,
            ::esp_hal::gpio::Level::High,
            ::esp_hal::gpio::OutputConfig::default(),
        )
    }};
}

#[cfg(all(esp32s3, feature = "bus8"))]
#[macro_export]
macro_rules! oe_pin {
    ($p:expr) => {{
        ::esp_hal::gpio::Output::new(
            ($p).GPIO48,
            ::esp_hal::gpio::Level::High,
            ::esp_hal::gpio::OutputConfig::default(),
        )
    }};
}

#[cfg(all(esp32c6, feature = "bus8"))]
#[macro_export]
macro_rules! oe_pin {
    ($p:expr) => {{
        ::esp_hal::gpio::Output::new(
            ($p).GPIO20,
            ::esp_hal::gpio::Level::High,
            ::esp_hal::gpio::OutputConfig::default(),
        )
    }};
}

#[cfg(all(esp32c5, feature = "bus8"))]
#[macro_export]
macro_rules! oe_pin {
    ($p:expr) => {{
        ::esp_hal::gpio::Output::new(
            ($p).GPIO4,
            ::esp_hal::gpio::Level::High,
            ::esp_hal::gpio::OutputConfig::default(),
        )
    }};
}

/// See the `bus8` variants of [`oe_pin!`]: a direct-drive board has no latch
/// hardware whose brightness could be scaled, so there is no pin to drive.
///
/// [`oe_pin!`]: crate::oe_pin
#[cfg(not(feature = "bus8"))]
#[macro_export]
macro_rules! oe_pin {
    ($p:expr) => {
        ()
    };
}
