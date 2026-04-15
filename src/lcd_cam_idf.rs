//! ESP32-S3 / ESP32-P4 HUB75 driver using the ESP-IDF I80 (Intel 8080) LCD
//! peripheral.
//!
//! This module drives HUB75 panels via the ESP-IDF's `esp_lcd_panel_io_i80`
//! C API, which configures the LCD_CAM peripheral and its built-in GDMA
//! channel automatically. No manual DMA descriptor management is required.
//!
//! Supported chips (via the corresponding Cargo feature):
//! - `esp32s3-idf` — target `xtensa-esp32s3-espidf`
//! - `esp32p4-idf` — target `riscv32imafc-esp-espidf`
//!
//! # Single-instance constraint
//!
//! A static atomic flag and a static transfer-done signal are used for GDMA
//! ISR coordination. Only one [`Hub75`] instance may exist at a time; the
//! constructor enforces this with a runtime check.
//!
//! # Memory
//!
//! The framebuffer must reside in internal SRAM. GDMA cannot access PSRAM
//! directly. This is the same constraint as the bare-metal `esp32s3` backend.

use core::sync::atomic::AtomicBool;
use core::sync::atomic::AtomicI32;
use core::sync::atomic::Ordering;

use esp_idf_sys::esp_err_t;
use esp_idf_sys::esp_lcd_del_i80_bus;
use esp_idf_sys::esp_lcd_i80_bus_config_t;
use esp_idf_sys::esp_lcd_i80_bus_handle_t;
use esp_idf_sys::esp_lcd_new_i80_bus;
use esp_idf_sys::esp_lcd_new_panel_io_i80;
use esp_idf_sys::esp_lcd_panel_io_del;
use esp_idf_sys::esp_lcd_panel_io_handle_t;
use esp_idf_sys::esp_lcd_panel_io_i80_config_t;
use esp_idf_sys::esp_lcd_panel_io_tx_color;
use esp_idf_sys::ESP_OK;

use crate::framebuffer::FrameBuffer;
use crate::Hub75Error;
use crate::Hub75IdfPins;
use crate::Hub75IdfPins16;
use crate::Hub75IdfPins8;

/// GPIO_NUM_NC — "no connect" sentinel used by the IDF I80 bus for unused data
/// lines.
const GPIO_NUM_NC: i32 = -1;

/// Base GPIO-matrix signal index for LCD_DATA_OUT[0].
/// Data-line signals are contiguous: DATA[N] uses signal LCD_DATA_OUT0_SIGNAL +
/// N.
///
/// ESP32-S3: LCD_DATA_OUT0_IDX = 133 (soc/gpio_sig_map.h)
/// ESP32-P4: LCD_DATA_OUT_PAD_OUT0_IDX = 154 (soc/gpio_sig_map.h)
#[cfg(feature = "esp32s3-idf")]
const LCD_DATA_OUT0_SIGNAL: u32 = 133;
#[cfg(feature = "esp32p4-idf")]
const LCD_DATA_OUT0_SIGNAL: u32 = 154;

// esp_rom_gpio_connect_out_signal is a ROM function present on all ESP32 chips.
// It is not exposed through the esp-idf-sys bindings.h wrapper, so we declare
// it directly. The linker resolves it via the IDF ROM linker scripts.
extern "C" {
    fn esp_rom_gpio_connect_out_signal(
        gpio_num: u32,
        signal_idx: u32,
        out_inv: bool,
        oen_inv: bool,
    );
}

// ---- Static state (shared by both S3-IDF and P4-IDF) ----

/// Flag set by the GDMA ISR callback when a color transfer completes.
static TRANSFER_DONE: AtomicBool = AtomicBool::new(false);

/// Guard ensuring at most one [`Hub75`] instance exists.
static INSTANCE_TAKEN: AtomicBool = AtomicBool::new(false);

/// BLANK GPIO number stored for use by the shutdown handler (-1 = none).
static BLANK_GPIO_NUM: AtomicI32 = AtomicI32::new(-1);

/// Shutdown handler registered with ESP-IDF via
/// `esp_register_shutdown_handler`.
///
/// Drives the BLANK (OE) GPIO HIGH via the simple GPIO output path (bypassing
/// the LCD peripheral) so the panel turns off before `esp_restart()` executes.
/// This runs for software-triggered resets (OTA, watchdog, explicit restart).
///
/// Hard resets from a flashing tool (RTS/DTR toggle) cannot be caught in
/// software; a pull-up resistor on the OE line is recommended for those.
unsafe extern "C" fn blank_on_shutdown() {
    let gpio = BLANK_GPIO_NUM.load(Ordering::Relaxed);
    if gpio >= 0 {
        // Reconfigure as simple GPIO output and drive HIGH.
        // HIGH on the BLANK pad = OE HIGH = display off (panel active-low OE).
        // This bypasses both the LCD peripheral and the inv_sel inversion.
        esp_idf_sys::gpio_set_direction(gpio, esp_idf_sys::gpio_mode_t_GPIO_MODE_OUTPUT);
        esp_idf_sys::gpio_set_level(gpio, 1);
    }
}

/// Callback invoked by the IDF GDMA ISR when a color transfer finishes.
///
/// # Safety
/// Called from interrupt context; only ISR-safe operations are performed.
unsafe extern "C" fn on_color_trans_done(
    _io: esp_lcd_panel_io_handle_t,
    _event_data: *mut esp_idf_sys::esp_lcd_panel_io_event_data_t,
    _user_ctx: *mut core::ffi::c_void,
) -> bool {
    TRANSFER_DONE.store(true, Ordering::Release);
    false // returning false: no higher-priority task woken (FreeRTOS
          // convention)
}

/// Convert an `esp_err_t` to `Result<(), Hub75Error>`.
#[inline]
fn esp_check(ret: esp_err_t) -> Result<(), Hub75Error> {
    if ret == ESP_OK {
        Ok(())
    } else {
        // SAFETY: ret != ESP_OK means ret is non-zero
        Err(Hub75Error::Idf(unsafe {
            esp_idf_sys::EspError::from_non_zero(core::num::NonZeroI32::new_unchecked(ret))
        }))
    }
}

/// HUB75 LED matrix display driver for ESP32-S3 / ESP32-P4 running ESP-IDF.
///
/// Uses the ESP-IDF I80 (Intel 8080) LCD peripheral with automatic GDMA.
/// Obtain an instance via [`Hub75::new`].
///
/// See the crate-level documentation for usage examples.
pub struct Hub75 {
    io_handle: esp_lcd_panel_io_handle_t,
    bus_handle: esp_lcd_i80_bus_handle_t,
    blank_gpio: i32,
}

impl Hub75 {
    /// Creates a new HUB75 driver instance.
    ///
    /// # Arguments
    /// * `pins` — Pin configuration implementing [`Hub75IdfPins`]
    /// * `frequency_hz` — Pixel clock frequency in Hz (e.g. `20_000_000` for 20
    ///   MHz)
    /// * `max_transfer_bytes` — Maximum single transfer size in bytes; must be
    ///   at least as large as the framebuffer. Use
    ///   `core::mem::size_of::<FBType>()` to compute this.
    ///
    /// # Errors
    /// Returns [`Hub75Error::Idf`] if the ESP-IDF driver cannot be configured.
    ///
    /// # Panics
    /// Panics if a second instance is created while the first is still alive.
    pub fn new(
        pins: impl Hub75IdfPins,
        frequency_hz: u32,
        max_transfer_bytes: usize,
    ) -> Result<Self, Hub75Error> {
        assert!(
            !INSTANCE_TAKEN.swap(true, Ordering::Acquire),
            "Only one Hub75 instance may exist at a time"
        );

        let data_gpios = pins.data_gpios();
        let bus_width = pins.bus_width();

        // --- I80 bus ---
        let mut bus_config: esp_lcd_i80_bus_config_t = unsafe { core::mem::zeroed() };
        // The IDF I80 bus driver requires a D/C GPIO even though HUB75 panels
        // have no such signal. The user supplies any spare GPIO via
        // Hub75IdfPins::dc_gpio(). The driver will drive it HIGH during every
        // color transfer; the panel ignores it.
        bus_config.dc_gpio_num = pins.dc_gpio();
        bus_config.wr_gpio_num = pins.wr_gpio();
        // Must be set explicitly; zeroed() leaves clk_src = 0 = SOC_MOD_CLK_INVALID
        bus_config.clk_src = esp_idf_sys::soc_periph_lcd_clk_src_t_LCD_CLK_SRC_DEFAULT;
        bus_config.bus_width = bus_width;
        bus_config.max_transfer_bytes = max_transfer_bytes;
        // Copy data GPIO numbers into the fixed-size C array
        // SAFETY: data_gpio_nums is [i32; 16] (SOC_LCD_I80_BUS_WIDTH=16 on ESP32-S3 and
        // ESP32-P4)
        for (i, &gpio) in data_gpios.iter().enumerate() {
            bus_config.data_gpio_nums[i] = gpio;
        }

        let mut bus_handle: esp_lcd_i80_bus_handle_t = core::ptr::null_mut();
        esp_check(unsafe { esp_lcd_new_i80_bus(&bus_config, &mut bus_handle) })?;

        // --- Panel IO on the bus ---
        let mut io_config: esp_lcd_panel_io_i80_config_t = unsafe { core::mem::zeroed() };
        io_config.cs_gpio_num = GPIO_NUM_NC; // no CS for HUB75
        io_config.pclk_hz = frequency_hz;
        io_config.trans_queue_depth = 1;
        io_config.on_color_trans_done = Some(on_color_trans_done);
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        // pclk_active_neg controls clock polarity.
        // Default (0): data is latched on rising edge of WR — standard for HUB75.
        // Set to 1 (via the `invert-clock` feature) if your panel requires the
        // opposite polarity. The exact bindgen accessor is:
        //   unsafe { io_config.flags.set_pclk_active_neg(1) }
        // Verify the generated field name against the esp-idf-sys bindings for
        // your IDF version before enabling.
        #[cfg(feature = "invert-clock")]
        unsafe {
            io_config.flags.set_pclk_active_neg(1);
        }

        let mut io_handle: esp_lcd_panel_io_handle_t = core::ptr::null_mut();
        let ret = unsafe { esp_lcd_new_panel_io_i80(bus_handle, &io_config, &mut io_handle) };
        if let Err(e) = esp_check(ret) {
            // Clean up the already-created bus before returning
            unsafe { esp_lcd_del_i80_bus(bus_handle) };
            INSTANCE_TAKEN.store(false, Ordering::Release);
            return Err(e);
        }

        // Apply hardware output inversion to BLANK (OE) if required.
        // We use esp_rom_gpio_connect_out_signal() (a ROM function) rather than
        // a direct register write so that the correct chip-specific signal
        // routing path is used. Direct writes to GPIO_FUNCn_OUT_SEL_CFG can
        // trigger hard faults on ESP32-P4 due to memory-protection constraints.
        //
        // The IDF I80 bus driver has already routed DATA[N] signals to the GPIO
        // pads (with out_inv=false). We find which data slot the BLANK GPIO
        // occupies and re-route that same signal with out_inv=true.
        let blank_gpio = pins.blank_gpio();
        if pins.invert_blank() {
            if let Some(slot) = data_gpios.iter().position(|&g| g == blank_gpio) {
                unsafe {
                    esp_rom_gpio_connect_out_signal(
                        blank_gpio as u32,
                        LCD_DATA_OUT0_SIGNAL + slot as u32,
                        true,  // out_inv: invert the output signal
                        false, // oen_inv: do not invert output-enable
                    );
                }
            }
        }

        // Register a shutdown handler so the panel is blanked before any
        // software-triggered reset (OTA, esp_restart(), watchdog, panic).
        BLANK_GPIO_NUM.store(blank_gpio, Ordering::Relaxed);
        unsafe { esp_idf_sys::esp_register_shutdown_handler(Some(blank_on_shutdown)) };

        Ok(Self {
            io_handle,
            bus_handle,
            blank_gpio,
        })
    }

    /// Renders a framebuffer to the display via GDMA.
    ///
    /// Consumes `self` and returns a [`Hub75Transfer`] handle. Call
    /// [`Hub75Transfer::wait`] (blocking) or
    /// [`Hub75Transfer::wait_for_done`] (async) to retrieve the driver
    /// after the transfer completes.
    ///
    /// # Errors
    /// Returns a tuple of `(Hub75Error, Hub75)` so the driver can be recovered
    /// on error.
    pub fn render<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    >(
        self,
        fb: &impl FrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>,
    ) -> Result<Hub75Transfer, (Hub75Error, Self)> {
        // Reset the done flag before initiating the transfer
        TRANSFER_DONE.store(false, Ordering::Release);

        let (ptr, len) = unsafe { fb.read_buffer() };

        // lcd_cmd = -1: the I80 driver treats this as a raw color transfer with
        // no preceding command phase, which is what HUB75 expects.
        let ret = unsafe {
            esp_lcd_panel_io_tx_color(self.io_handle, -1, ptr as *const core::ffi::c_void, len)
        };

        match esp_check(ret) {
            Ok(()) => Ok(Hub75Transfer { hub75: self }),
            Err(e) => Err((e, self)),
        }
    }
}

impl Drop for Hub75 {
    fn drop(&mut self) {
        // Blank the display before tearing down the LCD peripheral so the
        // panel goes dark rather than showing garbage during the transition.
        unsafe {
            esp_idf_sys::gpio_set_direction(
                self.blank_gpio,
                esp_idf_sys::gpio_mode_t_GPIO_MODE_OUTPUT,
            );
            esp_idf_sys::gpio_set_level(self.blank_gpio, 1);
        }

        unsafe {
            esp_idf_sys::esp_unregister_shutdown_handler(Some(blank_on_shutdown));
            esp_lcd_panel_io_del(self.io_handle);
            esp_lcd_del_i80_bus(self.bus_handle);
        }
        BLANK_GPIO_NUM.store(-1, Ordering::Relaxed);
        INSTANCE_TAKEN.store(false, Ordering::Release);
    }
}

/// An in-progress GDMA transfer to the HUB75 display.
///
/// Returned by [`Hub75::render`]. Use [`wait`](Hub75Transfer::wait) or
/// [`wait_for_done`](Hub75Transfer::wait_for_done) to block until the
/// transfer completes and recover the [`Hub75`] driver.
pub struct Hub75Transfer {
    hub75: Hub75,
}

impl Hub75Transfer {
    /// Returns `true` if the GDMA transfer has completed.
    pub fn is_done(&self) -> bool {
        TRANSFER_DONE.load(Ordering::Acquire)
    }

    /// Blocks until the GDMA transfer completes, then returns the driver.
    ///
    /// Yields to the FreeRTOS scheduler between polls so other tasks can run.
    pub fn wait(self) -> (Result<(), Hub75Error>, Hub75) {
        while !self.is_done() {
            // Yield one FreeRTOS tick to allow other tasks to run.
            // SAFETY: vTaskDelay is safe to call from any FreeRTOS task.
            unsafe { esp_idf_sys::vTaskDelay(1) };
        }
        (Ok(()), self.hub75)
    }

    /// Asynchronously waits for the GDMA transfer to complete.
    ///
    /// Polls the transfer-done flag and yields to the async executor between
    /// checks. Compatible with any Rust async executor.
    ///
    /// After this returns, call [`wait`](Hub75Transfer::wait) to retrieve the
    /// [`Hub75`] driver — it will not block at that point.
    pub async fn wait_for_done(&mut self) -> Result<(), Hub75Error> {
        core::future::poll_fn(|cx| {
            if self.is_done() {
                core::task::Poll::Ready(())
            } else {
                // Re-schedule this future on the next executor poll.
                cx.waker().wake_by_ref();
                core::task::Poll::Pending
            }
        })
        .await;
        Ok(())
    }
}

// --- Hub75IdfPins implementations ---

impl Hub75IdfPins for Hub75IdfPins16 {
    fn bus_width(&self) -> usize {
        16
    }

    fn wr_gpio(&self) -> i32 {
        self.clock
    }

    fn dc_gpio(&self) -> i32 {
        self.dc
    }

    fn blank_gpio(&self) -> i32 {
        self.blank
    }

    /// 16-bit mode always inverts BLANK: the framebuffer drives DATA[8] high
    /// when the display should be active; the panel's OE input is active-low.
    fn invert_blank(&self) -> bool {
        true
    }

    /// Maps HUB75 signals to DATA[0..15] to mirror the bare-metal lcd_cam.rs:
    /// DATA[0..4] = addr0-4, DATA[5] = latch, DATA[6..7] = unused,
    /// DATA[8] = blank, DATA[9..14] = R1/G1/B1/R2/G2/B2, DATA[15] = unused.
    ///
    /// The IDF I80 bus driver requires every slot within `bus_width` to be a
    /// valid output GPIO — it rejects -1 (GPIO_NUM_NC). The three unused slots
    /// are wired to the `dc` pin (already a no-connect dummy), which the
    /// driver will configure as an output; the panel ignores all of them.
    fn data_gpios(&self) -> [i32; 16] {
        [
            self.addr0, // DATA[0]
            self.addr1, // DATA[1]
            self.addr2, // DATA[2]
            self.addr3, // DATA[3]
            self.addr4, // DATA[4]
            self.latch, // DATA[5]
            self.dc,    // DATA[6] unused — routed to dc dummy GPIO
            self.dc,    // DATA[7] unused — routed to dc dummy GPIO
            self.blank, // DATA[8]
            self.red1,  // DATA[9]
            self.grn1,  // DATA[10]
            self.blu1,  // DATA[11]
            self.red2,  // DATA[12]
            self.grn2,  // DATA[13]
            self.blu2,  // DATA[14]
            self.dc,    // DATA[15] unused — routed to dc dummy GPIO
        ]
    }
}

impl Hub75IdfPins for Hub75IdfPins8 {
    fn bus_width(&self) -> usize {
        8
    }

    fn wr_gpio(&self) -> i32 {
        self.clock
    }

    fn dc_gpio(&self) -> i32 {
        self.dc
    }

    fn blank_gpio(&self) -> i32 {
        self.blank
    }

    /// 8-bit mode respects the `invert-blank` feature (some controller boards
    /// have an external inverter and need the signal non-inverted here).
    fn invert_blank(&self) -> bool {
        cfg!(feature = "invert-blank")
    }

    /// Maps HUB75 signals to DATA[0..7]:
    /// DATA[0..5] = R1/G1/B1/R2/G2/B2, DATA[6] = latch, DATA[7] = blank.
    fn data_gpios(&self) -> [i32; 16] {
        [
            self.red1,   // DATA[0]
            self.grn1,   // DATA[1]
            self.blu1,   // DATA[2]
            self.red2,   // DATA[3]
            self.grn2,   // DATA[4]
            self.blu2,   // DATA[5]
            self.latch,  // DATA[6]
            self.blank,  // DATA[7]
            GPIO_NUM_NC, // DATA[8..15] unused
            GPIO_NUM_NC,
            GPIO_NUM_NC,
            GPIO_NUM_NC,
            GPIO_NUM_NC,
            GPIO_NUM_NC,
            GPIO_NUM_NC,
            GPIO_NUM_NC,
        ]
    }
}
