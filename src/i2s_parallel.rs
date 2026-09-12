//! HUB75 driver for I2S Parallel peripherals (ESP32).
//!
//! The I2S DMA `out_total_eof` interrupt runs the BCM loop in both refresh
//! modes: linear chains end with `suc_eof` + `NULL` next, and a circular
//! chain ends the same way while a swap has armed the pass-boundary
//! detector. The loop streams the current framebuffer to the panel
//! continuously, and a buffer swap takes effect at a frame boundary.
//!
//! ## Blocking Example
//!
//! ```rust,ignore
//! let hub75 = Hub75::new(
//!     peripherals.I2S0, pins, peripherals.DMA_I2S0,
//!     tx_descriptors, Hub75Config::new(), &*fb,
//! ).expect("failed to create Hub75");
//!
//! // Display refreshes on its own; the main thread is free.
//! loop { core::hint::spin_loop(); }
//! ```
//!
//! ## Async Example
//!
//! ```rust,ignore
//! let hub75 = Hub75::new_async(
//!     peripherals.I2S0, pins, peripherals.DMA_I2S0,
//!     tx_descriptors, Hub75Config::new(), &*fb0,
//! ).expect("failed to create Hub75");
//!
//! // Swap buffers; `wait()` spin-loops until the DMA no longer reads the
//! // old framebuffer. Use `wait_for_done().await` first to yield instead.
//! let old_fb = hub75.swap(fb1)?.wait().expect("DMA error");
//! ```

use esp_hal::Blocking;
use esp_hal::gpio::AnyPin;
use esp_hal::gpio::NoPin;
use esp_hal::i2s::parallel::I2sParallel;
use esp_hal::i2s::parallel::I2sParallelDmaChannel;
use esp_hal::i2s::parallel::Instance;
use esp_hal::i2s::parallel::TxEightBits;
use esp_hal::i2s::parallel::TxPins;
use esp_hal::i2s::parallel::TxSixteenBits;
use esp_hal::interrupt::InterruptHandler;
use esp_hal::peripherals::Interrupt;

use crate::Hub75Config;
use crate::Hub75DmaDescriptors;
// ---------------------------------------------------------------------------
// Interrupt binding for the I2S peripheral (ESP32)
// ---------------------------------------------------------------------------

/// The `I2S` peripheral instance the driver was constructed with (`0` = none,
/// `1` = `I2S0`, `2` = `I2S1`). Written once by the constructor, before the
/// ISR is enabled, and read by the ISR to clear the frame-boundary flag on
/// the correct peripheral.
static SELECTED_I2S: AtomicU8 = AtomicU8::new(0);

/// Binds the refresh ISR to the `I2S` peripheral interrupt and enables the
/// `out_total_eof` source.
///
/// This trait is internal to the driver and is not part of the public API. The
/// constructor calls it on the concrete `I2S0`/`I2S1` peripheral passed to
/// [`Hub75::new`](crate::Hub75).
#[doc(hidden)]
pub trait I2sInterruptBinding: Instance {
    /// Binds `handler` to this peripheral's CPU interrupt (with the handler's
    /// priority), enables the `out_total_eof` source, and records this
    /// instance as the driver's `I2S` peripheral.
    fn bind_and_enable_isr(handler: InterruptHandler);

    /// Clears the `out_total_eof` flag on this peripheral. The driver calls it
    /// only in circular-DMA mode.
    #[cfg_attr(not(feature = "circular-dma"), allow(dead_code))]
    fn clear_frame_interrupt();
}

impl I2sInterruptBinding for esp_hal::peripherals::I2S0<'_> {
    fn bind_and_enable_isr(handler: InterruptHandler) {
        SELECTED_I2S.store(1, Ordering::Relaxed);
        // SAFETY: This runs in the constructor before the ISR is enabled and
        // before any transfer starts; the peripheral handle is owned by the
        // caller and the stolen handle is only used for register access, so
        // there is no concurrent access.
        unsafe {
            esp_hal::interrupt::bind_handler(Interrupt::I2S0, handler);
            esp_hal::peripherals::I2S0::steal()
                .register_block()
                .int_ena()
                .modify(|_, w| w.out_total_eof().set_bit());
        }
    }

    fn clear_frame_interrupt() {
        // SAFETY: See the `mark`/enable path above: the stolen handle is only
        // used for the interrupt-clear register write from ISR context, and
        // the driver never touches this register elsewhere.
        unsafe {
            esp_hal::peripherals::I2S0::steal()
                .register_block()
                .int_clr()
                .write(|w| w.out_total_eof().clear_bit_by_one());
        }
    }
}

impl I2sInterruptBinding for esp_hal::peripherals::I2S1<'_> {
    fn bind_and_enable_isr(handler: InterruptHandler) {
        SELECTED_I2S.store(2, Ordering::Relaxed);
        // SAFETY: Same justification as the `I2S0` implementation: the stolen
        // handle is only used to bind the handler and enable `out_total_eof`
        // during construction, while no transfer is running.
        unsafe {
            esp_hal::interrupt::bind_handler(Interrupt::I2S1, handler);
            esp_hal::peripherals::I2S1::steal()
                .register_block()
                .int_ena()
                .modify(|_, w| w.out_total_eof().set_bit());
        }
    }

    fn clear_frame_interrupt() {
        // SAFETY: See the `I2S0` implementation: the stolen handle is only
        // used for the interrupt-clear register write from ISR context, and
        // the driver never touches this register elsewhere.
        unsafe {
            esp_hal::peripherals::I2S1::steal()
                .register_block()
                .int_clr()
                .write(|w| w.out_total_eof().clear_bit_by_one());
        }
    }
}

/// Clears the `out_total_eof` flag on the `I2S` peripheral the driver was
/// constructed with.
///
/// The boundary flag is `out_total_eof`: the armed chain ends with
/// `suc_eof` + `NULL` next, exactly like a linear transfer, and the
/// free-running (disarmed) ring never ends, so the flag can only be
/// latched at an armed pass boundary.
#[cfg_attr(not(feature = "circular-dma"), allow(dead_code))]
pub(crate) fn clear_frame_interrupt() {
    match SELECTED_I2S.load(Ordering::Relaxed) {
        1 => <esp_hal::peripherals::I2S0<'_> as I2sInterruptBinding>::clear_frame_interrupt(),
        2 => <esp_hal::peripherals::I2S1<'_> as I2sInterruptBinding>::clear_frame_interrupt(),
        _ => {}
    }
}

use core::sync::atomic::AtomicU8;
use core::sync::atomic::Ordering;

use crate::Hub75;
use crate::Hub75Error;
use crate::Hub75Pins;
use crate::Hub75Pins8;
use crate::Hub75Pins16;
use crate::framebuffer::WordSize;
use crate::isr::BcmBuf;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

impl<DM: esp_hal::DriverMode, FB: crate::framebuffer::FrameBuffer + 'static> Hub75<DM, FB> {
    fn new_internal<
        T: TxPins<'static> + 'static,
        P: Hub75Pins<Word = FB::Word> + I2sPins<'static, T>,
        I: Instance + I2sInterruptBinding + 'static,
        const N: usize,
    >(
        i2s: I,
        hub75_pins: P,
        channel: impl I2sParallelDmaChannel<'static, I>,
        tx_descriptors: &'static mut Hub75DmaDescriptors<FB, N>,
        config: Hub75Config,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        crate::isr::claim_driver()?;
        crate::bcm::validate_fb_internal_ram(fb);

        let (pins, clock_pin) = hub75_pins.convert_pins();

        // By default data changes on the falling edge of CLK so it is stable
        // when the panel latches on the rising edge. The ESP32 I2S peripheral
        // shifts on the rising edge, so we invert the clock output unless the
        // user opted into the opposite polarity.
        #[cfg(not(feature = "invert-clock"))]
        let clock_pin = clock_pin.into_output_signal().with_output_inverter(true);

        let i2s_parallel = I2sParallel::new(i2s, channel, config.frequency, pins, clock_pin);

        // This connects `isr` to the interrupt. Both refresh modes use the
        // same source, `out_total_eof` (the DMA finished the descriptor
        // chain):
        //
        // - Linear: every chain ends with `suc_eof` + `NULL` next, so `out_total_eof`
        //   fires at every segment-group boundary and runs the BCM loop.
        // - Circular: the free-running ring carries no `suc_eof` and never ends, so
        //   `out_total_eof` never fires in steady state. A swap arms the boundary
        //   detector by relinking the second-to-last ring descriptor to the spare
        //   boundary descriptor (`suc_eof` + `NULL` next) — from the DMA's point of
        //   view that is a normal end-of-transfer, so `out_total_eof` fires exactly at
        //   the armed pass boundary.
        //
        // Binding also enables the CPU interrupt with the handler's priority.
        I::bind_and_enable_isr(crate::isr::handler_with_priority(
            crate::isr::isr,
            config.interrupt_priority,
        ));

        let buf = BcmBuf::new(tx_descriptors.as_slice());
        crate::isr::init_state(i2s_parallel, buf);
        crate::isr::start_internal(fb)?;

        Ok(Self::from_phantom())
    }
}

impl<FB: crate::framebuffer::FrameBuffer + 'static> Hub75<Blocking, FB> {
    /// Creates a new blocking HUB75 driver.
    ///
    /// Configures the I2S peripheral, applies pin assignments, and
    /// immediately starts DMA-driven display refresh with the provided
    /// framebuffer.
    ///
    /// The pin configuration's word type must match the framebuffer's word
    /// type; passing a 16-bit framebuffer with 8-bit pins (or vice versa)
    /// is a compile-time error.
    ///
    /// Takes the I2S peripheral instance (I2S0 or I2S1), the HUB75 pin
    /// configuration (8-bit or 16-bit), a DMA channel (`DMA_I2S0` or
    /// `DMA_I2S1`), DMA descriptor storage from [`hub75_dma_descriptors!`],
    /// the I2S clock rate, and the initial framebuffer to display.
    ///
    /// # Errors
    ///
    /// Returns [`Hub75Error::AlreadyInitialised`] if a `Hub75` instance
    /// already exists. Returns [`Hub75Error::AlreadyRunning`] or
    /// [`Hub75Error::Dma`] if the initial DMA transfer fails.
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new<
        T: TxPins<'static> + 'static,
        P: Hub75Pins<Word = FB::Word> + I2sPins<'static, T>,
        I: Instance + I2sInterruptBinding + 'static,
        const N: usize,
    >(
        i2s: I,
        hub75_pins: P,
        channel: impl I2sParallelDmaChannel<'static, I>,
        tx_descriptors: &'static mut Hub75DmaDescriptors<FB, N>,
        config: Hub75Config,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(i2s, hub75_pins, channel, tx_descriptors, config, fb)
    }
}

impl<FB: crate::framebuffer::FrameBuffer + 'static> Hub75<esp_hal::Async, FB> {
    /// Creates a new async HUB75 driver.
    ///
    /// Configures the I2S peripheral, applies pin assignments, and
    /// immediately starts DMA-driven display refresh with the provided
    /// framebuffer.
    ///
    /// The pin configuration's word type must match the framebuffer's word
    /// type; passing a 16-bit framebuffer with 8-bit pins (or vice versa)
    /// is a compile-time error.
    ///
    /// Takes the I2S peripheral instance (I2S0 or I2S1), the HUB75 pin
    /// configuration (8-bit or 16-bit), a DMA channel (`DMA_I2S0` or
    /// `DMA_I2S1`), DMA descriptor storage from [`hub75_dma_descriptors!`],
    /// the I2S clock rate, and the initial framebuffer to display.
    ///
    /// # Errors
    ///
    /// Returns [`Hub75Error::AlreadyInitialised`] if a `Hub75` instance
    /// already exists. Returns [`Hub75Error::AlreadyRunning`] or
    /// [`Hub75Error::Dma`] if the initial DMA transfer fails.
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new_async<
        T: TxPins<'static> + 'static,
        P: Hub75Pins<Word = FB::Word> + I2sPins<'static, T>,
        I: Instance + I2sInterruptBinding + 'static,
        const N: usize,
    >(
        i2s: I,
        hub75_pins: P,
        channel: impl I2sParallelDmaChannel<'static, I>,
        tx_descriptors: &'static mut Hub75DmaDescriptors<FB, N>,
        config: Hub75Config,
        fb: &'static FB,
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(i2s, hub75_pins, channel, tx_descriptors, config, fb)
    }
}

// ---------------------------------------------------------------------------
// Pin configurations
// ---------------------------------------------------------------------------

impl crate::Hub75Pins for Hub75Pins16<'_> {
    type Word = u16;

    fn word_size(&self) -> WordSize {
        WordSize::Sixteen
    }
}

impl crate::Hub75Pins for Hub75Pins8<'_> {
    type Word = u8;

    fn word_size(&self) -> WordSize {
        WordSize::Eight
    }
}

/// Converts a HUB75 pin configuration into the I2S parallel pin format.
///
/// This trait is internal to the driver and is not part of the public API.
#[doc(hidden)]
pub trait I2sPins<'d, T> {
    /// Converts the high-level pin definition into the peripheral-specific
    /// format, returning the converted pins and the clock pin.
    fn convert_pins(self) -> (T, AnyPin<'d>);
}

impl<'d> I2sPins<'d, TxSixteenBits<'d>> for Hub75Pins16<'d> {
    fn convert_pins(self) -> (TxSixteenBits<'d>, AnyPin<'d>) {
        let blank = self.blank.into_output_signal();
        #[cfg(feature = "invert-blank")]
        let blank = blank.with_output_inverter(true);

        let pins = TxSixteenBits::new(
            self.addr0, self.addr1, self.addr2, self.addr3, self.addr4, self.latch, NoPin, NoPin,
            blank, self.red1, self.grn1, self.blu1, self.red2, self.grn2, self.blu2, NoPin,
        );
        (pins, self.clock)
    }
}

impl<'d> I2sPins<'d, TxEightBits<'d>> for Hub75Pins8<'d> {
    fn convert_pins(self) -> (TxEightBits<'d>, AnyPin<'d>) {
        let blank = self.blank.into_output_signal();
        #[cfg(feature = "invert-blank")]
        let blank = blank.with_output_inverter(true);

        let pins = TxEightBits::new(
            self.red1, self.grn1, self.blu1, self.red2, self.grn2, self.blu2, self.latch, blank,
        );
        (pins, self.clock)
    }
}
