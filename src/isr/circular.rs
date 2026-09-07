//! Circular-DMA refresh state machine.
//!
//! A single circular descriptor chain encodes the full BCM repetition
//! sequence; the DMA engine starts once and loops forever with no interrupts
//! enabled in steady state. Buffer swaps arm a pass-boundary detector and the
//! boundary ISR applies a pointer delta to the descriptors at the boundary,
//! then disarms again:
//!
//! - Arming relinks the second-to-last ring descriptor to the spare boundary
//!   descriptor (`bcm::circular::arm_boundary`), which carries `suc_eof` and a
//!   `NULL` next pointer — the chain ends at the pass boundary exactly like a
//!   normal end-of-transfer on every backend (ESP32-C5 `PARL_IO`, ESP32/S3
//!   `LCD_CAM`, ESP32 `I2S` parallel).
//! - The ISR applies the pending delta while the DMA is stopped, relinks the
//!   ring (`bcm::circular::disarm_boundary`), and restarts the transfer from
//!   the head. The swap is therefore exact, race-free, and identical in
//!   behaviour on all backends.
//!
//! This module is compiled only when the `circular-dma` feature is enabled.
//! Its linear counterpart lives in [`super::linear`].
use core::sync::atomic::Ordering;

use esp_hal::Blocking;
use esp_hal::handler;
#[cfg(feature = "iram")]
use esp_hal::ram;

use super::{Shared, SWAP_DONE, signal_swap_done};
use crate::Hub75Error;
use crate::bcm::circular::CircularBcmBuf;
use crate::framebuffer::FrameBuffer;

// ---------------------------------------------------------------------------
// Platform-specific type aliases
// ---------------------------------------------------------------------------

cfg_select! {
    hub75_use_i2s_parallel => {
        pub(crate) type TxTransfer =
            esp_hal::i2s::parallel::I2sParallelTransfer<'static, CircularBcmBuf, Blocking>;
        /// Driver handle returned by [`consume_transfer`], for the restart.
        pub(crate) type TxDriver = esp_hal::i2s::parallel::I2sParallel<'static, Blocking>;
    }
    hub75_use_parl_io => {
        pub(crate) type TxTransfer =
            esp_hal::parl_io::ParlIoTxTransfer<'static, CircularBcmBuf, Blocking>;
        /// Driver handle returned by [`consume_transfer`], for the restart.
        pub(crate) type TxDriver = esp_hal::parl_io::ParlIoTx<'static, Blocking>;
    }
    hub75_use_lcd_cam => {
        pub(crate) type TxTransfer =
            esp_hal::lcd_cam::lcd::i8080::I8080Transfer<'static, CircularBcmBuf, Blocking>;
        /// Driver handle returned by [`consume_transfer`], for the restart.
        pub(crate) type TxDriver = esp_hal::lcd_cam::lcd::i8080::I8080<'static, Blocking>;
    }
    _ => {
        compile_error!("no HUB75 backend selected: enable exactly one chip feature");
    }
}

// ---------------------------------------------------------------------------
// Frame-boundary interrupt handling
// ---------------------------------------------------------------------------
//
// The circular descriptor ring carries no `suc_eof` in steady state (a
// `suc_eof` descriptor would halt the DMA channel on ESP32-C5 and signal
// spuriously on ESP32/S3). The boundary detector is *armed* by relinking the
// second-to-last ring descriptor to the spare boundary descriptor
// (`bcm::circular::arm_boundary`, which carries `suc_eof` and a `NULL` next)
// and enabling the backend's frame-boundary interrupt, and *disarmed* again
// by the ISR (`bcm::circular::disarm_boundary`). This is done only around a
// swap; in steady state no interrupts are enabled on any backend.

/// Per-backend frame-boundary interrupt management, through the transfer
/// kept alive in [`State`], so no `DMA::steal()` or stored closure is
/// needed on any backend.
pub(crate) trait FrameInterrupt {
    /// Clear a pending frame-boundary flag. Called when arming (to drain
    /// stale flags) and from the frame-count ISR.
    fn clear_frame_interrupt(&self);

    /// Enable the frame-boundary source.
    fn listen_frame_interrupt(&self);

    /// Disable the frame-boundary source.
    fn unlisten_frame_interrupt(&self);
}

cfg_select! {
    hub75_use_i2s_parallel => {
        impl FrameInterrupt for TxTransfer {
            fn clear_frame_interrupt(&self) {
                use esp_hal::i2s::parallel::I2sParallelInterrupt;
                self.clear_interrupts(I2sParallelInterrupt::Eof);
            }

            fn listen_frame_interrupt(&self) {
                use esp_hal::i2s::parallel::I2sParallelInterrupt;
                self.listen(I2sParallelInterrupt::Eof);
            }

            fn unlisten_frame_interrupt(&self) {
                use esp_hal::i2s::parallel::I2sParallelInterrupt;
                self.unlisten(I2sParallelInterrupt::Eof);
            }
        }
    }
    hub75_use_lcd_cam => {
        impl FrameInterrupt for TxTransfer {
            fn clear_frame_interrupt(&self) {
                use esp_hal::lcd_cam::lcd::i8080::I8080Interrupt;
                self.clear_interrupts(I8080Interrupt::TransDone);
            }

            fn listen_frame_interrupt(&self) {
                use esp_hal::lcd_cam::lcd::i8080::I8080Interrupt;
                self.listen(I8080Interrupt::TransDone);
            }

            fn unlisten_frame_interrupt(&self) {
                use esp_hal::lcd_cam::lcd::i8080::I8080Interrupt;
                self.unlisten(I8080Interrupt::TransDone);
            }
        }
    }
    // PARL_IO (ESP32-C5): the boundary source is the peripheral's `TxEof`
    // interrupt, which fires when the GDMA signals `suc_eof`. A consumed
    // `suc_eof` also *halts* the DMA channel, so the PARL_IO ISR must restart
    // the transfer after every boundary.
    hub75_use_parl_io => {
        impl FrameInterrupt for TxTransfer {
            fn clear_frame_interrupt(&self) {
                use esp_hal::parl_io::ParlIoTxInterrupt;
                self.clear_interrupts(ParlIoTxInterrupt::Eof);
            }

            fn listen_frame_interrupt(&self) {
                use esp_hal::parl_io::ParlIoTxInterrupt;
                self.listen(ParlIoTxInterrupt::Eof);
            }

            fn unlisten_frame_interrupt(&self) {
                use esp_hal::parl_io::ParlIoTxInterrupt;
                self.unlisten(ParlIoTxInterrupt::Eof);
            }
        }
    }
    _ => {}
}

// ---------------------------------------------------------------------------
// ISR shared state
// ---------------------------------------------------------------------------

/// Circular-mode ISR state. Mirror of [`super::linear::State`]: both carry
/// `current_fb_ptr` + `pending_delta` for the pointer-delta swap protocol;
/// the other fields are mode-specific.
pub(crate) struct State {
    // Kept alive to prevent the DMA from stopping. All backends also use
    // the stored transfer to manage the frame-boundary interrupt (see
    // `FrameInterrupt`).
    pub(crate) transfer: Option<TxTransfer>,
    pub(crate) descriptors: *mut esp_hal::dma::DmaDescriptor,
    pub(crate) descriptor_count: usize,
    pub(crate) current_fb_ptr: *const (),
    /// Pointer delta for a pending swap, applied by the ISR at the next pass
    /// boundary (see `Hub75::swap`).
    pub(crate) pending_delta: Option<isize>,
    /// Set by `swap()` and cleared by the frame-boundary ISR; prevents
    /// a second `swap()` while one is still in-flight.
    pub(crate) swap_in_flight: bool,
    /// Word width of the transfer. Only used by `LCD_CAM`, to reconstruct
    /// the `I8080::send()` call when the boundary ISR restarts the chain;
    /// other backends pass a placeholder matching their transfer width.
    pub(crate) word_size: crate::framebuffer::WordSize,
}

// SAFETY (`Send`): same justification as `linear::State` — required so the
// `Mutex<RefCell<Option<_>>>` static is `Sync`; all access is serialised by
// the embassy mutex.
unsafe impl Send for State {}

type SharedState = Shared<Option<State>>;

static STATE: SharedState = Shared::new(None);

// ---------------------------------------------------------------------------
// Pass-boundary handling
// ---------------------------------------------------------------------------

/// Apply a pending framebuffer pointer delta to every descriptor, at a pass
/// boundary. Shared by the per-backend ISRs.
#[cfg_attr(feature = "iram", ram)]
fn apply_pending_delta(state: &mut State, delta: isize) {
    // SAFETY: `descriptors` points to a `&'static mut` descriptor
    // array that outlives everything. The DMA engine may be reading
    // descriptor fields while we rewrite the `buffer` pointers here;
    // that race is fine:
    //  1. Each `buffer` field is a naturally aligned 32-bit pointer; aligned 32-bit
    //     stores are atomic with respect to the DMA bus master, so DMA never sees a
    //     half-written pointer.
    //  2. The delta is applied while the DMA is stopped: the armed chain ends
    //     on the spare boundary descriptor (`suc_eof` + `NULL` next), so the
    //     engine has halted at the pass boundary before this runs, and the
    //     transfer is only restarted after this rewrite, from the head of the
    //     ring. The rewritten pointers therefore take effect deterministically
    //     before the engine fetches anything.
    //  3. The delta stays valid because all plane data lives in one contiguous `FB`
    //     allocation and old and new framebuffers have identical layout.
    unsafe {
        for i in 0..state.descriptor_count {
            let desc = &mut *state.descriptors.add(i);
            desc.buffer = desc.buffer.wrapping_byte_offset(delta);
        }
    }
}

/// Consume the ended transfer in the boundary ISR and return the driver
/// handle and buffer for the restart.
///
/// Panics if the transfer is not complete: the armed chain ends on the spare
/// boundary descriptor, so the DMA must have halted before this ISR runs.
/// Busy-waiting in an interrupt handler is not acceptable.
///
/// Per-backend completion handling (this is where the flag-timing rules
/// live):
/// - **I2S / `LCD_CAM`**: `wait()` polls peripheral *state* registers
///   (`tx_idle` / `lcd_start`), not interrupt flags, so the `Eof` flag is
///   cleared first to drop stale latches.
/// - **`PARL_IO`**: `wait()` polls `INT_RAW.tx_eof` — the very flag that fired
///   this interrupt (`suc_eof` on the boundary descriptor drives the
///   peripheral's `tx_eof` via `TxEofSource::DmaEof`). It must **not** be
///   cleared beforehand: `clear_frame_interrupt` writes `INT_CLR`, which
///   clears `INT_RAW` (write-to-clear) and would leave `wait()` spinning on
///   a flag that no longer exists. `wait()` itself clears it on completion.
#[cfg_attr(feature = "iram", ram)]
fn consume_transfer(xfer: TxTransfer) -> (TxDriver, CircularBcmBuf) {
    cfg_select! {
        hub75_use_parl_io => {}
        _ => {
            // Safe here: `wait()` polls peripheral state, not this flag.
            xfer.clear_frame_interrupt();
        }
    }

    assert!(
        xfer.is_done(),
        "circular boundary ISR: transfer not complete at the pass boundary"
    );

    cfg_select! {
        hub75_use_i2s_parallel => {
            let (driver, buf) = xfer.wait();
            (driver, buf)
        }
        _ => {
            let (_, driver, buf) = xfer.wait();
            (driver, buf)
        }
    }
}

/// Restart the transfer from the head of the ring (which now sources the new
/// framebuffer) and return the new transfer, or `None` on failure (the
/// driver handle and buffer are dropped; the display stops).
///
/// `word_size` is only meaningful on `LCD_CAM`; other backends ignore it.
#[cfg_attr(feature = "iram", ram)]
fn restart_transfer(
    driver: TxDriver,
    buf: CircularBcmBuf,
    word_size: crate::framebuffer::WordSize,
) -> Option<TxTransfer> {
    cfg_select! {
        hub75_use_i2s_parallel => {
            let _ = word_size;
            driver.send(buf).ok()
        }
        hub75_use_lcd_cam => {
            use esp_hal::lcd_cam::lcd::i8080::Command;
            match word_size {
                crate::framebuffer::WordSize::Eight => {
                    driver.send(Command::<u8>::None, 0, buf).ok()
                }
                crate::framebuffer::WordSize::Sixteen => {
                    driver.send(Command::<u16>::None, 0, buf).ok()
                }
            }
        }
        hub75_use_parl_io => {
            let _ = word_size;
            // Dummy transfer length (`tx_bytelen = 0`): with
            // `TxEofSource::DmaEof` the frame ends at the boundary
            // descriptor's `suc_eof` regardless of the bit-length counter,
            // so there is no size limit.
            driver.write(PARL_IO_DUMMY_TRANSFER_LEN, buf).ok()
        }
        _ => {
            unreachable!("no HUB75 backend selected")
        }
    }
}

/// Swap-boundary ISR — single handler for all backends.
///
/// The detector is armed only around a swap (see [`super::Hub75::swap`]);
/// a stale interrupt is ignored. When armed, the chain has ended on the
/// spare boundary descriptor: the handler consumes the ended transfer,
/// applies the pending pointer delta while the engine is stopped,
/// relinks the ring, and restarts the transfer from the head — the
/// swap is exact, with no pass boundary ever sourced from the old
/// buffer after the delta is applied.
#[handler]
#[cfg_attr(feature = "iram", ram)]
pub(crate) fn isr() {
    STATE.with(|state| {
        let Some(state) = state.as_mut() else {
            return;
        };

        // Gate: ignore the boundary while no swap is armed (stale
        // interrupt). The display must never be touched otherwise.
        if !state.swap_in_flight {
            return;
        }

        // Take the transfer; the chain has ended on the boundary
        // descriptor, so the DMA is already stopped.
        let Some(xfer) = state.transfer.take() else {
            return;
        };
        xfer.unlisten_frame_interrupt();

        // Consume the ended transfer. The DMA stays stopped; the
        // restart only happens after the pending delta has been
        // applied below.
        let (driver, buf) = consume_transfer(xfer);

        // Apply the delta while the DMA is stopped: from the
        // restart below, every descriptor sources the new
        // framebuffer.
        if let Some(delta) = state.pending_delta.take() {
            apply_pending_delta(state, delta);
        }

        // Swap-armed boundary handled: disarm until the next swap
        // by relinking the ring (`next` back to the head).
        crate::bcm::circular::disarm_boundary(state.descriptors, state.descriptor_count);

        // Restart the transfer from the head of the ring, now
        // sourcing the new framebuffer. On failure the driver handle
        // and buffer are dropped; the display stops.
        state.transfer = restart_transfer(driver, buf, state.word_size);

        // The DMA is provably no longer reading the old framebuffer.
        state.swap_in_flight = false;
        signal_swap_done();
    });
}

/// On ESP32-C5, the GDMA EOF signal is generated by the DMA channel rather
/// than the `PARL_IO` byte counter, so the transfer-length field is unused.
#[cfg(all(hub75_use_parl_io, esp32c5))]
pub(crate) const PARL_IO_DUMMY_TRANSFER_LEN: usize = 0;

// ---------------------------------------------------------------------------
// Circular-DMA state storage (called by platform constructors after starting
// DMA)
// ---------------------------------------------------------------------------

/// Store the ISR state after the platform constructor started the DMA.
///
/// `word_size` is the transfer's word width — only `LCD_CAM` uses it (to
/// reconstruct the `I8080::send()` call when the boundary ISR restarts the
/// chain); other backends pass a placeholder.
pub(crate) fn init_state(
    xfer: TxTransfer,
    descriptor_ptr: *mut esp_hal::dma::DmaDescriptor,
    descriptor_count: usize,
    fb_ptr: *const (),
    word_size: crate::framebuffer::WordSize,
) {
    STATE.with(|state| {
        *state = Some(State {
            transfer: Some(xfer),
            descriptors: descriptor_ptr,
            descriptor_count,
            current_fb_ptr: fb_ptr,
            pending_delta: None,
            swap_in_flight: false,
            word_size,
        });
    });
    drain_stale_frame_interrupt();
}

/// The transfer is stored; from this point on every interrupt can be
/// serviced through it. Drain any stale frame-boundary flag. The detector is
/// armed by `swap()` and disarmed by the ISR; no interrupts are enabled in
/// steady state.
fn drain_stale_frame_interrupt() {
    STATE.with(|state| {
        let state = state.as_mut().expect("just stored");
        let transfer = state.transfer.as_ref().expect("transfer kept alive");
        transfer.clear_frame_interrupt();
    });
}

// ---------------------------------------------------------------------------
// Swap
// ---------------------------------------------------------------------------

impl<DM: esp_hal::DriverMode, FB: FrameBuffer + 'static> super::Hub75<DM, FB> {
    /// Initiate a framebuffer swap (circular-DMA mode).
    ///
    /// Updates all DMA descriptor buffer pointers immediately and returns a
    /// [`Hub75Swap`](crate::Hub75Swap) transfer object. The DMA engine's internal register may
    /// still be pointing into the old buffer for the currently in-flight
    /// descriptor. Call [`.wait_for_done()`](crate::Hub75Swap::wait_for_done) then
    /// [`.wait()`](crate::Hub75Swap::wait), or just `.wait()` directly for blocking.
    ///
    /// # Swap granularity
    ///
    /// The switch is exact on all backends. Arming relinks the second-to-last
    /// ring descriptor to the spare boundary descriptor (`suc_eof` + `NULL`
    /// next) with a single atomic write, so the chain ends at the next pass
    /// boundary; the ISR applies the pointer delta while the DMA is stopped,
    /// relinks the ring, and restarts the transfer from the head sourcing the
    /// new framebuffer. The swap takes up to one pass period to complete, and
    /// the output is blanked for the (very brief) peripheral-drain window at
    /// the boundary.
    ///
    /// # Errors
    ///
    /// Returns [`Hub75Error::SwapInFlight`] along with ownership of `new_fb`
    /// if a previous [`Hub75Swap`](crate::Hub75Swap) is still outstanding. Only one swap may be
    /// in-flight at a time; call `.wait()` (or `.wait_for_done().await` then
    /// `.wait()`) on the previous [`Hub75Swap`](crate::Hub75Swap) before calling `swap()` again.
    ///
    /// # Panics
    ///
    /// Panics if the driver has not been initialised (no `Hub75` instance
    /// was created).
    pub fn swap(
        &self,
        new_fb: &'static mut FB,
    ) -> Result<super::Hub75Swap<FB>, (Hub75Error, &'static mut FB)> {
        let new_fb_ptr = core::ptr::from_mut::<FB>(new_fb);
        let old_fb_ptr = STATE.with(|state| {
            let state = state.as_mut().expect("Hub75 not initialised");
            if state.swap_in_flight {
                return Err(new_fb_ptr as *const ());
            }
            let delta = new_fb_ptr as isize - state.current_fb_ptr as isize;
            // The delta is applied by the frame-boundary ISR at the pass
            // boundary, while the DMA is stopped (see `isr`).
            state.pending_delta = Some(delta);
            state.swap_in_flight = true;
            // Arm the boundary detector and enable the backend's
            // frame-boundary interrupt.
            //
            // `arm_boundary` copies the last ring descriptor's buffer/length
            // into the spare boundary descriptor (`suc_eof` + `NULL` next)
            // and relinks the second-to-last ring descriptor to it with a
            // single atomic write — the chain ends at the next pass boundary
            // on every backend, and the ISR restores the ring and restarts
            // the transfer.
            crate::bcm::circular::arm_boundary(state.descriptors, state.descriptor_count);
            if let Some(xfer) = state.transfer.as_ref() {
                // Drain any frame-boundary flag latched before the update,
                // so the ISR cannot attribute a pre-swap boundary to the new
                // buffer and release the old framebuffer while the DMA may
                // still be reading from it.
                xfer.clear_frame_interrupt();
                xfer.listen_frame_interrupt();
            }
            let old_ptr = state.current_fb_ptr;
            state.current_fb_ptr = new_fb_ptr as *const ();
            SWAP_DONE.store(false, Ordering::Release);
            Ok(old_ptr)
        });

        match old_fb_ptr {
            Ok(old) => Ok(super::Hub75Swap {
                old_fb_ptr: old as *mut FB,
            }),
            Err(_) => Err((Hub75Error::SwapInFlight, new_fb)),
        }
    }
}

// ---------------------------------------------------------------------------
// Hub75Swap completion (circular)
// ---------------------------------------------------------------------------

impl<FB: FrameBuffer + 'static> super::Hub75Swap<FB> {
    /// Spin-loops until the DMA is guaranteed to no longer be reading from
    /// the old framebuffer, then returns it for reuse.
    ///
    /// If [`wait_for_done()`](Self::wait_for_done) was already awaited, this
    /// returns immediately.
    ///
    /// # Errors
    ///
    /// The error variant is never returned in circular-DMA mode; it exists
    /// for API consistency with the non-circular [`wait()`](Self::wait).
    pub fn wait(self) -> Result<&'static mut FB, (Hub75Error, &'static mut FB)> {
        while !SWAP_DONE.load(Ordering::Acquire) {
            core::hint::spin_loop();
        }
        Ok(unsafe { &mut *self.old_fb_ptr })
    }

    /// Yields to the executor until the swap is complete.
    ///
    /// After this resolves, call [`wait()`](Self::wait) to obtain the old
    /// framebuffer.
    pub async fn wait_for_done(&mut self) {
        core::future::poll_fn(|cx| {
            if SWAP_DONE.load(Ordering::Acquire) {
                return core::task::Poll::Ready(());
            }
            super::SWAP_WAKER.with(|waker| {
                if SWAP_DONE.load(Ordering::Acquire) {
                    return core::task::Poll::Ready(());
                }
                *waker = Some(cx.waker().clone());
                core::task::Poll::Pending
            })
        })
        .await;
    }
}
