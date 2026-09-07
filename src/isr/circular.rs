//! Circular-DMA refresh state machine.
//!
//! A single circular descriptor chain encodes the full BCM repetition
//! sequence; the DMA engine starts once and loops forever with no interrupts
//! enabled in steady state. Buffer swaps arm a pass-boundary detector
//! (`suc_eof` on the last descriptor) and the boundary ISR applies a pointer
//! delta to the descriptors at the boundary, then disarms again.
//!
//! This module is compiled only when the `circular-dma` feature is enabled.
//! Its linear counterpart lives in [`super::linear`].
use core::sync::atomic::Ordering;

use esp_hal::Blocking;
#[cfg(hub75_use_lcd_cam)]
use esp_hal::dma::DmaTxInterrupt;
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
    }
    hub75_use_parl_io => {
        pub(crate) type TxTransfer =
            esp_hal::parl_io::ParlIoTxTransfer<'static, CircularBcmBuf, Blocking>;
    }
    hub75_use_lcd_cam => {
        pub(crate) type TxTransfer =
            esp_hal::lcd_cam::lcd::i8080::I8080Transfer<'static, CircularBcmBuf, Blocking>;
    }
    _ => {
        compile_error!("no HUB75 backend selected: enable exactly one chip feature");
    }
}

// ---------------------------------------------------------------------------
// Frame-boundary interrupt handling
// ---------------------------------------------------------------------------
//
// The circular descriptor chain carries no `suc_eof` in steady state (a
// `suc_eof` descriptor would halt the DMA channel on ESP32-C5). The boundary
// detector is *armed* by setting `suc_eof` on the last descriptor and
// enabling the backend's frame-boundary interrupt, and *disarmed* again by
// the ISR. This is done only around a swap; in steady state no interrupts
// are enabled on any backend.

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
                self.clear_interrupts_dma(DmaTxInterrupt::Eof);
            }

            fn listen_frame_interrupt(&self) {
                self.listen_dma(DmaTxInterrupt::Eof);
            }

            fn unlisten_frame_interrupt(&self) {
                self.unlisten_dma(DmaTxInterrupt::Eof);
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
    //  2. At a pass boundary the DMA has consumed the whole chain, so the pointers
    //     take effect from the start of the next pass. On chips where `suc_eof`
    //     does not halt the DMA (ESP32/S3), the engine has already fetched
    //     descriptor 0's `buffer` at the wrap, before this ISR runs — plus one
    //     extra chunk per descriptor-transfer-time (`chunk_bytes / bus_hz`, ~200us
    //     at 4KiB/10MHz) of ISR entry latency. Those head chunks of the post-swap
    //     pass are sourced from the old framebuffer. This is a worst-case one-pass
    //     visual artifact confined to the LSB-first head of the BCM sequence; it is
    //     not memory-unsafe. The rewrite loop itself is orders of magnitude faster
    //     than the DMA's per-descriptor advance, so iteration order cannot race the
    //     engine.
    //  3. The delta stays valid because all plane data lives in one contiguous `FB`
    //     allocation and old and new framebuffers have identical layout.
    unsafe {
        for i in 0..state.descriptor_count {
            let desc = &mut *state.descriptors.add(i);
            desc.buffer = desc.buffer.wrapping_byte_offset(delta);
        }
    }
}

// Swap-boundary ISR.
//
// The boundary detector is armed only around a swap (see `Hub75::swap`);
// this handler applies the pending buffer delta at the pass boundary and
// disarms the detector again, leaving the chain free-running with no
// interrupts enabled in steady state.
//
// Two backend variants:
// - **I2S / `LCD_CAM` (ESP32, ESP32-S3)**: the DMA does not halt on
//   `suc_eof`, so the chain keeps running; just clear + disarm.
// - **PARL_IO (ESP32-C5)**: a consumed `suc_eof` *halts* the DMA channel,
//   so the ISR must restart the transfer after applying the pending delta.
cfg_select! {
    any(hub75_use_lcd_cam, hub75_use_i2s_parallel) => {
        /// Swap-boundary ISR (I2S / `LCD_CAM`).
        ///
        /// The boundary detector is armed only around a swap
        /// (see [`super::Hub75::swap`]); this handler applies the pending
        /// buffer delta at the pass boundary and disarms the detector again,
        /// leaving the chain free-running with no interrupts enabled in
        /// steady state.
        #[handler]
        #[cfg_attr(feature = "iram", ram)]
        pub(crate) fn isr() {
            STATE.with(|state| {
                let Some(state) = state.as_mut() else {
                    return;
                };

                // Scope the transfer borrow so the pending delta can be
                // applied to the state below.
                {
                    let Some(xfer) = state.transfer.as_ref() else {
                        return;
                    };
                    xfer.clear_frame_interrupt();
                }

                if let Some(delta) = state.pending_delta.take() {
                    apply_pending_delta(state, delta);
                    state.swap_in_flight = false;
                    signal_swap_done();
                }

                // Swap-armed boundary handled: disarm until the next swap. The
                // DMA does not halt on `suc_eof` on these chips, so the chain
                // keeps running.
                crate::bcm::circular::set_last_suc_eof(state.descriptors, state.descriptor_count, false);
                if let Some(xfer) = state.transfer.as_ref() {
                    xfer.unlisten_frame_interrupt();
                }
            });
        }
    }
    hub75_use_parl_io => {
        /// Swap-boundary ISR (PARL_IO / ESP32-C5).
        ///
        /// A consumed `suc_eof` *halts* the DMA channel, so this handler must
        /// restart the transfer after applying the pending delta. The
        /// detector is armed only around a swap (see [`super::Hub75::swap`]);
        /// in steady state the chain runs `suc_eof`-free and no interrupts
        /// are enabled.
        #[handler]
        #[cfg_attr(feature = "iram", ram)]
        pub(crate) fn isr() {
            STATE.with(|state| {
                let Some(state) = state.as_mut() else {
                    return;
                };
                // Take the transfer: `wait()` consumes it and returns the driver
                // and buffer for the restart. `wait()` returns instantly — the
                // boundary interrupt has already fired — and clears the `tx_eof`
                // flag.
                let Some(xfer) = state.transfer.take() else {
                    return;
                };

                if let Some(delta) = state.pending_delta.take() {
                    apply_pending_delta(state, delta);
                    state.swap_in_flight = false;
                    signal_swap_done();
                }

                // Swap-armed boundary handled: disarm until the next swap; the
                // chain runs suc_eof-free again until the next swap arms it.
                crate::bcm::circular::set_last_suc_eof(state.descriptors, state.descriptor_count, false);
                xfer.unlisten_frame_interrupt();

                let (_, tx, buf) = xfer.wait();
                // Dummy transfer length (`tx_bytelen = 0`): with
                // `TxEofSource::DmaEof` the frame ends at the armed `suc_eof`
                // descriptor regardless of the bit-length counter, so there is
                // no size limit.
                match tx.write(PARL_IO_DUMMY_TRANSFER_LEN, buf) {
                    Ok(xfer) => state.transfer = Some(xfer),
                    // The driver handle and buffer are dropped; the display
                    // stops.
                    Err((_, _tx, _buf)) => {}
                }
            });
        }
    }
    _ => {}
}

/// On ESP32-C5, the GDMA EOF signal is generated by the DMA channel rather
/// than the `PARL_IO` byte counter, so the transfer-length field is unused.
#[cfg(all(hub75_use_parl_io, esp32c5))]
pub(crate) const PARL_IO_DUMMY_TRANSFER_LEN: usize = 0;

// ---------------------------------------------------------------------------
// Circular-DMA state storage (called by platform constructors after starting
// DMA)
// ---------------------------------------------------------------------------

pub(crate) fn init_state(
    xfer: TxTransfer,
    descriptor_ptr: *mut esp_hal::dma::DmaDescriptor,
    descriptor_count: usize,
    fb_ptr: *const (),
) {
    STATE.with(|state| {
        *state = Some(State {
            transfer: Some(xfer),
            descriptors: descriptor_ptr,
            descriptor_count,
            current_fb_ptr: fb_ptr,
            pending_delta: None,
            swap_in_flight: false,
        });

        // The transfer is stored; from this point on every interrupt can be
        // serviced through it. Drain any stale frame-boundary flag. The
        // detector is armed by `swap()` and disarmed by the ISR; no
        // interrupts are enabled in steady state.
        {
            let state = state.as_mut().expect("just stored");
            let transfer = state.transfer.as_ref().expect("transfer kept alive");
            transfer.clear_frame_interrupt();
        }
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
    /// On ESP32-C5 (`PARL_IO`) the DMA halts at the boundary and the switch
    /// is exact. On ESP32/ESP32-S3 the DMA wraps without halting, so the
    /// first DMA chunk (~4KiB, the LSB end of the BCM sequence) of the pass
    /// in which the swap completes may still come from the previous
    /// framebuffer. The affected slice carries at most a few percent of
    /// frame brightness and the display is transitioning to new content
    /// anyway, so this is not visible in practice.
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
            // The delta is applied by the frame-boundary ISR at the next pass
            // boundary. On ESP32/S3 the wrap-time fetch of descriptor 0 (plus
            // ISR latency) means the first chunk(s) of that pass still source
            // the old framebuffer — a benign, LSB-weighted, one-pass artifact
            // (see `apply_pending_delta`).
            state.pending_delta = Some(delta);
            state.swap_in_flight = true;
            // Arm the boundary detector: mark the last descriptor with
            // `suc_eof` and enable the backend's frame-boundary interrupt.
            //
            // On ESP32/S3 the DMA wraps without halting, so the bit is
            // purely a marker for the next pass boundary; the ISR clears it
            // and disarms. On ESP32-C5 the consumed `suc_eof` *halts* the
            // DMA channel; the ISR restarts the transfer (see
            // `isr`).
            crate::bcm::circular::set_last_suc_eof(state.descriptors, state.descriptor_count, true);
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
