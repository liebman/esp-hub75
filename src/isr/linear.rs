//! Linear (interrupt-per-transfer) refresh state machine.
//!
//! One DMA transfer per BCM segment group; the ISR rebuilds the descriptor
//! chain and restarts the transfer at every completion. Buffer swaps apply a
//! pointer delta to the cached segments at a frame boundary.
//!
//! The mode-shared pieces — transfer types, `State`, `swap()`, `Hub75Swap`
//! completion, and the per-backend `start_transfer` / `finish_transfer`
//! helpers — live in [`super`] and are compiled once for both refresh modes.
//!
//! This module is compiled only when the `circular-dma` feature is **not**
//! enabled. Its circular counterpart lives in [`super::circular`].

use core::sync::atomic::Ordering;

use super::start_transfer;
use super::State;
use super::TransferPhase;
use super::TxDriver;
use super::{HAS_ERROR, STATE, SWAP_DONE, signal_swap_done};
#[cfg(all(hub75_use_parl_io, esp32c5))]
use super::PARL_IO_DUMMY_TRANSFER_LEN;
use crate::Hub75Error;
use crate::bcm::cache_ptr;
use crate::bcm::linear::LinearBcmBuf;
use crate::bcm::segments_from_fb_into;
use crate::framebuffer::FrameBuffer;
#[cfg(hub75_use_lcd_cam)]
use crate::framebuffer::WordSize;

// ---------------------------------------------------------------------------
// ISR state initialisation (called by platform constructors)
// ---------------------------------------------------------------------------

// The `word_size` parameter and field only exist on LCD_CAM (ESP32-S3);
// they are cfg'd at the parameter/field level so there is a single function.
#[allow(unused_variables)]
pub(crate) fn init_state(
    tx: TxDriver,
    buf: LinearBcmBuf,
    #[cfg(hub75_use_lcd_cam)] word_size: WordSize,
) {
    STATE.with(|state| {
        *state = Some(State {
            transfer: TransferPhase::Idle(tx, buf),
            #[cfg(hub75_use_lcd_cam)]
            word_size,
            current_fb_ptr: core::ptr::null(),
            pending_delta: None,
        });
    });
}

// ---------------------------------------------------------------------------
// Start / restart
// ---------------------------------------------------------------------------

pub(crate) fn start_internal(fb: &'static impl FrameBuffer) -> Result<(), Hub75Error> {
    crate::bcm::validate_fb_internal_ram(fb);

    STATE.with(|state| {
        let state = state.as_mut().expect("Hub75 not initialised");

        let (tx, mut buf) =
            match core::mem::replace(&mut state.transfer, TransferPhase::Transitioning) {
                TransferPhase::Idle(tx, buf) | TransferPhase::Error(_, tx, buf) => (tx, buf),
                other => {
                    state.transfer = other;
                    return Err(Hub75Error::AlreadyRunning);
                }
            };

        // Resolve any stale Hub75Swap that was waiting on a previous error
        // state before it was consumed. Without this, a swap left un-waited
        // when restart() is called would spin forever because HAS_ERROR and
        // SWAP_DONE get cleared below, and no new pending_delta exists to
        // drive a fresh completion signal.
        signal_swap_done();

        // Build the cache in-place (restart/re-init path).
        segments_from_fb_into(fb, unsafe { &mut *cache_ptr().cast_mut() });
        buf.reset_with_cache();
        state.current_fb_ptr = core::ptr::from_ref(fb).cast::<()>();
        state.pending_delta = None;

        // PARL_IO only: the peripheral's EOF bit-length counter. On the C5
        // the EOF comes from the DMA channel, so the field is a dummy.
        #[cfg(hub75_use_parl_io)]
        let transfer_len = cfg_select! {
            esp32c5 => PARL_IO_DUMMY_TRANSFER_LEN,
            _ => buf.current_transfer_len(),
        };

        let xfer_result = start_transfer(
            tx,
            buf,
            #[cfg(hub75_use_parl_io)]
            transfer_len,
            #[cfg(hub75_use_lcd_cam)]
            state.word_size,
        );

        match xfer_result {
            Ok(xfer) => {
                state.transfer = TransferPhase::InFlight(xfer);
                HAS_ERROR.store(false, Ordering::Release);
                SWAP_DONE.store(false, Ordering::Release);
                Ok(())
            }
            Err((hub_err, tx, buf)) => {
                state.transfer = TransferPhase::Error(hub_err, tx, buf);
                HAS_ERROR.store(true, Ordering::Release);
                Err(hub_err)
            }
        }
    })
}

// ---------------------------------------------------------------------------
// Hub75::restart (linear only — circular never stops, so has no restart)
// ---------------------------------------------------------------------------

impl<DM: esp_hal::DriverMode, FB: FrameBuffer + 'static> super::Hub75<DM, FB> {
    /// Restart display refresh after an error.
    ///
    /// Callable after [`Hub75::swap`](super::Hub75::swap) returned an error.
    /// Sets up the BCM segment data and kicks off the first DMA transfer with
    /// the same framebuffer type.
    ///
    /// This method is only available when `circular-dma` is **not** enabled.
    /// In circular-DMA mode the DMA engine never stops, so there is no
    /// restart path.
    ///
    /// # Errors
    ///
    /// Returns [`Hub75Error::AlreadyRunning`](crate::Hub75Error::AlreadyRunning)
    /// if called while a transfer is already in flight. Call
    /// [`Hub75Swap::wait`](crate::Hub75Swap::wait) on the outstanding swap
    /// first.
    pub fn restart(&self, fb: &'static FB) -> Result<(), Hub75Error> {
        start_internal(fb)
    }
}
