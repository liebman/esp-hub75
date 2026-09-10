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

use super::State;
use super::TransferPhase;
use super::TxDriver;
use super::STATE;
use crate::Hub75Error;
use crate::bcm::linear::BcmBuf;
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
    buf: BcmBuf,
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
        super::start_internal(fb)
    }
}
