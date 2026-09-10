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

#[cfg(feature = "iram")]
use esp_hal::ram;

use super::State;
use super::TransferPhase;
use super::TxDriver;
use super::{STATE, TxXfer};
use esp_hal::dma::DmaTxBuffer;

use crate::bcm::circular::BcmBuf;
#[cfg(hub75_use_lcd_cam)]
use crate::framebuffer::WordSize;

// ---------------------------------------------------------------------------
// Frame-boundary interrupt plumbing (circular mode only)
// ---------------------------------------------------------------------------

/// Clears a pending frame-boundary flag on the active backend.
///
/// Called by the ISR (to drain the handled boundary flag and any stale flag).
/// The free-running ring carries no `suc_eof`; the frame-boundary interrupt
/// source is enabled once by the platform constructor (before the first
/// transfer starts) and left enabled for the driver's lifetime. A swap arms
/// the boundary detector by relinking the ring to the spare boundary
/// descriptor; the ISR drains the boundary flag after consuming it.
///
/// The flag registers live in the peripherals rather than the DMA transfers,
/// so each backend exposes the clear through its peripheral (register access;
/// on ESP32 the constructor records which `I2S` instance the driver owns).
#[cfg_attr(feature = "iram", ram)]
pub(crate) fn clear_frame_interrupt() {
    cfg_select! {
        hub75_use_i2s_parallel => {
            crate::hub75::clear_frame_interrupt();
        }
        hub75_use_lcd_cam => {
            crate::hub75::clear_frame_interrupt();
        }
        // PARL_IO (ESP32-C5): the boundary source is the peripheral's `TxEof`
        // interrupt, which fires when the GDMA signals `suc_eof`. A consumed
        // `suc_eof` also *halts* the DMA channel, so the PARL_IO ISR must
        // restart the transfer after every boundary.
        hub75_use_parl_io => {
            crate::hub75::clear_frame_interrupt();
        }
        _ => {}
    }
}

// ---------------------------------------------------------------------------
// Pass-boundary handling
// ---------------------------------------------------------------------------

/// Apply a pending framebuffer pointer delta to every descriptor, at a pass
/// boundary. Called from the shared refresh ISR ([`super::isr`]) in
/// circular mode.
#[cfg_attr(feature = "iram", ram)]
pub(crate) fn apply_pending_delta(state: &mut State, delta: isize) {
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

/// Arm the pass-boundary detector for a pending swap. Called from the shared
/// [`super::swap`].
///
/// The frame-boundary interrupt source was enabled once by the platform
/// constructor (before the first transfer started) and stays enabled for the
/// driver's lifetime: the disarmed ring carries no `suc_eof`, so in steady
/// state no boundary interrupts fire and the ISR's stale-interrupt gate is
/// never hit. Arming is a single atomic relink (`arm_boundary`); the next
/// pass boundary fires the ISR.
#[cfg_attr(feature = "iram", ram)]
pub(crate) fn arm_swap_detector(state: &mut State) {
    crate::bcm::circular::arm_boundary(state.descriptors, state.descriptor_count);
}

/// Drain a stale frame-boundary flag and assert the transfer completed.
///
/// Called from the shared [`super::finish_transfer`] in circular mode. The
/// armed chain ends on the spare boundary descriptor, so the DMA must have
/// halted before the ISR runs; busy-waiting in an interrupt handler is not
/// acceptable.
///
/// Note: the boundary *flag* is drained by the ISR (or by `PARL_IO`'s
/// `wait()` itself), not here — see the flag-handling notes in
/// [`super::isr`].
#[cfg_attr(feature = "iram", ram)]
pub(crate) fn assert_transfer_done<B: DmaTxBuffer>(xfer: &TxXfer<B>) {
    assert!(
        xfer.is_done(),
        "circular boundary ISR: transfer not complete at the pass boundary"
    );
}

// ---------------------------------------------------------------------------
// Circular-DMA state storage (called by platform constructors before starting
// DMA)
// ---------------------------------------------------------------------------

/// Store the ISR state before the platform constructor starts the DMA.
///
/// Both refresh modes share the same two-step boot: [`init_state`] stores an
/// `Idle` `(tx, buf)` pair and `super::start_internal` binds the buffer to the
/// framebuffer and kicks off the first transfer. `word_size` is the
/// transfer's word width — only `LCD_CAM` uses it (to reconstruct the
/// `I8080::send()` call when the boundary ISR restarts the chain).
#[allow(unused_variables)]
pub(crate) fn init_state(
    tx: TxDriver,
    buf: BcmBuf,
    #[cfg(hub75_use_lcd_cam)] word_size: WordSize,
) {
    STATE.with(|state| {
        *state = Some(State {
            transfer: TransferPhase::Idle(tx, buf),
            descriptors: core::ptr::null_mut(),
            descriptor_count: 0,
            #[cfg(hub75_use_lcd_cam)]
            word_size,
            current_fb_ptr: core::ptr::null(),
            pending_delta: None,
        });
    });
}

