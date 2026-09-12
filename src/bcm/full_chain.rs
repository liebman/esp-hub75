//! Full-chain DMA buffer for HUB75 refresh.
//!
//! [`BcmBuf`] builds a single DMA descriptor chain encoding the full BCM
//! (Binary Code Modulation) repetition sequence, streaming segments straight
//! from the framebuffer. The chain is built once ([`build`](BcmBuf::build)) and
//! afterwards only the descriptor `buffer` pointers are rewritten on a
//! framebuffer swap, so no segment cache is ever materialised (neither on the
//! stack nor in BSS).
//!
//! Two variants share this code, selected at compile time:
//!
//! - **`full-chain-dma` (terminating)**: the last descriptor has `next = NULL`
//!   and carries `suc_eof`, so the DMA raises a frame-boundary interrupt after
//!   every full frame. The ISR applies the pending pointer delta at that
//!   natural boundary and restarts the transfer from the head.
//! - **`circular-dma` (free-running ring)**: the last descriptor wraps back to
//!   `desc[0]` and no descriptor carries `suc_eof` — on ESP32/S3 that keeps the
//!   free-running loop free of `out_eof` events, and on ESP32-C5 (`PARL_IO`) a
//!   `suc_eof` descriptor would *terminate* the transfer.
//!
//! A circular swap arms the pass-boundary detector with a spare **boundary
//! descriptor** (`BOUNDARY_DESCRIPTOR`): [`swap`] copies the last ring
//! descriptor's `buffer`/`size`/`length` into it (invisible to the DMA — the
//! spare is not linked yet) and then relinks the second-to-last ring descriptor
//! to the spare with a single atomic write. The spare carries `suc_eof` and a
//! `NULL` next, so the chain ends at the next pass boundary exactly like a
//! normal end-of-transfer on every backend. The boundary ISR applies the
//! pending pointer delta to the ring descriptors while the engine is stopped,
//! relinks the ring (see `disarm_boundary`), and restarts the transfer.
//!
//! [`swap`]: crate::Hub75::swap

use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::dma::Preparation;
#[cfg(feature = "iram")]
use esp_hal::ram;

use crate::framebuffer::FrameBuffer;

pub(crate) struct BcmBuf {
    descriptors: &'static mut [DmaDescriptor],
    descriptor_count: usize,
    /// Total bytes streamed by the chain. ESP32-C6 (`PARL_IO`) only: the
    /// peripheral's EOF bit-length counter needs the whole-frame length up
    /// front (see `Transfer::start`), and this buffer keeps no segments to sum.
    #[cfg(esp32c6)]
    total_len: usize,
}

impl BcmBuf {
    /// Create an empty buffer bound to the given descriptor storage.
    ///
    /// The descriptor chain is filled later by [`build`](Self::build) once the
    /// framebuffer is known.
    pub(crate) fn new(descriptors: &'static mut [DmaDescriptor]) -> Self {
        Self {
            descriptors,
            descriptor_count: 0,
            #[cfg(esp32c6)]
            total_len: 0,
        }
    }

    /// Build the full descriptor chain from the given framebuffer.
    ///
    /// The chain encodes the full BCM repetition sequence with the
    /// terminating / ring difference described in the module docs. Segments
    /// are streamed straight from the framebuffer, so no segment cache is
    /// materialised. The descriptor count comes from
    /// [`dma_descriptor_count`](crate::dma_descriptor_count), which computes it
    /// at compile time from the framebuffer's static BCM sequence.
    pub(crate) fn build<FB: FrameBuffer>(&mut self, fb: &'static FB) {
        let total_descs = crate::dma_descriptor_count::<FB>(crate::MAX_DMA_CHUNK_SIZE);
        debug_assert!(
            self.descriptors.len() >= total_descs,
            "not enough DMA descriptors: have {}, need {}",
            self.descriptors.len(),
            total_descs,
        );

        // Circular wraps the last descriptor back to the head and leaves every
        // descriptor without `suc_eof`; full-chain terminates (`suc_eof` +
        // `NULL` next). See the module docs.
        let (last_next, last_suc_eof) = cfg_select! {
            feature = "circular-dma" => (self.descriptors.as_mut_ptr(), false),
            _ => (core::ptr::null_mut(), true),
        };

        super::fill_descriptor_chain(
            &mut self.descriptors[..total_descs],
            fb.bcm_segment_count(),
            |i| {
                let seg = fb.bcm_segment(i);
                debug_assert!(!seg.ptr.is_null(), "segment {i} returned a null pointer");
                seg
            },
            total_descs,
            last_next,
            last_suc_eof,
        );

        self.descriptor_count = total_descs;

        // ESP32-C6 needs the whole-frame byte length for the PARL_IO EOF
        // counter, and this buffer keeps no segments to sum later.
        #[cfg(esp32c6)]
        {
            let mut total = 0;
            for i in 0..fb.bcm_segment_count() {
                let seg = fb.bcm_segment(i);
                total += seg.len * seg.reps;
            }
            self.total_len = total;
        }
    }

    /// Raw pointer to the descriptor array (for ISR access after `send()`
    /// consumes this buffer).
    pub(crate) fn descriptors_ptr(&mut self) -> *mut DmaDescriptor {
        self.descriptors.as_mut_ptr()
    }

    /// Number of active descriptors in the chain.
    pub(crate) fn descriptor_count(&self) -> usize {
        self.descriptor_count
    }

    /// Byte length of the DMA transfer `prepare()` will start.
    ///
    /// ESP32-C6 (`PARL_IO`) only — the whole-frame byte length captured in
    /// [`build`](Self::build). On the other backends the transfer length is a
    /// dummy or unused (see `Transfer::start`).
    #[cfg(esp32c6)]
    #[cfg_attr(feature = "iram", ram)]
    pub(crate) fn current_transfer_len(&self) -> usize {
        self.total_len
    }
}

// ---------------------------------------------------------------------------
// Pass-boundary detector (circular ring only)
// ---------------------------------------------------------------------------

/// Spare DMA descriptor used as the pass-boundary marker (`BOUNDARY_DESCRIPTOR`
/// in the module docs). Declared separately from the user-provided descriptor
/// array so the array size — and the public `dma_descriptor_count` API — stay
/// unchanged; circular mode owns these 12 bytes of DMA-capable internal SRAM.
///
/// # SAFETY (static mut)
///
/// Written by `arm_boundary` (task context) and read by the DMA only while it
/// is linked into the chain — i.e. between `arm_boundary` and
/// `disarm_boundary`, during which nobody writes it. All writes are
/// serialised against the ISRs by the ISR state lock (`STATE` in `isr`),
/// exactly like the ring descriptors. Access is only through raw pointers
/// under that lock (`static mut` is never referenced).
#[cfg(feature = "circular-dma")]
pub(crate) static mut BOUNDARY_DESCRIPTOR: DmaDescriptor = DmaDescriptor::EMPTY;

/// Arm the pass-boundary detector.
///
/// Copies the last ring descriptor's `buffer` and flags into the spare
/// boundary descriptor (setting `suc_eof`, `next = NULL`), then relinks
/// the second-to-last ring descriptor to the spare.
///
/// The field writes happen while the spare is unlinked and therefore
/// invisible to the DMA; the relink is the single DMA-visible step and is a
/// naturally aligned 32-bit store, i.e. atomic with respect to the DMA bus
/// master. The DMA can never observe a half-armed chain.
///
/// Callable from task and interrupt context.
///
/// # Arming race
///
/// If the DMA has already fetched the second-to-last descriptor's `next` for
/// the current pass, the ring wraps as usual and the chain ends one pass
/// later — the swap simply completes a pass later, with no other effect.
#[cfg(feature = "circular-dma")]
#[cfg_attr(feature = "iram", ram)]
pub(crate) fn arm_boundary(descriptors: *mut DmaDescriptor, descriptor_count: usize) {
    debug_assert!(
        descriptor_count >= 2,
        "the circular chain needs at least two descriptors"
    );

    // SAFETY: `descriptors` originates from a `&'static mut` descriptor array
    // stored in the ISR state, valid for the driver's lifetime. The spare is
    // accessed under the same lock; see the `BOUNDARY_DESCRIPTOR` SAFETY note.
    unsafe {
        let last = &*descriptors.add(descriptor_count - 1);
        // Raw-pointer access to the `static mut`; writes only happen while
        // the spare is unlinked, under the ISR state lock (see the
        // `BOUNDARY_DESCRIPTOR` SAFETY note above).
        let spare = &raw mut BOUNDARY_DESCRIPTOR;

        // Invisible to the DMA: the spare is not linked yet.
        (*spare).flags = last.flags;
        (*spare).flags.set_suc_eof(true);
        (*spare).buffer = last.buffer;
        (*spare).next = core::ptr::null_mut();

        // The single atomic, DMA-visible arming write.
        (*descriptors.add(descriptor_count - 2)).next = spare;
    }
}

/// Disarm the pass-boundary detector: relink the second-to-last ring
/// descriptor back to the head of the ring, restoring the free-running
/// circular chain and unlinking the spare boundary descriptor.
///
/// Single atomic, naturally aligned 32-bit write; callable from task and
/// interrupt context.
#[cfg(feature = "circular-dma")]
#[cfg_attr(feature = "iram", ram)]
pub(crate) fn disarm_boundary(descriptors: *mut DmaDescriptor, descriptor_count: usize) {
    debug_assert!(
        descriptor_count >= 2,
        "the circular chain needs at least two descriptors"
    );

    // SAFETY: same as `arm_boundary`. `descriptors` is the `&'static mut`
    // descriptor ring stored in the ISR state, valid for the driver's lifetime,
    // and all access is serialised by the ISR state lock.
    unsafe {
        // Restore the free-running ring by pointing the second-to-last
        // descriptor back at the REAL last descriptor (whose own `next` still
        // wraps to the head). Pointing straight at the head instead would drop
        // `descriptors[count-1]` from the loop permanently, losing its data on
        // every frame after the first swap.
        (*descriptors.add(descriptor_count - 2)).next = descriptors.add(descriptor_count - 1);
    }
}

/// Apply a pending framebuffer pointer delta to every descriptor.
///
/// Called from the shared refresh ISR at a frame (full-chain) or pass
/// (circular) boundary, while the DMA is not reading the affected pointers —
/// the transfer is restarted from the head only afterwards. All plane data
/// lives in one contiguous framebuffer allocation, and the two framebuffers
/// have identical layout, so a single delta shifts every descriptor's
/// `buffer` pointer.
#[cfg_attr(feature = "iram", ram)]
pub(crate) fn apply_delta(descriptors: *mut DmaDescriptor, descriptor_count: usize, delta: isize) {
    // SAFETY: `descriptors` points to a `&'static mut` descriptor
    // array that outlives everything. The DMA engine may be reading
    // descriptor fields while we rewrite the `buffer` pointers here;
    // that race is fine:
    //  1. Each `buffer` field is a naturally aligned 32-bit pointer; aligned 32-bit
    //     stores are atomic with respect to the DMA bus master, so DMA never sees a
    //     half-written pointer.
    //  2. The delta is applied while the DMA is stopped: a circular chain ends on
    //     the spare boundary descriptor (`suc_eof` + `NULL` next), and a full-chain
    //     transfer ends on its own `suc_eof` + `NULL` next, so the engine has
    //     halted before this runs and the transfer is only restarted after this
    //     rewrite, from the head. The rewritten pointers therefore take effect
    //     deterministically before the engine fetches anything.
    //  3. The delta stays valid because all plane data lives in one contiguous
    //     framebuffer allocation and old and new framebuffers have identical
    //     layout.
    unsafe {
        for i in 0..descriptor_count {
            let desc = &mut *descriptors.add(i);
            desc.buffer = desc.buffer.wrapping_byte_offset(delta);
        }
    }
}

// SAFETY: All access is serialised by the ISR state lock (`STATE` in `isr`,
// an `esp_sync::NonReentrantMutex`).
unsafe impl Send for BcmBuf {}

unsafe impl DmaTxBuffer for BcmBuf {
    type View = Self;
    type Final = Self;

    #[cfg_attr(feature = "iram", ram)]
    fn prepare(&mut self) -> Preparation {
        super::make_preparation(self.descriptors)
    }

    #[cfg_attr(feature = "iram", ram)]
    fn into_view(self) -> Self::View {
        self
    }

    #[cfg_attr(feature = "iram", ram)]
    fn from_view(view: Self::View) -> Self::Final {
        view
    }
}
