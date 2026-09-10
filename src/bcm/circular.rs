//! Circular DMA buffer for continuous HUB75 refresh.
//!
//! `BcmBuf` builds a single circular DMA descriptor chain encoding the
//! full BCM (Binary Code Modulation) repetition sequence. The DMA engine starts
//! once and loops forever. No descriptor in the ring carries `suc_eof`: on
//! ESP32/S3 that keeps the free-running loop free of `out_eof` events, and on
//! ESP32-C5 (`PARL_IO`) a `suc_eof` descriptor would *terminate* the transfer.
//!
//! A swap arms the pass-boundary detector with a spare **boundary descriptor**
//! ([`BOUNDARY_DESCRIPTOR`]): `swap()` copies the last ring descriptor's
//! `buffer`/`size`/`length` into it (invisible to the DMA — the spare is not
//! linked yet) and then relinks the second-to-last ring descriptor to the spare
//! with a single atomic write. The spare carries `suc_eof` and a `NULL` next,
//! so the chain ends at the next pass boundary exactly like a normal
//! end-of-transfer on every backend. The boundary ISR applies the pending
//! pointer delta to the ring descriptors while the engine is stopped, relinks
//! the ring (see [`disarm_boundary`]), and restarts the transfer.

use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::dma::Preparation;
#[cfg(feature = "iram")]
use esp_hal::ram;

use crate::framebuffer::FrameBuffer;

pub(crate) struct BcmBuf {
    descriptors: &'static mut [DmaDescriptor],
    descriptor_count: usize,
}

impl BcmBuf {
    /// Build a circular descriptor chain from the given framebuffer.
    ///
    /// The chain encodes the full BCM repetition sequence (identical layout to
    /// `full-chain-dma`) with two differences:
    /// - The last descriptor's `next` points back to `desc[0]` (circular).
    /// - No descriptor has `suc_eof` set (see the module docs).
    ///
    /// Segments are streamed straight from the framebuffer, so no
    /// `SegmentCache` is materialised (neither on the stack nor in BSS) —
    /// the cache is only needed by linear mode, where the ISR walks it at
    /// runtime. The descriptor count comes from
    /// [`dma_descriptor_count`](crate::dma_descriptor_count), which computes
    /// it at compile time from `FB::BCM_SEGMENT_SHAPES`.
    pub(crate) fn new<FB: FrameBuffer>(
        descriptors: &'static mut [DmaDescriptor],
        fb: &'static FB,
    ) -> Self {
        let total_descs = crate::dma_descriptor_count::<FB>(crate::MAX_DMA_CHUNK_SIZE);
        debug_assert!(
            descriptors.len() >= total_descs,
            "not enough DMA descriptors: have {}, need {}",
            descriptors.len(),
            total_descs,
        );

        let ring_start = descriptors.as_mut_ptr();
        super::fill_full_chain(
            &mut descriptors[..total_descs],
            fb.bcm_segment_count(),
            |i| {
                let seg = fb.bcm_segment(i);
                debug_assert!(!seg.ptr.is_null(), "segment {i} returned a null pointer");
                seg
            },
            total_descs,
            ring_start,
            // Free-running ring: no descriptor carries `suc_eof` (a consumed
            // `suc_eof` would halt the DMA on ESP32-C5 and signal spuriously
            // on ESP32/S3). A swap arms the boundary detector by relinking the
            // second-to-last ring descriptor to the spare boundary descriptor
            // (see `arm_boundary`).
            false,
        );

        Self {
            descriptors,
            descriptor_count: total_descs,
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
}

/// Spare DMA descriptor used as the pass-boundary marker (see the module
/// docs). Declared separately from the user-provided descriptor array so the
/// array size — and the public `dma_descriptor_count` API — stay unchanged;
/// circular mode owns these 12 bytes of DMA-capable internal SRAM.
///
/// # SAFETY (static mut)
///
/// Written by `arm_boundary` (task context) and read by the DMA only while it
/// is linked into the chain — i.e. between `arm_boundary` and
/// `disarm_boundary`, during which nobody writes it. All writes are
/// serialised against the ISRs by the ISR state lock (`STATE` in `isr`
/// `isr.rs`), exactly like the ring descriptors. Access is only through raw
/// pointers under that lock (`static mut` is never referenced).
pub(crate) static mut BOUNDARY_DESCRIPTOR: DmaDescriptor = DmaDescriptor::EMPTY;

/// Arm the pass-boundary detector.
///
/// Copies the last ring descriptor's `buffer` and flags into the spare
/// [`BOUNDARY_DESCRIPTOR`] (setting `suc_eof`, `next = NULL`), then relinks
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
/// circular chain and unlinking the spare [`BOUNDARY_DESCRIPTOR`].
///
/// Single atomic, naturally aligned 32-bit write; callable from task and
/// interrupt context.
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

// SAFETY: All access is serialised by the ISR state lock (`STATE` in `isr`
// `isr.rs`, an `esp_sync::RawMutex`).
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
