//! Circular DMA buffer for continuous HUB75 refresh.
//!
//! `CircularBcmBuf` builds a single circular DMA descriptor chain encoding the
//! full BCM (Binary Code Modulation) repetition sequence. The DMA engine starts
//! once and loops forever. No descriptor carries `suc_eof`: on ESP32/S3 that
//! keeps the free-running loop free of `out_eof` events, and on ESP32-C5
//! (`PARL_IO`) a `suc_eof` descriptor would *terminate* the transfer. A swap
//! arms the boundary detector by setting `suc_eof` on the last descriptor
//! (see [`set_last_suc_eof`]); the backend's ISR applies the pending pointer
//! delta at that pass boundary and disarms again.

use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::dma::Preparation;
#[cfg(feature = "iram")]
use esp_hal::ram;

use crate::framebuffer::FrameBuffer;

pub(crate) struct CircularBcmBuf {
    descriptors: &'static mut [DmaDescriptor],
    desc_count: usize,
}

impl CircularBcmBuf {
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
        );
        // `fill_full_chain` marks the last descriptor with `suc_eof = 1` (the
        // linear full-chain mode relies on it); clear it for the free-running
        // ring.
        descriptors[total_descs - 1].set_suc_eof(false);

        Self {
            descriptors,
            desc_count: total_descs,
        }
    }

    /// Raw pointer to the descriptor array (for ISR access after `send()`
    /// consumes this buffer).
    pub(crate) fn descriptors_ptr(&mut self) -> *mut DmaDescriptor {
        self.descriptors.as_mut_ptr()
    }

    /// Number of active descriptors in the chain.
    pub(crate) fn desc_count(&self) -> usize {
        self.desc_count
    }
}

/// Set or clear `suc_eof` on the last descriptor of the circular chain.
///
/// Arming the bit makes the DMA raise its end-of-frame event at the next pass
/// boundary. On ESP32/S3 this is purely a marker (the DMA wraps without
/// halting); on ESP32-C5 (`PARL_IO`) a consumed `suc_eof` *halts* the channel,
/// so the backend ISR must restart the transfer after a boundary.
///
/// Callable from task and interrupt context; the 32-bit flag write is atomic
/// with respect to the DMA bus master.
#[cfg_attr(feature = "iram", ram)]
pub(crate) fn set_last_suc_eof(descriptors: *mut DmaDescriptor, desc_count: usize, enabled: bool) {
    // SAFETY: `descriptors` originates from a `&'static mut` descriptor array
    // stored in the ISR state, valid for the driver's lifetime.
    unsafe {
        (*descriptors.add(desc_count - 1)).set_suc_eof(enabled);
    }
}

// SAFETY: All access is serialised by the ISR state lock (`STATE_LOCK` in
// `isr.rs`, an `esp_sync::RawMutex`).
unsafe impl Send for CircularBcmBuf {}

unsafe impl DmaTxBuffer for CircularBcmBuf {
    type View = Self;
    type Final = Self;

    #[cfg_attr(feature = "iram", ram)]
    fn prepare(&mut self) -> Preparation {
        super::make_preparation(self.descriptors)
    }

    fn into_view(self) -> Self::View {
        self
    }

    fn from_view(view: Self::View) -> Self::Final {
        view
    }
}
