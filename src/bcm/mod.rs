//! BCM (Binary Code Modulation) DMA buffer infrastructure.
//!
//! Common types and helpers shared by the linear and circular buffer
//! implementations.

use core::ptr::null;

use esp_hal::dma::BurstConfig;
use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::dma::EmptyBuf;
#[cfg(any(feature = "full-chain-dma", feature = "circular-dma"))]
use esp_hal::dma::Owner;
use esp_hal::dma::Preparation;
use esp_hal::dma::TransferDirection;

#[cfg(any(feature = "full-chain-dma", feature = "circular-dma"))]
use crate::MAX_DMA_CHUNK_SIZE;
use crate::framebuffer::BcmSegment;
use crate::framebuffer::FrameBuffer;

#[cfg(feature = "circular-dma")]
pub(crate) mod circular;
#[cfg(not(feature = "circular-dma"))]
pub(crate) mod linear;

/// Maximum number of BCM segments that can be cached for ISR use.
///
/// Sized for worst case: 32 row-pairs × (8 planes + 1 inter-row gap + 1
/// end-of-row trailer) = 320 segments.
pub(crate) const MAX_SEGMENTS: usize = 320;

const EMPTY_SEGMENT: BcmSegment = BcmSegment {
    ptr: null(),
    len: 0,
    reps: 0,
};

/// Cached BCM segment data for ISR use.
///
/// Stores the full segment sequence extracted from a `FrameBuffer` so the
/// ISR can drive DMA without calling trait methods (the framebuffer type is
/// erased in the ISR statics).
#[derive(Clone)]
pub(crate) struct SegmentCache {
    pub segments: [BcmSegment; MAX_SEGMENTS],
    pub count: usize,
    /// Consecutive segments that form one DMA transfer group.
    /// The ISR builds a descriptor chain for a whole group and fires
    /// only at group boundaries.
    pub segments_per_group: usize,
}

impl SegmentCache {
    pub const fn new() -> Self {
        Self {
            segments: [EMPTY_SEGMENT; MAX_SEGMENTS],
            count: 0,
            segments_per_group: 1,
        }
    }

    /// Total DMA descriptors required by this segment sequence.
    #[cfg(feature = "full-chain-dma")]
    pub fn descriptor_count(&self) -> usize {
        let max_chunk = crate::MAX_DMA_CHUNK_SIZE;
        let mut total = 0;
        let mut i = 0;
        while i < self.count {
            let seg = &self.segments[i];
            let descs_per_rep = seg.len.div_ceil(max_chunk);
            total += descs_per_rep * seg.reps;
            i += 1;
        }
        total
    }

    /// Number of DMA transfer groups in this sequence.
    #[cfg(not(feature = "full-chain-dma"))]
    pub fn group_count(&self) -> usize {
        self.count / self.segments_per_group
    }

    /// Maximum number of DMA descriptors required by any single group.
    ///
    /// Used by the default group-based mode, which rebuilds the descriptor
    /// table for each group transfer and therefore never needs more than
    /// the largest group's worth of descriptors.
    #[cfg(not(feature = "full-chain-dma"))]
    pub fn max_group_descriptor_count(&self) -> usize {
        let mut max = 0;
        let mut group = 0;
        while group < self.group_count() {
            let count = self.group_descriptor_count(group);
            if count > max {
                max = count;
            }
            group += 1;
        }
        max
    }

    /// DMA descriptors required for a single group starting at `group_idx`.
    #[cfg(not(feature = "full-chain-dma"))]
    pub fn group_descriptor_count(&self, group_idx: usize) -> usize {
        let max_chunk = crate::MAX_DMA_CHUNK_SIZE;
        let start = group_idx * self.segments_per_group;
        let end = start + self.segments_per_group;
        let mut total = 0;
        let mut i = start;
        while i < end {
            let seg = &self.segments[i];
            let descs_per_rep = seg.len.div_ceil(max_chunk);
            total += descs_per_rep * seg.reps;
            i += 1;
        }
        total
    }

    /// Total bytes in a single group (all segments × their reps).
    #[cfg(all(esp32c6, not(feature = "full-chain-dma")))]
    pub fn group_byte_count(&self, group_idx: usize) -> usize {
        let start = group_idx * self.segments_per_group;
        let end = start + self.segments_per_group;
        let mut total = 0;
        let mut i = start;
        while i < end {
            total += self.segments[i].len * self.segments[i].reps;
            i += 1;
        }
        total
    }
}

/// Extract BCM segments from a framebuffer into a [`SegmentCache`].
pub(crate) fn segments_from_fb<FB: FrameBuffer>(fb: &FB) -> SegmentCache {
    // Compile-time check that the segment cache can hold the framebuffer's
    // full scan sequence (evaluated per monomorphization).
    const {
        assert!(
            FB::BCM_SEGMENT_COUNT <= MAX_SEGMENTS,
            "framebuffer BCM segment count exceeds MAX_SEGMENTS"
        );
    }
    let count = fb.bcm_segment_count();
    let spg = fb.bcm_segments_per_group();
    assert!(
        count <= MAX_SEGMENTS,
        "bcm_segment_count {count} exceeds MAX_SEGMENTS"
    );
    assert!(
        spg > 0 && count % spg == 0,
        "bcm_segment_count {count} not divisible by segments_per_group {spg}"
    );
    let mut cache = SegmentCache::new();
    cache.count = count;
    cache.segments_per_group = spg;
    for i in 0..count {
        let seg = fb.bcm_segment(i);
        debug_assert!(!seg.ptr.is_null(), "segment {i} returned a null pointer");
        cache.segments[i] = seg;
    }
    cache
}

/// Build a `Preparation` pointing to the first descriptor in a chain.
///
/// Shared by both linear and circular buffer implementations.
pub(super) fn make_preparation(descriptors: &mut [DmaDescriptor]) -> Preparation {
    // `EmptyBuf` provides a `Preparation` with safe defaults; we override
    // the fields relevant to our descriptor chain. If `Preparation` gains
    // new fields in a future esp-hal release, review them here.
    let mut empty = EmptyBuf;
    let mut prep: Preparation = empty.prepare();
    prep.start = descriptors.as_mut_ptr();
    prep.direction = TransferDirection::Out;
    prep.burst_transfer = BurstConfig::default();
    prep.check_owner = Some(false);
    prep.auto_write_back = false;
    prep
}

#[cfg(any(feature = "full-chain-dma", feature = "circular-dma"))]
/// Fill a full-chain BCM descriptor sequence from cached segments.
///
/// The caller provides the `next` pointer for the last descriptor in the
/// chain — `null_mut()` for linear mode (last `next` = null),
/// `ring_start` (points back to `desc[0]`) for circular mode.
/// The last descriptor always has `suc_eof = 1`.
pub(super) fn fill_full_chain(
    descriptors: &mut [DmaDescriptor],
    cache: &SegmentCache,
    total_descs: usize,
    last_next: *mut DmaDescriptor,
) {
    let base_ptr = descriptors.as_mut_ptr();
    let mut desc_idx = 0;

    for seg_idx in 0..cache.count {
        let seg = &cache.segments[seg_idx];

        for _ in 0..seg.reps {
            let mut remaining = seg.len;
            let mut offset = 0;
            while remaining > 0 {
                let chunk = remaining.min(MAX_DMA_CHUNK_SIZE);
                let is_last = desc_idx + 1 == total_descs;
                let next = if is_last {
                    last_next
                } else {
                    unsafe { base_ptr.add(desc_idx + 1) }
                };
                let desc = &mut descriptors[desc_idx];
                // SAFETY: `seg.ptr` originates from a live framebuffer
                // and `offset` stays within the segment's byte length.
                desc.buffer = unsafe { seg.ptr.add(offset) as *mut u8 };
                desc.set_size(chunk);
                desc.set_length(chunk);
                desc.set_owner(Owner::Dma);
                desc.set_suc_eof(is_last);
                desc.next = next;
                remaining -= chunk;
                offset += chunk;
                desc_idx += 1;
            }
        }
    }
}
