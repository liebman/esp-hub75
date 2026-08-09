//! BCM (Binary Code Modulation) DMA buffer infrastructure.
//!
//! Common types and helpers shared by the linear and circular buffer
//! implementations.

#[cfg(not(feature = "circular-dma"))]
use core::cell::UnsafeCell;
use core::ptr::null;

use esp_hal::dma::BurstConfig;
use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::dma::EmptyBuf;
#[cfg(any(feature = "full-chain-dma", feature = "circular-dma"))]
use esp_hal::dma::Owner;
use esp_hal::dma::Preparation;
use esp_hal::dma::TransferDirection;
#[cfg(feature = "iram")]
use esp_hal::ram;

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

/// Single segment cache for ISR use (linear mode only).
///
/// One static slot holds the full segment sequence. `swap()` stores a
/// pointer delta (old FB → new FB) and the ISR applies it at the frame
/// boundary — same approach as circular-DMA mode. No copies, no slots.
#[cfg(not(feature = "circular-dma"))]
struct CacheCell(UnsafeCell<SegmentCache>);

#[cfg(not(feature = "circular-dma"))]
// SAFETY: All access is serialised by `critical_section::with`. The cache
// is written only by `start_internal()` (init/restart path) and the ISR
// (delta application at frame boundaries); `swap()` never touches it.
unsafe impl Sync for CacheCell {}

#[cfg(not(feature = "circular-dma"))]
static SEGMENT_CACHE: CacheCell = CacheCell(UnsafeCell::new(SegmentCache::new()));

/// Return a raw pointer to the static segment cache.
///
/// Used by [`BcmBuf`][super::linear::BcmBuf] and `start_internal()`.
#[cfg(not(feature = "circular-dma"))]
pub(crate) fn cache_ptr() -> *const SegmentCache {
    SEGMENT_CACHE.0.get()
}

/// Cached BCM segment data for ISR use.
///
/// Stores the full segment sequence extracted from a `FrameBuffer` so the
/// ISR can drive DMA without calling trait methods (the framebuffer type is
/// erased in the ISR statics).
pub(crate) struct SegmentCache {
    pub(crate) segments: [BcmSegment; MAX_SEGMENTS],
    pub(crate) count: usize,
    /// Consecutive segments that form one DMA transfer group.
    /// The ISR builds a descriptor chain for a whole group and fires
    /// only at group boundaries.
    pub(crate) segments_per_group: usize,
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
    #[cfg_attr(feature = "iram", ram)]
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
    #[cfg_attr(feature = "iram", ram)]
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
    #[cfg_attr(feature = "iram", ram)]
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

/// Extract BCM segments from a framebuffer into an existing [`SegmentCache`].
///
/// Builds directly into `cache`, writing only the first `count` entries.
/// Callers that need a stack build should use [`segments_from_fb`].
pub(crate) fn segments_from_fb_into<FB: FrameBuffer>(fb: &FB, cache: &mut SegmentCache) {
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
    cache.count = count;
    cache.segments_per_group = spg;
    for i in 0..count {
        let seg = fb.bcm_segment(i);
        debug_assert!(!seg.ptr.is_null(), "segment {i} returned a null pointer");
        // Verify that the runtime segment agrees with the static shape array
        // (catches `FrameBuffer` implementations whose `bcm_segment()` and
        // `BCM_SEGMENT_SHAPES` are out of sync before a descriptor overflow).
        debug_assert!(
            {
                let (shape_len, shape_reps) = FB::BCM_SEGMENT_SHAPES[i % FB::BCM_SEQUENCE_LEN];
                seg.len == shape_len && seg.reps == shape_reps
            },
            "bcm_segment({i}) len={} reps={} disagrees with BCM_SEGMENT_SHAPES {shape:?}",
            seg.len,
            seg.reps,
            shape = FB::BCM_SEGMENT_SHAPES[i % FB::BCM_SEQUENCE_LEN],
        );
        cache.segments[i] = seg;
    }
}

/// Convenience wrapper that returns a fresh [`SegmentCache`] by value.
///
/// Prefer [`segments_from_fb_into`] in performance-sensitive paths — this
/// construction requires ~3.9 KB of stack headroom. Only used by
/// circular-DMA init.
#[cfg(feature = "circular-dma")]
pub(crate) fn segments_from_fb<FB: FrameBuffer>(fb: &FB) -> SegmentCache {
    let mut cache = SegmentCache::new();
    segments_from_fb_into(fb, &mut cache);
    cache
}

/// Build a `Preparation` pointing to the first descriptor in a chain.
///
/// Shared by both linear and circular buffer implementations.
#[cfg_attr(feature = "iram", ram)]
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
#[cfg_attr(feature = "iram", ram)]
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
