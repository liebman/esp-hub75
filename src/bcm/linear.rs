//! Linear (non-circular) BCM DMA buffer.
//!
//! `BcmBuf` implements [`DmaTxBuffer`] and provides the BCM state machine
//! used by interrupt-driven display refresh. Each transfer is a linear
//! descriptor chain (last `next = null`) that the ISR rebuilds after every
//! completion.
//!
//! This module also owns the linear-only machinery the buffer relies on: the
//! static [`SegmentCache`] (materialised here and never in circular mode),
//! [`fill_segment_cache`], and the descriptor-count helpers the ISR uses to
//! size each transfer.

use core::cell::UnsafeCell;
use core::ptr::null;
use core::ptr::null_mut;

use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::dma::Preparation;
#[cfg(feature = "iram")]
use esp_hal::ram;

use crate::framebuffer::BcmSegment;
use crate::framebuffer::FrameBuffer;

// ---------------------------------------------------------------------------
// Segment cache (linear-only ISR state)
// ---------------------------------------------------------------------------

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

/// Single segment cache for ISR use.
///
/// One static slot holds the full segment sequence. `swap()` stores a
/// pointer delta (old FB → new FB) and the ISR applies it at the frame
/// boundary, the same approach as circular mode. No copies, no slots.
struct CacheCell(UnsafeCell<SegmentCache>);

// SAFETY: All access is serialised by the ISR state lock (the `STATE` static
// in `isr`, an `esp_sync::NonReentrantMutex`). The cache is written only by
// `start_internal()` (init/restart path) and the ISR (delta application at
// frame boundaries); `swap()` never touches it.
unsafe impl Sync for CacheCell {}

static SEGMENT_CACHE: CacheCell = CacheCell(UnsafeCell::new(SegmentCache::new()));

/// Return a raw pointer to the static segment cache.
///
/// Used by [`BcmBuf`] and `start_internal()`.
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
    #[cfg_attr(feature = "iram", ram)]
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

/// Fill the static [`SegmentCache`] from a framebuffer's BCM segments.
///
/// Circular-DMA init streams segments straight from the framebuffer (see
/// [`super::fill_descriptor_chain`]); no cache is materialised there, so the
/// ~3.9 KB `SegmentCache` never touches the stack or BSS in circular mode.
pub(crate) fn fill_segment_cache<FB: FrameBuffer>(fb: &FB, cache: &mut SegmentCache) {
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
        spg > 0 && count.is_multiple_of(spg),
        "bcm_segment_count {count} not divisible by segments_per_group {spg}"
    );
    cache.count = count;
    cache.segments_per_group = spg;
    for i in 0..count {
        let segment = fb.bcm_segment(i);
        debug_assert!(
            !segment.ptr.is_null(),
            "segment {i} returned a null pointer"
        );
        cache.segments[i] = segment;
    }
}

/// ISR-driven BCM DMA transmit buffer.
///
/// Behaviour depends on the `full-chain-dma` feature:
/// - **Default (group-based):** `prepare()` links descriptors for the current
///   group of segments (all segments within the group, each repeated its `reps`
///   times). The ISR calls `advance()` after each group transfer to walk
///   through the groups. For frame-major framebuffers each group is one plane;
///   for row-major framebuffers each group is one row's complete BCM cycle.
/// - **`full-chain-dma`:** `prepare()` links the full BCM repetition chain.
///   `advance()` always returns `true` (every transfer is a complete frame).
pub(crate) struct BcmBuf {
    descriptors: &'static mut [DmaDescriptor],
    /// Raw pointer to the static segment cache (`SEGMENT_CACHE`).
    /// Dereferenced inline at each use site; the pointer is always
    /// valid because the cache is a `'static` and all access is
    /// serialised by the ISR state lock (`STATE` in `isr`).
    cache: *const SegmentCache,
    #[cfg(not(feature = "full-chain-dma"))]
    current_group: usize,
}

impl BcmBuf {
    pub(crate) fn new(descriptors: &'static mut [DmaDescriptor]) -> Self {
        Self {
            descriptors,
            cache: cache_ptr(),
            #[cfg(not(feature = "full-chain-dma"))]
            current_group: 0,
        }
    }

    /// Validate the cache and reset the BCM state machine.
    ///
    /// Called during `start()`/`restart()`. The cache pointer already
    /// points at `SEGMENT_CACHE` — this just re-validates and resets
    /// `current_group`.
    pub(crate) fn reset_with_cache(&mut self) {
        // SAFETY: self.cache points to SEGMENT_CACHE; accessible because
        // we just wrote it under critical section.
        let cache = unsafe { &*self.cache };
        debug_assert!(cache.count > 0 && cache.count <= MAX_SEGMENTS);
        #[cfg(feature = "full-chain-dma")]
        let needed = cache.descriptor_count();
        #[cfg(not(feature = "full-chain-dma"))]
        let needed = cache.max_group_descriptor_count();
        debug_assert!(
            self.descriptors.len() >= needed,
            "not enough DMA descriptors: have {}, need {}",
            self.descriptors.len(),
            needed,
        );
        #[cfg(not(feature = "full-chain-dma"))]
        {
            self.current_group = 0;
        }
    }

    /// Advance the BCM state machine after a transfer completes.
    /// Returns `true` when a full BCM frame boundary is reached.
    #[allow(clippy::unused_self)]
    #[cfg_attr(feature = "iram", ram)]
    pub(crate) fn advance(&mut self) -> bool {
        #[cfg(feature = "full-chain-dma")]
        {
            true
        }
        #[cfg(not(feature = "full-chain-dma"))]
        {
            // SAFETY: self.cache is valid under cs/ISR serialisation.
            let group_count = unsafe { (*self.cache).group_count() };
            self.current_group += 1;
            if self.current_group >= group_count {
                self.current_group = 0;
                return true;
            }
            false
        }
    }

    /// Apply a framebuffer pointer delta to all cached segments.
    ///
    /// Called at frame boundaries when a swap is pending. Every segment's
    /// `ptr` is shifted by `delta` — the byte offset between the old and
    /// new framebuffer allocations. Same invariant as circular-DMA mode:
    /// both framebuffers are the same type with identical internal layout.
    #[cfg_attr(feature = "iram", ram)]
    pub(crate) fn apply_delta(&mut self, delta: isize) {
        // SAFETY: Called from the ISR (under the ISR state lock (`STATE`)). The cache
        // is not concurrently accessed; `swap()` only reads FB pointers
        // to compute the delta and never writes to the cache.
        let cache = unsafe { &mut *self.cache.cast_mut() };
        for i in 0..cache.count {
            cache.segments[i].ptr = cache.segments[i].ptr.wrapping_byte_offset(delta);
        }
    }

    /// Byte length of the next DMA transfer that `prepare()` will build.
    #[cfg(esp32c6)]
    #[cfg_attr(feature = "iram", ram)]
    pub(crate) fn current_transfer_len(&self) -> usize {
        // SAFETY: self.cache is valid under cs/ISR serialisation.
        let cache = unsafe { &*self.cache };
        #[cfg(feature = "full-chain-dma")]
        {
            let mut total = 0;
            for i in 0..cache.count {
                let seg = &cache.segments[i];
                total += seg.len * seg.reps;
            }
            total
        }
        #[cfg(not(feature = "full-chain-dma"))]
        {
            cache.group_byte_count(self.current_group)
        }
    }
}

// SAFETY: All access to `BcmBuf` is serialised by the ISR state lock
// (the `STATE` static in `isr`, an `esp_sync::NonReentrantMutex`): it disables
// interrupts on the current core and CAS-spins on an owner word on
// multi-core chips like ESP32 and ESP32-S3. There is therefore no
// concurrent access. The raw `cache` pointer points to `SEGMENT_CACHE`
// (a `'static`), which is only mutated under the same lock (by the ISR
// applying deltas and by `start_internal` rebuilding it).
unsafe impl Send for BcmBuf {}

unsafe impl DmaTxBuffer for BcmBuf {
    type View = Self;
    type Final = Self;

    #[cfg_attr(feature = "iram", ram)]
    fn prepare(&mut self) -> Preparation {
        self.prepare_descriptors()
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

impl BcmBuf {
    #[cfg(not(feature = "full-chain-dma"))]
    #[cfg_attr(feature = "iram", ram)]
    fn prepare_descriptors(&mut self) -> Preparation {
        // SAFETY: self.cache is valid under cs/ISR serialisation.
        // The descriptor writes below never alias the cache slots.
        let cache = unsafe { &*self.cache };
        let spg = cache.segments_per_group;
        let start = self.current_group * spg;
        let total_descs = cache.group_descriptor_count(self.current_group);
        // Link this group's segments into a linear chain (last `next = null`)
        // with the shared chain builder; the closure offsets into the cache so
        // the builder sees a self-contained group.
        super::fill_descriptor_chain(
            &mut self.descriptors[..total_descs],
            spg,
            |i| cache.segments[start + i],
            total_descs,
            null_mut(),
            // The last descriptor's `suc_eof` ends the group transfer so the
            // ISR can advance/restart it.
            true,
        );
        super::make_preparation(self.descriptors)
    }

    #[cfg(feature = "full-chain-dma")]
    #[cfg_attr(feature = "iram", ram)]
    fn prepare_descriptors(&mut self) -> Preparation {
        // SAFETY: self.cache is valid under cs/ISR serialisation.
        let cache = unsafe { &*self.cache };
        let total_descs = cache.descriptor_count();
        super::fill_descriptor_chain(
            &mut self.descriptors[..total_descs],
            cache.count,
            |i| cache.segments[i],
            total_descs,
            null_mut(),
            // Linear full-chain mode: the last descriptor's `suc_eof` ends
            // the transfer so the ISR can advance/restart it.
            true,
        );
        super::make_preparation(self.descriptors)
    }
}
