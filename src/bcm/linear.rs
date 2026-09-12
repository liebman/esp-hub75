//! Linear (group-based) BCM DMA buffer.
//!
//! `BcmBuf` implements [`DmaTxBuffer`] and provides the BCM state machine used
//! by interrupt-driven display refresh in the default (no `full-chain-dma`)
//! mode. Each transfer links one *group* of BCM segments — a row's worth for
//! row-major framebuffers, one plane for frame-major ones — and the ISR walks
//! the groups, applying any pending swap at a full-frame boundary.
//!
//! This module owns the linear-only machinery the buffer relies on: the static
//! [`SegmentCache`], its [`fill`](SegmentCache::fill) method, and the
//! descriptor-count helpers the ISR uses to size each transfer. The build-once
//! `full-chain-dma` / `circular-dma` chains live in `bcm::full_chain` instead.

use core::ptr::null;
use core::ptr::null_mut;

use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::dma::Preparation;
#[cfg(feature = "iram")]
use esp_hal::ram;
use static_cell::StaticCell;

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

/// Upper bound on the number of DMA transfer groups within one BCM period.
///
/// A group holds at least one segment, so this cannot exceed the period
/// length, whose worst case is
/// [`BCM_SEQUENCE_CAPACITY`](crate::framebuffer::BCM_SEQUENCE_CAPACITY).
const MAX_GROUPS_PER_PERIOD: usize = crate::framebuffer::BCM_SEQUENCE_CAPACITY;

const EMPTY_SEGMENT: BcmSegment = BcmSegment {
    ptr: null(),
    len: 0,
    reps: 0,
};

/// Static storage for the single segment cache.
///
/// `StaticCell` supplies the `Sync` static and hands out one `&'static mut` on
/// [`init_with`](StaticCell::init_with). It is safe here even though
/// `SegmentCache` is `!Send`/`!Sync` (its [`BcmSegment`]s hold raw pointers):
/// the taken reference is stored in [`BcmBuf`] and shared with the ISR through
/// the `STATE` lock, so the cell itself is touched only once, at construction.
///
/// A second `init_with` panics, which can only mean a second driver was built
/// (already prevented by `claim_driver`).
static SEGMENT_CACHE: StaticCell<SegmentCache> = StaticCell::new();

/// Initialise (once) and take the static segment cache.
///
/// `init_with` constructs the ~3.9 KB cache in place instead of on the stack.
fn segment_cache() -> &'static mut SegmentCache {
    SEGMENT_CACHE.init_with(SegmentCache::new)
}

/// Cached BCM segment data for ISR use.
///
/// Stores the full segment sequence extracted from a `FrameBuffer` so the
/// ISR can drive DMA without calling trait methods (the framebuffer type is
/// erased in the ISR statics), plus the per-group descriptor counts
/// precomputed from the framebuffer's compile-time BCM sequence.
pub(crate) struct SegmentCache {
    pub(crate) segments: [BcmSegment; MAX_SEGMENTS],
    pub(crate) count: usize,
    /// Consecutive segments that form one DMA transfer group.
    /// The ISR builds a descriptor chain for a whole group and fires
    /// only at group boundaries.
    pub(crate) segments_per_group: usize,
    /// Descriptors per group within one period, indexed by
    /// `group % groups_per_period`.
    pub(crate) group_descs: [usize; MAX_GROUPS_PER_PERIOD],
    /// Number of groups in one period (the modulus above).
    pub(crate) groups_per_period: usize,
}

impl SegmentCache {
    pub const fn new() -> Self {
        Self {
            segments: [EMPTY_SEGMENT; MAX_SEGMENTS],
            count: 0,
            segments_per_group: 1,
            group_descs: [0; MAX_GROUPS_PER_PERIOD],
            groups_per_period: 1,
        }
    }

    /// Fill the cache from a framebuffer's BCM segments and precompute the
    /// per-group descriptor counts the ISR needs.
    ///
    /// The build-once `full-chain-dma` / `circular-dma` paths stream segments
    /// straight from the framebuffer (`bcm::full_chain`); no cache is
    /// materialised there, so the ~3.9 KB `SegmentCache` only exists in this
    /// group-based mode. Called once per binding, from `BcmBuf::bind_cache`.
    pub(crate) fn fill<FB: FrameBuffer>(&mut self, fb: &FB) {
        // Compile-time checks: the cache can hold the framebuffer's full scan
        // sequence, and its groups divide the period evenly (groups never
        // straddle a period). Evaluated per monomorphization.
        const {
            assert!(
                FB::BCM_SEGMENT_COUNT <= MAX_SEGMENTS,
                "framebuffer BCM segment count exceeds MAX_SEGMENTS"
            );
            assert!(
                FB::BCM_SEQUENCE_LEN % FB::BCM_SEGMENTS_PER_GROUP == 0,
                "BCM_SEQUENCE_LEN must be divisible by BCM_SEGMENTS_PER_GROUP"
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
        self.count = count;
        self.segments_per_group = spg;
        for i in 0..count {
            let segment = fb.bcm_segment(i);
            debug_assert!(
                !segment.ptr.is_null(),
                "segment {i} returned a null pointer"
            );
            self.segments[i] = segment;
        }

        // Precompute the descriptor counts the ISR needs, from the framebuffer's
        // compile-time BCM sequence, via the same helper `dma_descriptor_count`
        // uses. Groups never straddle a period, so a group's count repeats every
        // period and is indexed as `group % groups_per_period`.
        let groups_per_period = FB::BCM_SEQUENCE_LEN / FB::BCM_SEGMENTS_PER_GROUP;
        assert!(
            groups_per_period <= MAX_GROUPS_PER_PERIOD,
            "groups per period {groups_per_period} exceeds MAX_GROUPS_PER_PERIOD"
        );
        self.groups_per_period = groups_per_period;
        let mut group = 0;
        while group < groups_per_period {
            self.group_descs[group] = crate::group_descriptor_count::<FB>(
                group,
                FB::BCM_SEGMENTS_PER_GROUP,
                crate::MAX_DMA_CHUNK_SIZE,
            );
            group += 1;
        }
    }

    /// Number of DMA transfer groups in this sequence.
    #[cfg_attr(feature = "iram", ram)]
    pub fn group_count(&self) -> usize {
        self.count / self.segments_per_group
    }

    /// DMA descriptors required for a single group starting at `group_idx`.
    ///
    /// Precomputed once from the framebuffer type (see [`fill`](Self::fill))
    /// with the same arithmetic as [`crate::dma_descriptor_count`], so the ISR
    /// only indexes a table.
    #[cfg_attr(feature = "iram", ram)]
    pub fn group_descriptor_count(&self, group_idx: usize) -> usize {
        self.group_descs[group_idx % self.groups_per_period]
    }

    /// Total bytes in a single group (all segments × their reps).
    #[cfg(esp32c6)]
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

/// ISR-driven BCM DMA transmit buffer (group-based refresh).
///
/// `prepare()` links descriptors for the current group of segments (all
/// segments within the group, each repeated its `reps` times). The ISR calls
/// [`advance`](Self::advance) after each group transfer to walk through the
/// groups. For frame-major framebuffers each group is one plane; for row-major
/// framebuffers each group is one row's complete BCM cycle.
pub(crate) struct BcmBuf {
    descriptors: &'static mut [DmaDescriptor],
    /// The static segment cache, taken once in [`new`](Self::new) and shared
    /// with the ISR through the `STATE` lock.
    cache: &'static mut SegmentCache,
    current_group: usize,
}

impl BcmBuf {
    pub(crate) fn new(descriptors: &'static mut [DmaDescriptor]) -> Self {
        Self {
            descriptors,
            cache: segment_cache(),
            current_group: 0,
        }
    }

    /// Bind the buffer to `fb`: rebuild the segment cache and reset the BCM
    /// state machine.
    ///
    /// Called from `start_internal()` (init/restart path) before the first
    /// transfer. The `full-chain-dma` / `circular-dma` modes build their
    /// descriptor chain instead (see `full_chain::BcmBuf::build`).
    pub(crate) fn bind_cache<FB: FrameBuffer>(&mut self, fb: &FB) {
        self.cache.fill(fb);
        // The static descriptor storage is sized by `dma_descriptor_count`, the
        // single source of truth for the largest-group invariant. This is a
        // hard check on a type-level invariant, so it only needs to run once.
        debug_assert!(
            self.descriptors.len() >= crate::dma_descriptor_count::<FB>(crate::MAX_DMA_CHUNK_SIZE),
            "not enough DMA descriptors: have {}, need {}",
            self.descriptors.len(),
            crate::dma_descriptor_count::<FB>(crate::MAX_DMA_CHUNK_SIZE),
        );
        self.current_group = 0;
    }

    /// Advance the BCM state machine after a transfer completes.
    /// Returns `true` when a full BCM frame boundary is reached.
    #[cfg_attr(feature = "iram", ram)]
    pub(crate) fn advance(&mut self) -> bool {
        let group_count = self.cache.group_count();
        self.current_group += 1;
        if self.current_group >= group_count {
            self.current_group = 0;
            return true;
        }
        false
    }

    /// Apply a framebuffer pointer delta to all cached segments.
    ///
    /// Called at frame boundaries when a swap is pending. Every segment's
    /// `ptr` is shifted by `delta` — the byte offset between the old and
    /// new framebuffer allocations. Same invariant as circular-DMA mode:
    /// both framebuffers are the same type with identical internal layout.
    #[cfg_attr(feature = "iram", ram)]
    pub(crate) fn apply_delta(&mut self, delta: isize) {
        // Called from the ISR under the ISR state lock (`STATE`), so the cache
        // is not concurrently accessed; `swap()` only reads FB pointers to
        // compute the delta and never writes to the cache.
        let cache = &mut *self.cache;
        for i in 0..cache.count {
            cache.segments[i].ptr = cache.segments[i].ptr.wrapping_byte_offset(delta);
        }
    }

    /// Byte length of the next DMA transfer that `prepare()` will build.
    #[cfg(esp32c6)]
    #[cfg_attr(feature = "iram", ram)]
    pub(crate) fn current_transfer_len(&self) -> usize {
        self.cache.group_byte_count(self.current_group)
    }
}

// SAFETY: All access to `BcmBuf` is serialised by the ISR state lock
// (the `STATE` static in `isr`, an `esp_sync::NonReentrantMutex`): it disables
// interrupts on the current core and CAS-spins on an owner word on
// multi-core chips like ESP32 and ESP32-S3. There is therefore no
// concurrent access. The `cache` reference points into `SEGMENT_CACHE` (a
// `'static`), which is only mutated under the same lock (by the ISR applying
// deltas and by `start_internal` rebuilding it).
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
    #[cfg_attr(feature = "iram", ram)]
    fn prepare_descriptors(&mut self) -> Preparation {
        let cache = &*self.cache;
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
}
