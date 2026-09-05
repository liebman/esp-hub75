//! Linear (non-circular) BCM DMA buffer.
//!
//! `BcmBuf` implements [`DmaTxBuffer`] and provides the BCM state machine
//! used by interrupt-driven display refresh. Each transfer is a linear
//! descriptor chain (last `next = null`) that the ISR rebuilds after every
//! completion.

use core::ptr::null_mut;

use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaTxBuffer;
#[cfg(not(feature = "full-chain-dma"))]
use esp_hal::dma::Owner;
use esp_hal::dma::Preparation;
#[cfg(feature = "iram")]
use esp_hal::ram;

use super::SegmentCache;
#[cfg(not(feature = "full-chain-dma"))]
use crate::MAX_DMA_CHUNK_SIZE;

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
    /// serialised by the ISR state lock (`STATE_LOCK`).
    cache: *const SegmentCache,
    #[cfg(not(feature = "full-chain-dma"))]
    current_group: usize,
}

impl BcmBuf {
    pub(crate) fn new(descriptors: &'static mut [DmaDescriptor]) -> Self {
        Self {
            descriptors,
            cache: super::cache_ptr(),
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
        debug_assert!(cache.count > 0 && cache.count <= super::MAX_SEGMENTS);
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
        // SAFETY: Called from the ISR (under the ISR `STATE_LOCK`). The cache
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
// (`STATE_LOCK` in `isr.rs`, an `esp_sync::RawMutex`): it disables
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

    fn into_view(self) -> Self::View {
        self
    }

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
        let end = start + spg;
        let total_descs = cache.group_descriptor_count(self.current_group);
        let base_ptr = self.descriptors.as_mut_ptr();
        let mut desc_idx = 0;

        for seg_idx in start..end {
            let segment = &cache.segments[seg_idx];
            for _ in 0..segment.reps {
                let mut remaining = segment.len;
                let mut offset = 0;
                while remaining > 0 {
                    let chunk = remaining.min(MAX_DMA_CHUNK_SIZE);
                    let is_last = desc_idx + 1 == total_descs;
                    let next = if is_last {
                        null_mut()
                    } else {
                        unsafe { base_ptr.add(desc_idx + 1) }
                    };
                    let desc = &mut self.descriptors[desc_idx];
                    // SAFETY: `seg.ptr` originates from a live framebuffer
                    // and `offset` stays within the segment's byte length.
                    desc.buffer = unsafe { segment.ptr.add(offset).cast_mut() };
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

        super::make_preparation(self.descriptors)
    }

    #[cfg(feature = "full-chain-dma")]
    #[cfg_attr(feature = "iram", ram)]
    fn prepare_descriptors(&mut self) -> Preparation {
        // SAFETY: self.cache is valid under cs/ISR serialisation.
        let cache = unsafe { &*self.cache };
        let total_descs = cache.descriptor_count();
        super::fill_full_chain(
            &mut self.descriptors[..total_descs],
            cache.count,
            |i| cache.segments[i],
            total_descs,
            null_mut(),
        );
        super::make_preparation(self.descriptors)
    }
}
