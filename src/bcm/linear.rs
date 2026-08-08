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
    cache: SegmentCache,
    #[cfg(not(feature = "full-chain-dma"))]
    current_group: usize,
}

impl BcmBuf {
    pub(crate) fn new(descriptors: &'static mut [DmaDescriptor]) -> Self {
        Self {
            descriptors,
            cache: SegmentCache::new(),
            #[cfg(not(feature = "full-chain-dma"))]
            current_group: 0,
        }
    }

    /// Set segment data, resetting the BCM state machine.
    pub(crate) fn reset_with_segments(&mut self, new_cache: SegmentCache) {
        debug_assert!(new_cache.count > 0 && new_cache.count <= super::MAX_SEGMENTS);
        // Group-based mode rebuilds the descriptor table for every transfer
        // and only ever needs the largest group's descriptors; full-chain
        // mode links the entire BCM sequence at once.
        #[cfg(feature = "full-chain-dma")]
        let needed = new_cache.descriptor_count();
        #[cfg(not(feature = "full-chain-dma"))]
        let needed = new_cache.max_group_descriptor_count();
        debug_assert!(
            self.descriptors.len() >= needed,
            "not enough DMA descriptors: have {}, need {}",
            self.descriptors.len(),
            needed,
        );
        self.cache = new_cache;
        #[cfg(not(feature = "full-chain-dma"))]
        {
            self.current_group = 0;
        }
    }

    /// Advance the BCM state machine after a transfer completes.
    /// Returns `true` when a full BCM frame boundary is reached.
    #[cfg_attr(feature = "iram", ram)]
    pub(crate) fn advance(&mut self) -> bool {
        #[cfg(feature = "full-chain-dma")]
        {
            true
        }
        #[cfg(not(feature = "full-chain-dma"))]
        {
            self.current_group += 1;
            if self.current_group >= self.cache.group_count() {
                self.current_group = 0;
                return true;
            }
            false
        }
    }

    /// Replace the stored segment data (called at frame-boundary swap).
    #[cfg_attr(feature = "iram", ram)]
    pub(crate) fn update_segments(&mut self, new_cache: SegmentCache) {
        self.cache = new_cache;
    }

    /// Byte length of the next DMA transfer that `prepare()` will build.
    #[cfg(esp32c6)]
    #[cfg_attr(feature = "iram", ram)]
    pub(crate) fn current_transfer_len(&self) -> usize {
        #[cfg(feature = "full-chain-dma")]
        {
            let mut total = 0;
            for i in 0..self.cache.count {
                let seg = &self.cache.segments[i];
                total += seg.len * seg.reps;
            }
            total
        }
        #[cfg(not(feature = "full-chain-dma"))]
        {
            self.cache.group_byte_count(self.current_group)
        }
    }
}

// SAFETY: All access to `BcmBuf` is serialised by `critical_section::with`,
// which on esp-hal provides a cross-core critical section (interrupt-disable
// plus a cross-core spinlock on multi-core chips like ESP32 and ESP32-S3).
// There is therefore no concurrent access.
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
        let spg = self.cache.segments_per_group;
        let start = self.current_group * spg;
        let end = start + spg;
        let total_descs = self.cache.group_descriptor_count(self.current_group);
        let base_ptr = self.descriptors.as_mut_ptr();
        let mut desc_idx = 0;

        for seg_idx in start..end {
            let seg = &self.cache.segments[seg_idx];
            for _ in 0..seg.reps {
                let mut remaining = seg.len;
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

        super::make_preparation(self.descriptors)
    }

    #[cfg(feature = "full-chain-dma")]
    #[cfg_attr(feature = "iram", ram)]
    fn prepare_descriptors(&mut self) -> Preparation {
        let total_descs = self.cache.descriptor_count();
        super::fill_full_chain(
            &mut self.descriptors[..total_descs],
            &self.cache,
            total_descs,
            null_mut(),
        );
        super::make_preparation(self.descriptors)
    }
}
