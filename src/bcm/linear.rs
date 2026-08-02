//! Linear (non-circular) BCM DMA buffer.
//!
//! `BcmBuf` implements [`DmaTxBuffer`] and provides the BCM state machine
//! used by interrupt-driven display refresh. Each transfer is a linear
//! descriptor chain (last `next = null`) that the ISR rebuilds after every
//! completion.

use core::ptr::null;
use core::ptr::null_mut;

use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaTxBuffer;
#[cfg(not(feature = "full-chain-dma"))]
use esp_hal::dma::Owner;
use esp_hal::dma::Preparation;
#[cfg(feature = "iram")]
use esp_hal::ram;

use super::PlaneInfo;
#[cfg(not(feature = "full-chain-dma"))]
use crate::MAX_DMA_CHUNK_SIZE;

/// ISR-driven BCM DMA transmit buffer.
///
/// Behaviour depends on the `full-chain-dma` feature:
/// - **Default (single-plane):** `prepare()` links descriptors for the current
///   plane only. The ISR calls `advance()` after each transfer to walk the BCM
///   weighting sequence.
/// - **`full-chain-dma`:** `prepare()` links the full BCM repetition chain.
///   `advance()` always returns `true` (every transfer is a complete frame).
pub(crate) struct BcmBuf {
    descriptors: &'static mut [DmaDescriptor],
    planes: PlaneInfo,
    plane_count: usize,
    #[cfg(not(feature = "full-chain-dma"))]
    current_plane: usize,
    #[cfg(not(feature = "full-chain-dma"))]
    current_rep: usize,
}

impl BcmBuf {
    pub(crate) fn new(descriptors: &'static mut [DmaDescriptor]) -> Self {
        Self {
            descriptors,
            planes: [(null::<u8>(), 0usize); super::MAX_PLANES],
            plane_count: 0,
            #[cfg(not(feature = "full-chain-dma"))]
            current_plane: 0,
            #[cfg(not(feature = "full-chain-dma"))]
            current_rep: 0,
        }
    }

    /// Set plane pointers and count, resetting the BCM state machine.
    pub(crate) fn reset_with_planes(&mut self, new_planes: PlaneInfo, plane_count: usize) {
        debug_assert!(plane_count > 0 && plane_count <= super::MAX_PLANES);
        debug_assert!(
            self.descriptors.len() >= crate::dma_descriptor_count(plane_count, new_planes[0].1),
            "not enough DMA descriptors: have {}, need {}",
            self.descriptors.len(),
            crate::dma_descriptor_count(plane_count, new_planes[0].1),
        );
        self.planes = new_planes;
        self.plane_count = plane_count;
        #[cfg(not(feature = "full-chain-dma"))]
        {
            self.current_plane = 0;
            self.current_rep = 0;
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
            self.current_rep += 1;
            let reps = 1usize << (self.plane_count - 1 - self.current_plane);
            if self.current_rep >= reps {
                self.current_rep = 0;
                self.current_plane += 1;
                if self.current_plane >= self.plane_count {
                    self.current_plane = 0;
                    return true;
                }
            }
            false
        }
    }

    /// Replace the stored plane pointers (called at frame-boundary swap).
    #[cfg_attr(feature = "iram", ram)]
    pub(crate) fn update_planes(&mut self, new_planes: PlaneInfo) {
        self.planes = new_planes;
    }

    /// Byte length of the next DMA transfer that `prepare()` will build.
    #[cfg(esp32c6)]
    #[cfg_attr(feature = "iram", ram)]
    pub(crate) fn current_transfer_len(&self) -> usize {
        #[cfg(feature = "full-chain-dma")]
        {
            let plane_bytes = self.planes[0].1;
            let total_reps = (1usize << self.plane_count) - 1;
            plane_bytes * total_reps
        }
        #[cfg(not(feature = "full-chain-dma"))]
        {
            self.planes[self.current_plane].1
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
        let (ptr, len) = self.planes[self.current_plane];
        let desc_count = len.div_ceil(MAX_DMA_CHUNK_SIZE);
        let mut remaining = len;
        let mut offset = 0;

        for i in 0..desc_count {
            let chunk = remaining.min(MAX_DMA_CHUNK_SIZE);
            let is_last = i + 1 == desc_count;
            let next = if is_last {
                null_mut()
            } else {
                unsafe { self.descriptors.as_mut_ptr().add(i + 1) }
            };
            let desc = &mut self.descriptors[i];
            // SAFETY: `ptr` originates from a live framebuffer plane and
            // `offset` stays within the plane's `len` bytes.
            desc.buffer = unsafe { ptr.add(offset) as *mut u8 };
            desc.set_size(chunk);
            desc.set_length(chunk);
            desc.set_owner(Owner::Dma);
            desc.set_suc_eof(is_last);
            desc.next = next;
            remaining -= chunk;
            offset += chunk;
        }

        super::make_preparation(self.descriptors)
    }

    #[cfg(feature = "full-chain-dma")]
    #[cfg_attr(feature = "iram", ram)]
    fn prepare_descriptors(&mut self) -> Preparation {
        let total_descs = crate::dma_descriptor_count(self.plane_count, self.planes[0].1);
        super::fill_full_chain(
            &mut self.descriptors[..total_descs],
            &self.planes,
            self.plane_count,
            total_descs,
            null_mut(),
        );
        super::make_preparation(self.descriptors)
    }
}
