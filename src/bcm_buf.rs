//! Unified BCM DMA buffer for ISR-driven HUB75 drivers.
//!
//! `BcmBuf` implements [`DmaTxBuffer`] and provides the BCM (Binary Code
//! Modulation) state machine used by interrupt-driven display refresh.
//!
//! By default, each call to [`DmaTxBuffer::prepare`] builds descriptors for a
//! **single bit-plane** and [`advance()`](BcmBuf::advance) walks through the
//! BCM weighting sequence. With the `full-chain-dma` feature enabled,
//! `prepare()` builds the **entire BCM repetition chain** in one shot and
//! `advance()` always signals a frame boundary — reducing interrupt count at
//! the cost of more descriptor RAM and the ESP32-C6 65535-byte transfer limit.

use core::ptr::null;
use core::ptr::null_mut;

use esp_hal::dma::BurstConfig;
use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::dma::EmptyBuf;
use esp_hal::dma::Owner;
use esp_hal::dma::Preparation;
use esp_hal::dma::TransferDirection;
#[cfg(feature = "iram")]
use esp_hal::ram;

use crate::framebuffer::FrameBuffer;
use crate::MAX_DMA_CHUNK_SIZE;

pub(crate) const MAX_PLANES: usize = 8;

pub(crate) type PlaneInfo = [(*const u8, usize); MAX_PLANES];

/// Extract plane pointers from a framebuffer into a `PlaneInfo` array.
pub(crate) fn planes_from_fb(fb: &impl FrameBuffer) -> PlaneInfo {
    let plane_count = fb.plane_count();
    assert!(
        plane_count <= MAX_PLANES,
        "plane_count {plane_count} exceeds MAX_PLANES"
    );
    let mut planes: PlaneInfo = [(null::<u8>(), 0usize); MAX_PLANES];
    for (i, slot) in planes.iter_mut().enumerate().take(plane_count) {
        *slot = fb.plane_ptr_len(i);
        debug_assert!(!slot.0.is_null(), "plane {i} returned a null pointer");
    }
    planes
}

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
            planes: [(null::<u8>(), 0usize); MAX_PLANES],
            plane_count: 0,
            #[cfg(not(feature = "full-chain-dma"))]
            current_plane: 0,
            #[cfg(not(feature = "full-chain-dma"))]
            current_rep: 0,
        }
    }

    /// Set plane pointers and count, resetting the BCM state machine.
    pub(crate) fn reset_with_planes(&mut self, new_planes: PlaneInfo, plane_count: usize) {
        debug_assert!(plane_count > 0 && plane_count <= MAX_PLANES);
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
    /// Build DMA descriptors for the next transfer.
    ///
    /// Without `full-chain-dma` this covers a single bit-plane; with the
    /// feature enabled it covers the entire BCM repetition chain.
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

        // `EmptyBuf` provides a `Preparation` with safe defaults; we override
        // the fields relevant to our descriptor chain. If `Preparation` gains
        // new fields in a future esp-hal release, review them here.
        let mut empty = EmptyBuf;
        let mut prep: Preparation = empty.prepare();
        prep.start = self.descriptors.as_mut_ptr();
        prep.direction = TransferDirection::Out;
        prep.burst_transfer = BurstConfig::default();
        prep.check_owner = Some(false);
        prep.auto_write_back = false;
        prep
    }

    /// Build DMA descriptors for the next transfer.
    ///
    /// Without `full-chain-dma` this covers a single bit-plane; with the
    /// feature enabled it covers the entire BCM repetition chain.
    #[cfg(feature = "full-chain-dma")]
    #[cfg_attr(feature = "iram", ram)]
    fn prepare_descriptors(&mut self) -> Preparation {
        let total_descs = crate::dma_descriptor_count(self.plane_count, self.planes[0].1);
        let mut desc_idx = 0;

        for plane_idx in 0..self.plane_count {
            let reps = 1usize << (self.plane_count - 1 - plane_idx);
            let (plane_ptr, plane_bytes) = self.planes[plane_idx];

            for _ in 0..reps {
                let mut remaining = plane_bytes;
                let mut offset = 0;
                while remaining > 0 {
                    let chunk = remaining.min(MAX_DMA_CHUNK_SIZE);
                    let is_last = desc_idx + 1 == total_descs;
                    let next = if is_last {
                        null_mut()
                    } else {
                        unsafe { self.descriptors.as_mut_ptr().add(desc_idx + 1) }
                    };
                    let desc = &mut self.descriptors[desc_idx];
                    // SAFETY: `plane_ptr` originates from a live framebuffer
                    // plane and `offset` stays within the plane's byte length.
                    desc.buffer = unsafe { plane_ptr.add(offset) as *mut u8 };
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

        // `EmptyBuf` provides a `Preparation` with safe defaults; we override
        // the fields relevant to our descriptor chain. If `Preparation` gains
        // new fields in a future esp-hal release, review them here.
        let mut empty = EmptyBuf;
        let mut prep: Preparation = empty.prepare();
        prep.start = self.descriptors.as_mut_ptr();
        prep.direction = TransferDirection::Out;
        prep.burst_transfer = BurstConfig::default();
        prep.check_owner = Some(false);
        prep.auto_write_back = false;
        prep
    }
}
