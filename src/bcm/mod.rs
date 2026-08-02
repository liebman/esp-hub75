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

use crate::framebuffer::FrameBuffer;
#[cfg(any(feature = "full-chain-dma", feature = "circular-dma"))]
use crate::MAX_DMA_CHUNK_SIZE;

#[cfg(feature = "circular-dma")]
pub(crate) mod circular;
#[cfg(not(feature = "circular-dma"))]
pub(crate) mod linear;

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
/// Fill a full-chain BCM descriptor sequence.
///
/// The caller provides the `next` pointer for the last descriptor in the
/// chain — `null_mut()` for linear mode (last `next` = null),
/// `ring_start` (points back to `desc[0]`) for circular mode.
/// The last descriptor always has `suc_eof = 1`.
pub(super) fn fill_full_chain(
    descriptors: &mut [DmaDescriptor],
    planes: &PlaneInfo,
    plane_count: usize,
    total_descs: usize,
    last_next: *mut DmaDescriptor,
) {
    let base_ptr = descriptors.as_mut_ptr();
    let mut desc_idx = 0;

    for (plane_idx, &(plane_ptr, plane_bytes)) in planes.iter().enumerate().take(plane_count) {
        let reps = 1usize << (plane_count - 1 - plane_idx);

        for _ in 0..reps {
            let mut remaining = plane_bytes;
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
}
