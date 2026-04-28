use core::ptr::null_mut;

use esp_hal::dma::BurstConfig;
use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::dma::EmptyBuf;
use esp_hal::dma::Owner;
use esp_hal::dma::Preparation;
use esp_hal::dma::TransferDirection;

const MAX_PLANES: usize = 8;

/// Internal DMA transmit buffer used by [`Hub75::render()`].
///
/// This type is an implementation detail. Users should not need to interact
/// with it directly — use the [`hub75_dma_descriptors!`] macro to allocate
/// DMA descriptors and [`Hub75Transfer::wait()`] to recover resources.
///
/// [`Hub75::render()`]: crate::Hub75::render
/// [`Hub75Transfer::wait()`]: crate::Hub75Transfer::wait
/// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
pub struct BcmTxDmaBuf {
    descriptors: &'static mut [DmaDescriptor],
    planes: [(*const u8, usize); MAX_PLANES],
    plane_count: usize,
}

impl BcmTxDmaBuf {
    /// Creates a new `BcmTxDmaBuf` from a descriptor slice and the plane
    /// information extracted from a `FrameBuffer`.
    ///
    /// # Panics
    ///
    /// Panics if `plane_count` exceeds `MAX_PLANES` or the descriptor pool is
    /// too small for the computed chain.
    pub(crate) fn new(
        descriptors: &'static mut [DmaDescriptor],
        fb: &impl crate::framebuffer::FrameBuffer,
    ) -> Self {
        let pc = fb.plane_count();
        assert!(pc <= MAX_PLANES, "plane_count {pc} exceeds MAX_PLANES");

        let mut planes = [(null_mut() as *const u8, 0usize); MAX_PLANES];
        for (i, slot) in planes.iter_mut().enumerate().take(pc) {
            *slot = fb.plane_ptr_len(i);
        }

        let required = Self::descriptor_count_for(pc, planes[0].1);
        assert!(
            descriptors.len() >= required,
            "need {required} descriptors, got {}",
            descriptors.len()
        );

        Self {
            descriptors,
            planes,
            plane_count: pc,
        }
    }

    /// Returns the total number of bytes that will be transferred, accounting
    /// for BCM repetitions.
    #[allow(dead_code)]
    pub(crate) fn transfer_len(&self) -> usize {
        let plane_bytes = self.planes[0].1;
        let total_reps = (1usize << self.plane_count) - 1;
        plane_bytes * total_reps
    }

    /// Recovers the descriptor storage after a transfer completes.
    pub(crate) fn split(self) -> &'static mut [DmaDescriptor] {
        self.descriptors
    }

    /// Computes the number of DMA descriptors needed for the given BCM
    /// configuration. Delegates to [`crate::dma_descriptor_count`].
    #[must_use]
    pub(crate) const fn descriptor_count_for(
        bcm_chunk_count: usize,
        bcm_chunk_bytes: usize,
    ) -> usize {
        crate::dma_descriptor_count(bcm_chunk_count, bcm_chunk_bytes)
    }
}

unsafe impl DmaTxBuffer for BcmTxDmaBuf {
    type View = Self;
    type Final = Self;

    fn prepare(&mut self) -> Preparation {
        let mut desc_idx = 0;

        for plane_idx in 0..self.plane_count {
            let reps = 1usize << (self.plane_count - 1 - plane_idx);
            let (plane_ptr, plane_bytes) = self.planes[plane_idx];

            for _ in 0..reps {
                let mut remaining = plane_bytes;
                let mut offset = 0;
                while remaining > 0 {
                    let chunk = remaining.min(4095);
                    let desc = &mut self.descriptors[desc_idx];
                    desc.buffer = unsafe { plane_ptr.add(offset) as *mut u8 };
                    desc.set_size(chunk);
                    desc.set_length(chunk);
                    desc.set_owner(Owner::Dma);
                    desc.set_suc_eof(false);
                    desc.next = null_mut();
                    remaining -= chunk;
                    offset += chunk;
                    desc_idx += 1;
                }
            }
        }

        for i in 0..desc_idx {
            let is_last = i + 1 == desc_idx;
            let next = if is_last {
                null_mut()
            } else {
                unsafe { self.descriptors.as_mut_ptr().add(i + 1) }
            };
            let desc = &mut self.descriptors[i];
            desc.set_owner(Owner::Dma);
            desc.set_suc_eof(is_last);
            desc.next = next;
        }

        let mut seed = EmptyBuf;
        let mut prep: Preparation = seed.prepare();
        prep.start = self.descriptors.as_mut_ptr();
        prep.direction = TransferDirection::Out;
        prep.burst_transfer = BurstConfig::default();
        prep.check_owner = Some(false);
        prep.auto_write_back = false;
        prep
    }

    fn into_view(self) -> Self::View {
        self
    }

    fn from_view(view: Self::View) -> Self::Final {
        view
    }
}
