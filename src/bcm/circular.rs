//! Circular DMA buffer for continuous HUB75 refresh.
//!
//! `CircularBcmBuf` builds a single circular DMA descriptor chain encoding the
//! full BCM (Binary Code Modulation) repetition sequence. The DMA engine starts
//! once and loops forever. Buffer swaps are performed by applying a pointer
//! delta to every descriptor's `buffer` field — no descriptor rebuild, no DMA
//! stop/restart.

use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::dma::Preparation;
#[cfg(feature = "iram")]
use esp_hal::ram;

use crate::framebuffer::FrameBuffer;

pub(crate) struct CircularBcmBuf {
    descriptors: &'static mut [DmaDescriptor],
    desc_count: usize,
}

impl CircularBcmBuf {
    /// Build a circular descriptor chain from the given framebuffer.
    ///
    /// The chain encodes the full BCM repetition sequence (identical layout to
    /// `full-chain-dma`) with two differences:
    /// - The last descriptor's `next` points back to `desc[0]` (circular).
    /// - Only the last descriptor has `suc_eof = 1`.
    pub(crate) fn new(
        descriptors: &'static mut [DmaDescriptor],
        fb: &'static impl FrameBuffer,
    ) -> Self {
        let cache = super::segments_from_fb(fb);
        let total_descs = cache.descriptor_count();
        debug_assert!(
            descriptors.len() >= total_descs,
            "not enough DMA descriptors: have {}, need {}",
            descriptors.len(),
            total_descs,
        );

        let ring_start = descriptors.as_mut_ptr();
        super::fill_full_chain(
            &mut descriptors[..total_descs],
            &cache,
            total_descs,
            ring_start,
        );

        Self {
            descriptors,
            desc_count: total_descs,
        }
    }

    /// Raw pointer to the descriptor array (for ISR access after `send()`
    /// consumes this buffer).
    pub(crate) fn descriptors_ptr(&self) -> *mut DmaDescriptor {
        self.descriptors.as_ptr() as *mut DmaDescriptor
    }

    /// Number of active descriptors in the chain.
    pub(crate) fn desc_count(&self) -> usize {
        self.desc_count
    }
}

// SAFETY: All access is serialised by `critical_section::with`.
unsafe impl Send for CircularBcmBuf {}

unsafe impl DmaTxBuffer for CircularBcmBuf {
    type View = Self;
    type Final = Self;

    #[cfg_attr(feature = "iram", ram)]
    fn prepare(&mut self) -> Preparation {
        super::make_preparation(self.descriptors)
    }

    fn into_view(self) -> Self::View {
        self
    }

    fn from_view(view: Self::View) -> Self::Final {
        view
    }
}
