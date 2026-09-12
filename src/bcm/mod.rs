//! BCM (Binary Code Modulation) DMA buffer infrastructure.
//!
//! Common helpers shared by the linear and full-chain buffer implementations:
//! the descriptor-chain builder every mode uses ([`fill_descriptor_chain`]),
//! the `Preparation` construction ([`make_preparation`]), and the
//! internal-RAM validation ([`validate_fb_internal_ram`]).
//!
//! The mode-specific buffers live in their own submodules: `bcm::full_chain`
//! (the terminating `full-chain-dma` chain and the `circular-dma` ring, which
//! are the same built-once chain with only a terminating / free-running
//! difference) and `bcm::linear` (the default group-based chain, which owns the
//! segment cache).

use esp_hal::dma::BurstConfig;
use esp_hal::dma::DmaDescriptor;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::dma::EmptyBuf;
use esp_hal::dma::Owner;
use esp_hal::dma::Preparation;
#[cfg(feature = "iram")]
use esp_hal::ram;

use crate::MAX_DMA_CHUNK_SIZE;
use crate::framebuffer::BcmSegment;
use crate::framebuffer::FrameBuffer;

#[cfg(feature = "full-chain-dma")]
pub(crate) mod full_chain;
#[cfg(not(feature = "full-chain-dma"))]
pub(crate) mod linear;

/// Assert that a framebuffer and every BCM segment it exposes reside in
/// internal DRAM, not PSRAM.
///
/// PSRAM requires an explicit cache writeback before DMA reads, which the
/// custom `BcmBuf` paths never perform. The DMA reads
/// the *segment* buffers, not the `FrameBuffer` object itself, so each segment
/// pointer is checked too. This runs once per constructor, so it is a hard
/// `assert!` (not `debug_assert!`): a PSRAM framebuffer must fail loudly here
/// rather than corrupt the display silently in release builds.
pub(crate) fn validate_fb_internal_ram(fb: &impl FrameBuffer) {
    fn assert_in_dram(dram: &core::ops::Range<usize>, ptr: *const (), what: &str) {
        let addr = ptr as usize;
        assert!(
            dram.contains(&addr),
            "{what} at {addr:#010X} is not in internal DRAM ({dram:#010X?}); \
             PSRAM is not supported for DMA framebuffers"
        );
    }

    let dram = esp_metadata_generated::memory_range!("DRAM");
    assert_in_dram(&dram, core::ptr::from_ref(fb).cast::<()>(), "framebuffer");
    for i in 0..fb.bcm_segment_count() {
        let seg = fb.bcm_segment(i);
        assert_in_dram(&dram, seg.ptr.cast::<()>(), "BCM segment");
    }
}

/// Build a `Preparation` pointing to the first descriptor in a chain.
///
/// Shared by the linear and full-chain buffer implementations.
#[cfg_attr(feature = "iram", ram)]
pub(super) fn make_preparation(descriptors: &mut [DmaDescriptor]) -> Preparation {
    // `EmptyBuf` provides a `Preparation` with safe defaults; we override
    // the fields relevant to our descriptor chain. If `Preparation` gains
    // new fields in a future esp-hal release, review them here.
    //
    // `check_owner = Some(false)` and `auto_write_back = false` below are
    // load-bearing for the circular ring: the DMA must neither require the
    // `owner` bit before consuming a descriptor nor clear it afterwards,
    // otherwise the free-running chain stalls after one lap.
    let mut empty = EmptyBuf;
    let mut prep: Preparation = empty.prepare();
    prep.start = descriptors.as_mut_ptr();
    prep.burst_transfer = BurstConfig::default();
    prep.check_owner = Some(false);
    prep.auto_write_back = false;
    prep
}

/// Fill a BCM descriptor chain from a segment source.
///
/// Every refresh mode turns a sequence of [`BcmSegment`]s into DMA
/// descriptors through this function, chunking each segment repetition down
/// to [`MAX_DMA_CHUNK_SIZE`]:
///
/// - **Full-chain**: the whole frame, `last_next = null_mut()` and
///   `last_suc_eof = true` (the transfer must end so the ISR can restart it).
/// - **Circular**: the whole ring, `last_next = ring_start` (wraps back to
///   `desc[0]`) and `last_suc_eof = false` (a `suc_eof` in the free-running
///   ring would halt or signal spuriously — the boundary detector arms it
///   later).
/// - **Linear, group-based**: a single group, `last_next = null_mut()` and
///   `last_suc_eof = true`.
///
/// Segments are pulled on demand via `get_segment(idx)` for
/// `idx in 0..segment_count`, so callers can stream straight from a
/// framebuffer without materialising a segment cache.
#[cfg_attr(feature = "iram", ram)]
pub(super) fn fill_descriptor_chain(
    descriptors: &mut [DmaDescriptor],
    segment_count: usize,
    get_segment: impl Fn(usize) -> BcmSegment,
    total_descs: usize,
    last_next: *mut DmaDescriptor,
    last_suc_eof: bool,
) {
    let base_ptr = descriptors.as_mut_ptr();
    let mut desc_idx = 0;

    for seg_idx in 0..segment_count {
        let seg = get_segment(seg_idx);

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
                desc.buffer = unsafe { seg.ptr.add(offset).cast_mut() };
                desc.set_size(chunk);
                desc.set_length(chunk);
                desc.set_owner(Owner::Dma);
                desc.set_suc_eof(is_last & last_suc_eof);
                desc.next = next;
                remaining -= chunk;
                offset += chunk;
                desc_idx += 1;
            }
        }
    }
}
