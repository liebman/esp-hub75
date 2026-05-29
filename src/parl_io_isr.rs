//! ISR-driven HUB75 driver for PARL_IO peripherals (ESP32-C5 / ESP32-C6).
//!
//! This module provides an interrupt-driven display controller that
//! continuously refreshes a HUB75 panel from a framebuffer.  The PARL_IO
//! `TxEof` interrupt drives the entire BCM (Binary Code Modulation) refresh
//! loop.
//!
//! The driver sends one bit-plane at a time (always well under the 65535-byte
//! PARL_IO transfer limit), chaining them via the ISR to produce the full BCM
//! weighting.  Buffer swaps happen atomically at frame boundaries.
//!
//! # Blocking example
//!
//! ```rust,ignore
//! let hub75 = Hub75Isr::new(
//!     peripherals.PARL_IO, pins, peripherals.DMA_CH0,
//!     tx_descriptors, Rate::from_mhz(20), &*fb,
//! ).expect("failed to create Hub75Isr");
//!
//! // Display refreshes automatically — main thread is free.
//! loop { core::hint::spin_loop(); }
//! ```
//!
//! # Async example
//!
//! ```rust,ignore
//! let hub75 = Hub75Isr::new_async(
//!     peripherals.PARL_IO, pins, peripherals.DMA_CH0,
//!     tx_descriptors, Rate::from_mhz(20), &*fb0,
//! ).expect("failed to create Hub75Isr");
//!
//! // Swap buffers without blocking — yields to the executor.
//! let old_ptr = hub75.swap(&*fb1).await;
//! ```

use core::cell::RefCell;
use core::ptr::null_mut;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::task::Waker;

use critical_section::Mutex;
use esp_hal::Blocking;
use esp_hal::dma::{
    BurstConfig, DmaChannelFor, DmaDescriptor, DmaTxBuffer, EmptyBuf, Owner, Preparation,
    TransferDirection,
};
use esp_hal::handler;
use esp_hal::parl_io::{
    BitPackOrder, ClkOutPin, ConfigurePins, ParlIo, ParlIoInterrupt, ParlIoTx, ParlIoTxTransfer,
    SampleEdge, TxConfig, TxPins,
};
use esp_hal::peripherals::PARL_IO;
#[cfg(feature = "iram")]
use esp_hal::ram;
use esp_hal::time::Rate;

use crate::framebuffer::FrameBuffer;
use crate::Hub75Error;
use crate::Hub75Pins;

const MAX_PLANES: usize = 8;

// ---------------------------------------------------------------------------
// SinglePlaneBuf — self-contained BCM DmaTxBuffer
// ---------------------------------------------------------------------------

type PlaneInfo = [(*const u8, usize); MAX_PLANES];

struct SinglePlaneBuf {
    descriptors: &'static mut [DmaDescriptor],
    planes: PlaneInfo,
    plane_count: usize,
    current_plane: usize,
    current_rep: usize,
}

impl SinglePlaneBuf {
    fn new(descriptors: &'static mut [DmaDescriptor], fb: &impl FrameBuffer) -> Self {
        let plane_count = fb.plane_count();
        assert!(
            plane_count <= MAX_PLANES,
            "plane_count {plane_count} exceeds MAX_PLANES"
        );
        let mut planes: PlaneInfo = [(null_mut() as *const u8, 0usize); MAX_PLANES];
        for (i, slot) in planes.iter_mut().enumerate().take(plane_count) {
            *slot = fb.plane_ptr_len(i);
        }
        Self {
            descriptors,
            planes,
            plane_count,
            current_plane: 0,
            current_rep: 0,
        }
    }

    /// Advance the BCM state machine after a transfer completes.
    /// Returns `true` when a full BCM frame boundary is reached.
    #[cfg_attr(feature = "iram", ram)]
    fn advance(&mut self) -> bool {
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

    /// Replace the stored plane pointers (called at swap).
    #[cfg_attr(feature = "iram", ram)]
    fn update_planes(&mut self, new_planes: PlaneInfo) {
        self.planes = new_planes;
    }

    /// Returns the byte length for the current plane transfer.
    #[allow(dead_code)]
    #[cfg_attr(feature = "iram", ram)]
    fn current_transfer_len(&self) -> usize {
        self.planes[self.current_plane].1
    }
}

// SAFETY: SinglePlaneBuf is only accessed under critical sections or from
// within the ISR on single-core ESP32-C5/C6.  There is no concurrent access.
unsafe impl Send for SinglePlaneBuf {}

unsafe impl DmaTxBuffer for SinglePlaneBuf {
    type View = Self;
    type Final = Self;

    #[cfg_attr(feature = "iram", ram)]
    fn prepare(&mut self) -> Preparation {
        let (ptr, len) = self.planes[self.current_plane];
        let mut remaining = len;
        let mut offset = 0;
        let mut desc_idx = 0;

        while remaining > 0 {
            let chunk = remaining.min(4095);
            let desc = &mut self.descriptors[desc_idx];
            desc.buffer = unsafe { ptr.add(offset) as *mut u8 };
            desc.set_size(chunk);
            desc.set_length(chunk);
            desc.set_owner(Owner::Dma);
            desc.set_suc_eof(false);
            desc.next = null_mut();
            remaining -= chunk;
            offset += chunk;
            desc_idx += 1;
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

// ---------------------------------------------------------------------------
// ISR shared state
// ---------------------------------------------------------------------------

#[allow(dead_code)]
enum TransferPhase {
    InFlight(ParlIoTxTransfer<'static, SinglePlaneBuf, Blocking>),
    Idle(ParlIoTx<'static, Blocking>, SinglePlaneBuf),
    Transitioning,
}

struct IsrState {
    transfer: TransferPhase,

    current_fb_ptr: *const (),

    pending_planes: Option<PlaneInfo>,
    pending_fb_ptr: *const (),

    returned_fb_ptr: *const (),
}

// SAFETY: All access is serialised by `critical_section` on single-core C5/C6.
unsafe impl Send for IsrState {}

static ISR_STATE: Mutex<RefCell<Option<IsrState>>> = Mutex::new(RefCell::new(None));
static SWAP_DONE: AtomicBool = AtomicBool::new(false);
static SWAP_WAKER: Mutex<RefCell<Option<Waker>>> = Mutex::new(RefCell::new(None));
static FRAME_COUNT: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------
// TxEof interrupt handler
// ---------------------------------------------------------------------------

#[handler]
#[cfg_attr(feature = "iram", ram)]
fn hub75_isr() {
    critical_section::with(|cs| {
        let mut borrow = ISR_STATE.borrow_ref_mut(cs);
        let state = match borrow.as_mut() {
            Some(s) => s,
            None => return,
        };

        let xfer = match core::mem::replace(&mut state.transfer, TransferPhase::Transitioning) {
            TransferPhase::InFlight(xfer) => xfer,
            _ => return,
        };

        // .wait() returns instantly — TxEof already fired.
        let (_result, parl_io_tx, mut buf) = xfer.wait();

        let frame_boundary = buf.advance();

        if frame_boundary {
            FRAME_COUNT.fetch_add(1, Ordering::Relaxed);

            if let Some(pending) = state.pending_planes.take() {
                state.returned_fb_ptr = state.current_fb_ptr;
                state.current_fb_ptr = state.pending_fb_ptr;
                state.pending_fb_ptr = core::ptr::null();
                buf.update_planes(pending);
                SWAP_DONE.store(true, Ordering::Release);

                if let Some(waker) = SWAP_WAKER.borrow_ref_mut(cs).take() {
                    waker.wake();
                }
            }
        }

        #[cfg(feature = "esp32c5")]
        let transfer_len = 0;
        #[cfg(not(feature = "esp32c5"))]
        let transfer_len = buf.current_transfer_len();

        match parl_io_tx.write(transfer_len, buf) {
            Ok(new_xfer) => {
                state.transfer = TransferPhase::InFlight(new_xfer);
            }
            Err((_err, parl_io_tx, buf)) => {
                state.transfer = TransferPhase::Idle(parl_io_tx, buf);
            }
        }
    });
}

// ---------------------------------------------------------------------------
// Public driver handle
// ---------------------------------------------------------------------------

/// ISR-driven HUB75 display controller.
///
/// Once created, the driver continuously refreshes the panel from the current
/// framebuffer using the PARL_IO `TxEof` interrupt.
///
/// The `DM` type parameter selects the swap API:
/// - [`Hub75Isr<Blocking>`](esp_hal::Blocking): [`swap()`](Hub75Isr::swap)
///   spin-loops until the next frame boundary.
/// - [`Hub75Isr<Async>`](esp_hal::Async): [`swap()`](Hub75Isr::swap) is an
///   async method that yields to the executor.
pub struct Hub75Isr<DM: esp_hal::DriverMode> {
    _dm: core::marker::PhantomData<DM>,
}

impl<DM: esp_hal::DriverMode> Hub75Isr<DM> {
    fn new_internal<T: TxPins + ConfigurePins + 'static>(
        parl_io: PARL_IO<'static>,
        hub75_pins: impl Hub75Pins<'static, T>,
        channel: impl DmaChannelFor<PARL_IO<'static>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
        initial_fb: &'static (impl FrameBuffer + 'static),
    ) -> Result<Self, Hub75Error> {
        let (pins, clock_pin) = hub75_pins.convert_pins();

        let mut parl_io_dev = ParlIo::new(parl_io, channel)?;

        parl_io_dev.set_interrupt_handler(hub75_isr);
        parl_io_dev.listen(ParlIoInterrupt::TxEof);

        #[cfg(feature = "invert-clock")]
        let sample_edge = SampleEdge::Normal;
        #[cfg(not(feature = "invert-clock"))]
        let sample_edge = SampleEdge::Invert;

        let config = TxConfig::default()
            .with_frequency(frequency)
            .with_idle_value(0)
            .with_sample_edge(sample_edge)
            .with_bit_order(BitPackOrder::Msb);

        let clk_pin = ClkOutPin::new(clock_pin);
        let parl_io_tx = parl_io_dev.tx.with_config(pins, clk_pin, config)?;

        #[cfg(feature = "esp32c5")]
        unsafe {
            use esp32c5 as pac;
            let pio = pac::PARL_IO::steal();
            pio.tx_genrl_cfg()
                .modify(|_, w| w.tx_eof_gen_sel().set_bit());
        }

        let buf = SinglePlaneBuf::new(tx_descriptors, initial_fb);

        #[cfg(feature = "esp32c5")]
        let transfer_len = 0;
        #[cfg(not(feature = "esp32c5"))]
        let transfer_len = buf.current_transfer_len();

        let xfer = parl_io_tx
            .write(transfer_len, buf)
            .map_err(|(e, _tx, _buf)| Hub75Error::from(e))?;

        critical_section::with(|cs| {
            *ISR_STATE.borrow_ref_mut(cs) = Some(IsrState {
                transfer: TransferPhase::InFlight(xfer),
                current_fb_ptr: initial_fb as *const _ as *const (),
                pending_planes: None,
                pending_fb_ptr: core::ptr::null(),
                returned_fb_ptr: core::ptr::null(),
            });
        });

        Ok(Self {
            _dm: core::marker::PhantomData,
        })
    }

    /// Returns the number of complete BCM frames rendered since driver
    /// creation.
    pub fn frame_count(&self) -> u32 {
        FRAME_COUNT.load(Ordering::Relaxed)
    }
}

fn planes_from_fb(fb: &impl FrameBuffer) -> PlaneInfo {
    let plane_count = fb.plane_count();
    assert!(
        plane_count <= MAX_PLANES,
        "plane_count {plane_count} exceeds MAX_PLANES"
    );
    let mut planes: PlaneInfo = [(null_mut() as *const u8, 0usize); MAX_PLANES];
    for (i, slot) in planes.iter_mut().enumerate().take(plane_count) {
        *slot = fb.plane_ptr_len(i);
    }
    planes
}

impl Hub75Isr<Blocking> {
    /// Create a new blocking ISR-driven HUB75 driver and start refreshing
    /// immediately.
    ///
    /// # Arguments
    /// * `parl_io` -- The PARL_IO peripheral instance
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `frequency` -- PARL_IO clock rate
    /// * `initial_fb` -- The first framebuffer to display
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new<T: TxPins + ConfigurePins + 'static>(
        parl_io: PARL_IO<'static>,
        hub75_pins: impl Hub75Pins<'static, T>,
        channel: impl DmaChannelFor<PARL_IO<'static>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
        initial_fb: &'static (impl FrameBuffer + 'static),
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(
            parl_io,
            hub75_pins,
            channel,
            tx_descriptors,
            frequency,
            initial_fb,
        )
    }

    /// Replace the displayed framebuffer.
    ///
    /// Blocks (spin-loops) until the current BCM frame completes, then
    /// atomically swaps in `new_fb` and returns the old framebuffer pointer.
    ///
    /// # Safety contract
    ///
    /// The caller **must** ensure that every call to `swap` uses the same
    /// concrete framebuffer type that was passed to [`Hub75Isr::new`].  The
    /// returned raw pointer must be cast back to that same type.
    pub fn swap(&self, new_fb: &'static impl FrameBuffer) -> *const () {
        let new_planes = planes_from_fb(new_fb);

        critical_section::with(|cs| {
            let mut borrow = ISR_STATE.borrow_ref_mut(cs);
            let state = borrow.as_mut().expect("Hub75Isr not initialised");
            state.pending_planes = Some(new_planes);
            state.pending_fb_ptr = new_fb as *const _ as *const ();
            SWAP_DONE.store(false, Ordering::Relaxed);
        });

        while !SWAP_DONE.load(Ordering::Acquire) {
            core::hint::spin_loop();
        }

        critical_section::with(|cs| {
            let borrow = ISR_STATE.borrow_ref(cs);
            let state = borrow.as_ref().expect("Hub75Isr not initialised");
            state.returned_fb_ptr
        })
    }
}

impl Hub75Isr<esp_hal::Async> {
    /// Create a new async ISR-driven HUB75 driver and start refreshing
    /// immediately.
    ///
    /// # Arguments
    /// * `parl_io` -- The PARL_IO peripheral instance
    /// * `hub75_pins` -- HUB75 pin configuration (8- or 16-bit)
    /// * `channel` -- DMA channel
    /// * `tx_descriptors` -- DMA descriptor storage (use
    ///   [`hub75_dma_descriptors!`])
    /// * `frequency` -- PARL_IO clock rate
    /// * `initial_fb` -- The first framebuffer to display
    ///
    /// [`hub75_dma_descriptors!`]: crate::hub75_dma_descriptors
    pub fn new_async<T: TxPins + ConfigurePins + 'static>(
        parl_io: PARL_IO<'static>,
        hub75_pins: impl Hub75Pins<'static, T>,
        channel: impl DmaChannelFor<PARL_IO<'static>>,
        tx_descriptors: &'static mut [DmaDescriptor],
        frequency: Rate,
        initial_fb: &'static (impl FrameBuffer + 'static),
    ) -> Result<Self, Hub75Error> {
        Self::new_internal(
            parl_io,
            hub75_pins,
            channel,
            tx_descriptors,
            frequency,
            initial_fb,
        )
    }

    /// Replace the displayed framebuffer.
    ///
    /// Yields to the executor until the current BCM frame completes, then
    /// atomically swaps in `new_fb` and returns the old framebuffer pointer.
    ///
    /// # Safety contract
    ///
    /// The caller **must** ensure that every call to `swap` uses the same
    /// concrete framebuffer type that was passed to [`Hub75Isr::new_async`].
    /// The returned raw pointer must be cast back to that same type.
    pub async fn swap(&self, new_fb: &'static impl FrameBuffer) -> *const () {
        let new_planes = planes_from_fb(new_fb);

        critical_section::with(|cs| {
            let mut borrow = ISR_STATE.borrow_ref_mut(cs);
            let state = borrow.as_mut().expect("Hub75Isr not initialised");
            state.pending_planes = Some(new_planes);
            state.pending_fb_ptr = new_fb as *const _ as *const ();
            SWAP_DONE.store(false, Ordering::Relaxed);
        });

        core::future::poll_fn(|cx| {
            if SWAP_DONE.load(Ordering::Acquire) {
                return core::task::Poll::Ready(());
            }
            critical_section::with(|cs| {
                if SWAP_DONE.load(Ordering::Relaxed) {
                    return core::task::Poll::Ready(());
                }
                *SWAP_WAKER.borrow_ref_mut(cs) = Some(cx.waker().clone());
                core::task::Poll::Pending
            })
        })
        .await;

        critical_section::with(|cs| {
            let borrow = ISR_STATE.borrow_ref(cs);
            let state = borrow.as_ref().expect("Hub75Isr not initialised");
            state.returned_fb_ptr
        })
    }
}
