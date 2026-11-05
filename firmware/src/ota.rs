//! OTA firmware-update receiver.
//!
//! Integrates into the main RTIC loop: incoming mesh messages with an OTA
//! type prefix are routed to [`OtaReceiver::handle_message`], which drives a
//! state machine that buffers chunks and writes flash pages.  Once the whole
//! image is in the DFU slot and its CRC-32 matches the offer, it asks the
//! bootloader for a swap and replies `OTA_COMPLETE`; the main loop then
//! reboots the node when [`OtaReceiver::reboot_due`] says the reply has had
//! time to go out.
//!
//! Offers are refused (`BOOT_PENDING`) unless the boot log is settled: not
//! while a swap waits for that reboot, and not while the running image is
//! itself on trial, whose revert target the new image would overwrite.

use core::cell::UnsafeCell;

use flash_layout::crc32::Crc32;
use stm32wlxx_hal::flash::{AlignedAddr, Flash, Page};
use stm32wlxx_hal::pac;

use crate::boot_state::BootState;
use crate::config::{FIRMWARE_VERSION, TX_POLL_TIMEOUT_MS};
use crate::flash_ecc;
use crate::ota_protocol::{self, OtaChunk, OtaChunkAck, OtaOffer, msg, reason};

// -- Flash / partition constants ---------------------------------------------

use flash_layout::{DFU_BASE, MAX_FW_SIZE, PAGE_SIZE};

/// First DFU page index, in the HAL's u8 page numbering.
const DFU_PAGE_START: u8 = flash_layout::DFU_PAGE as u8;

/// Silence after which an in-progress transfer is abandoned (ms).
///
/// Sized well above the gap between chunks at the slowest preset, where a
/// single packet is ~6.6 s on the air.
const TRANSFER_IDLE_TIMEOUT_MS: u32 = 60_000;

/// How long after `OTA_COMPLETE` the node reboots into the new image: time
/// for the reply to go out, and for a sender that missed it to retry the
/// last chunk and hear it again, at the slowest preset too.
const REBOOT_DELAY_MS: u32 = 10_000 + 2 * TX_POLL_TIMEOUT_MS as u32;

// -- Page buffer (static to avoid stack overflow) ----------------------------

const PAGE_BUF_SIZE: usize = PAGE_SIZE as usize;

struct StaticPageBuf(UnsafeCell<[u8; PAGE_BUF_SIZE]>);
unsafe impl Sync for StaticPageBuf {}
static PAGE_BUF: StaticPageBuf = StaticPageBuf(UnsafeCell::new([0xFF; PAGE_BUF_SIZE]));

// -- OTA state ---------------------------------------------------------------

/// Receiving-state metadata.
struct Receiving {
    offer: OtaOffer,
    /// Next expected chunk index.
    next_chunk: u16,
    /// Current DFU page being filled (0-based index into DFU partition).
    current_page: u16,
    /// Byte offset within the page buffer.
    page_offset: u16,
    /// `platform::millis()` of the last frame accepted for this transfer.
    last_ms: u32,
}

enum State {
    Idle,
    Active(Receiving),
    /// The image is verified and the swap requested; waiting to reboot.
    Done {
        crc: u32,
        /// `platform::millis()` when the transfer completed.
        at_ms: u32,
    },
}

/// OTA firmware-update receiver.
pub struct OtaReceiver {
    state: State,
}

/// Response to send back to the OTA initiator.
pub struct OtaResponse {
    pub data: [u8; 32],
    pub len: usize,
}

impl Default for OtaReceiver {
    fn default() -> Self {
        Self::new()
    }
}

impl OtaReceiver {
    pub fn new() -> Self {
        Self { state: State::Idle }
    }

    fn page_buf(&self) -> &[u8; PAGE_BUF_SIZE] {
        unsafe { &*PAGE_BUF.0.get() }
    }

    fn page_buf_mut(&mut self) -> &mut [u8; PAGE_BUF_SIZE] {
        unsafe { &mut *PAGE_BUF.0.get() }
    }

    /// Process an incoming OTA mesh message.
    ///
    /// Returns `Some(OtaResponse)` if a reply should be sent back to the
    /// source node, or `None` if no reply is needed.
    pub fn handle_message(
        &mut self,
        msg_data: &[u8],
        flash: &mut pac::FLASH,
        boot: &mut BootState,
    ) -> Option<OtaResponse> {
        if msg_data.is_empty() {
            return None;
        }

        let msg_type = msg_data[0];
        let payload = &msg_data[1..];

        match msg_type {
            msg::OTA_OFFER => self.handle_offer(payload, boot),
            msg::OTA_CHUNK => self.handle_chunk(payload, flash, boot),
            // After completion the swap is requested and the reboot follows;
            // an abort cannot take that back.
            msg::OTA_ABORT if !matches!(self.state, State::Done { .. }) => {
                self.reset();
                None
            }
            _ => None,
        }
    }

    /// Handle an OTA_OFFER message.
    fn handle_offer(&mut self, payload: &[u8], boot: &BootState) -> Option<OtaResponse> {
        match self.state {
            State::Active(_) => return Some(make_reject(reason::BUSY)),
            State::Done { .. } => return Some(make_reject(reason::BOOT_PENDING)),
            State::Idle if !boot.settled() => return Some(make_reject(reason::BOOT_PENDING)),
            State::Idle => {}
        }

        let offer = OtaOffer::deserialize(payload)?;

        if offer.firmware_size > MAX_FW_SIZE || offer.firmware_size == 0 {
            return Some(make_reject(reason::NO_SPACE));
        }

        // Reject downgrades or same-version re-flashes.
        if offer.version <= FIRMWARE_VERSION {
            return Some(make_reject(reason::BAD_VERSION));
        }

        self.page_buf_mut().fill(0xFF);
        self.state = State::Active(Receiving {
            offer,
            next_chunk: 0,
            current_page: 0,
            page_offset: 0,
            last_ms: crate::platform::millis(),
        });

        let mut buf = [0u8; 32];
        let len = ota_protocol::serialize_accept(&mut buf);
        Some(OtaResponse { data: buf, len })
    }

    /// Handle an OTA_CHUNK message.
    fn handle_chunk(
        &mut self,
        payload: &[u8],
        flash: &mut pac::FLASH,
        boot: &mut BootState,
    ) -> Option<OtaResponse> {
        // Extract current state, or bail if idle.
        let (next_chunk, _total_chunks) = match &self.state {
            State::Active(rx) => (rx.next_chunk, rx.offer.total_chunks),
            State::Idle => return None,
            // A sender that missed OTA_COMPLETE retries its last chunk.
            State::Done { crc, .. } => return Some(make_complete(*crc)),
        };

        let chunk = OtaChunk::deserialize(payload)?;

        // Stamped before the duplicate and out-of-order returns below: a
        // sender that is retrying is still there, and only silence should
        // let [`OtaReceiver::expire`] drop the transfer.
        if let State::Active(rx) = &mut self.state {
            rx.last_ms = crate::platform::millis();
        }

        // Duplicate or old chunk - re-ACK.
        if chunk.index < next_chunk {
            return Some(make_ack(chunk.index, next_chunk));
        }

        // Out-of-order - request what we actually need.
        if chunk.index > next_chunk {
            return Some(make_ack(chunk.index, next_chunk));
        }

        // Correct next chunk - copy data into page buffer.
        let data_len = chunk.data_len as usize;

        let rx = match &mut self.state {
            State::Active(rx) => rx,
            _ => unreachable!(),
        };

        let offset = rx.page_offset as usize;
        let space = PAGE_BUF_SIZE - offset;
        let to_copy = data_len.min(space);

        self.page_buf_mut()[offset..offset + to_copy].copy_from_slice(&chunk.data[..to_copy]);

        // Update state (need to re-borrow after page_buf write)
        let rx = match &mut self.state {
            State::Active(rx) => rx,
            _ => unreachable!(),
        };
        rx.page_offset += to_copy as u16;
        rx.next_chunk = chunk.index + 1;

        // If page buffer is full, flush to flash.
        if rx.page_offset >= PAGE_SIZE as u16 {
            let page_idx = DFU_PAGE_START + rx.current_page as u8;
            if !self.write_page_to_flash(flash, page_idx) {
                self.reset();
                return Some(make_abort(reason::FLASH_ERROR));
            }

            let rx = match &mut self.state {
                State::Active(rx) => rx,
                _ => unreachable!(),
            };
            rx.current_page += 1;
            rx.page_offset = 0;
            self.page_buf_mut().fill(0xFF);

            // Copy remainder if chunk data spanned a page boundary.
            if to_copy < data_len {
                let remainder = data_len - to_copy;
                self.page_buf_mut()[..remainder].copy_from_slice(&chunk.data[to_copy..data_len]);
                let rx = match &mut self.state {
                    State::Active(rx) => rx,
                    _ => unreachable!(),
                };
                rx.page_offset = remainder as u16;
            }
        }

        // Check if transfer is complete.
        let (next, total, fw_size, expected_crc, page_offset, current_page) = match &self.state {
            State::Active(rx) => (
                rx.next_chunk,
                rx.offer.total_chunks,
                rx.offer.firmware_size,
                rx.offer.crc32,
                rx.page_offset,
                rx.current_page,
            ),
            _ => unreachable!(),
        };

        if next >= total {
            // Flush any remaining partial page.
            if page_offset > 0 {
                let page_idx = DFU_PAGE_START + current_page as u8;
                if !self.write_page_to_flash(flash, page_idx) {
                    self.reset();
                    return Some(make_abort(reason::FLASH_ERROR));
                }
            }

            // Verify CRC32.  The bootloader checks it again before swapping.
            if dfu_crc32(fw_size) != Some(expected_crc) {
                self.reset();
                return Some(make_abort(reason::CRC_MISMATCH));
            }
            if boot.request_swap(flash, fw_size, expected_crc).is_err() {
                self.reset();
                return Some(make_abort(reason::FLASH_ERROR));
            }
            self.state = State::Done {
                crc: expected_crc,
                at_ms: crate::platform::millis(),
            };
            return Some(make_complete(expected_crc));
        }

        let next_chunk = match &self.state {
            State::Active(rx) => rx.next_chunk,
            _ => unreachable!(),
        };
        Some(make_ack(chunk.index, next_chunk))
    }

    /// Write the page buffer to flash at the given page index.
    fn write_page_to_flash(&self, flash_periph: &mut pac::FLASH, page_idx: u8) -> bool {
        let mut flash = Flash::unlock(flash_periph);
        let page = unsafe { Page::from_index_unchecked(page_idx) };

        // Erase page
        if unsafe { flash.page_erase(page) }.is_err() {
            return false;
        }

        // Program page contents
        let addr = unsafe { AlignedAddr::new_unchecked(page.addr()) };
        if unsafe { flash.program_bytes(self.page_buf(), addr) }.is_err() {
            return false;
        }

        true
        // Flash locked on drop
    }

    /// Return current progress as (received_chunks, total_chunks), or None if idle.
    pub fn progress(&self) -> Option<(u16, u16)> {
        match &self.state {
            State::Active(rx) => Some((rx.next_chunk, rx.offer.total_chunks)),
            State::Idle | State::Done { .. } => None,
        }
    }

    /// A verified image is staged and its `OTA_COMPLETE` has had time to go
    /// out: reboot, and the bootloader swaps it in.
    pub fn reboot_due(&self, now_ms: u32) -> bool {
        match self.state {
            State::Done { at_ms, .. } => now_ms.wrapping_sub(at_ms) >= REBOOT_DELAY_MS,
            _ => false,
        }
    }

    /// Whether the receiver is currently in a transfer.
    pub fn is_active(&self) -> bool {
        matches!(self.state, State::Active(_))
    }

    /// Drop a transfer that has gone silent, returning `true` if one was
    /// dropped.
    ///
    /// An active transfer makes the node answer every new [`msg::OTA_OFFER`]
    /// with [`reason::BUSY`], and only an explicit [`msg::OTA_ABORT`] cleared
    /// it.  A sender that *crashes* mid-upload never sends one, so the node
    /// stayed unupdatable until it was power-cycled.  The window is far
    /// longer than any legitimate gap between chunks, which at the slowest
    /// preset are several seconds apart.
    pub fn expire(&mut self) -> bool {
        let last_ms = match &self.state {
            State::Active(rx) => rx.last_ms,
            State::Idle | State::Done { .. } => return false,
        };
        if crate::platform::millis().wrapping_sub(last_ms) > TRANSFER_IDLE_TIMEOUT_MS {
            self.reset();
            true
        } else {
            false
        }
    }

    fn reset(&mut self) {
        self.state = State::Idle;
    }
}

// -- Free functions ----------------------------------------------------------

/// CRC-32 of the first `size` bytes of the DFU slot, or `None` when the
/// flash's ECC rejected part of them.
fn dfu_crc32(size: u32) -> Option<u32> {
    let mut crc = Crc32::new();
    let mut addr = DFU_BASE;
    let mut left = size as usize;
    while left > 0 {
        let dw = flash_ecc::read_dw(addr).ok()?;
        let n = left.min(8);
        crc.update(&dw.to_le_bytes()[..n]);
        left -= n;
        addr += 8;
    }
    Some(crc.finish())
}

fn make_complete(crc: u32) -> OtaResponse {
    let mut buf = [0u8; 32];
    let len = ota_protocol::serialize_complete(&mut buf, crc);
    OtaResponse { data: buf, len }
}

fn make_ack(chunk_index: u16, next_needed: u16) -> OtaResponse {
    let ack = OtaChunkAck {
        chunk_index,
        next_needed,
    };
    let mut buf = [0u8; 32];
    let len = ack.serialize(&mut buf);
    OtaResponse { data: buf, len }
}

fn make_reject(reason: u8) -> OtaResponse {
    let mut buf = [0u8; 32];
    let len = ota_protocol::serialize_reject(&mut buf, reason);
    OtaResponse { data: buf, len }
}

fn make_abort(reason: u8) -> OtaResponse {
    let mut buf = [0u8; 32];
    let len = ota_protocol::serialize_abort(&mut buf, reason);
    OtaResponse { data: buf, len }
}
