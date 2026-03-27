//! OTA firmware-update receiver.
//!
//! Integrates into the main RTIC loop: incoming mesh messages with an OTA
//! type prefix are routed to [`OtaReceiver::handle_message`], which drives a
//! state machine that buffers chunks, writes flash pages, and eventually
//! triggers a reboot into the new firmware.

use core::cell::UnsafeCell;
use core::ptr;

use stm32wlxx_hal::flash::{AlignedAddr, Flash, Page};
use stm32wlxx_hal::pac;

use crate::config::FIRMWARE_VERSION;
use crate::ota_protocol::{self, msg, reason, OtaChunk, OtaChunkAck, OtaOffer};

// ── Flash / partition constants ─────────────────────────────────────────────

const FLASH_BASE: u32 = 0x0800_0000;
const PAGE_SIZE: u32 = 2048;

/// DFU staging partition: pages 64–119 (56 data pages, matching ACTIVE size).
const DFU_BASE: u32 = FLASH_BASE + 64 * PAGE_SIZE; // 0x0802_0000

/// First DFU page index.
const DFU_PAGE_START: u8 = 64;

/// Maximum firmware size (ACTIVE partition = 112 KB = 56 pages).
const MAX_FW_SIZE: u32 = 56 * PAGE_SIZE;

// ── Page buffer (static to avoid stack overflow) ────────────────────────────

const PAGE_BUF_SIZE: usize = PAGE_SIZE as usize;

struct StaticPageBuf(UnsafeCell<[u8; PAGE_BUF_SIZE]>);
unsafe impl Sync for StaticPageBuf {}
static PAGE_BUF: StaticPageBuf = StaticPageBuf(UnsafeCell::new([0xFF; PAGE_BUF_SIZE]));

// ── OTA state ───────────────────────────────────────────────────────────────

/// Receiving-state metadata.
struct Receiving {
    offer: OtaOffer,
    /// Next expected chunk index.
    next_chunk: u16,
    /// Current DFU page being filled (0-based index into DFU partition).
    current_page: u16,
    /// Byte offset within the page buffer.
    page_offset: u16,
}

enum State {
    Idle,
    Active(Receiving),
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
        Self {
            state: State::Idle,
        }
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
    ) -> Option<OtaResponse> {
        if msg_data.is_empty() {
            return None;
        }

        let msg_type = msg_data[0];
        let payload = &msg_data[1..];

        match msg_type {
            msg::OTA_OFFER => self.handle_offer(payload),
            msg::OTA_CHUNK => self.handle_chunk(payload, flash),
            msg::OTA_ABORT => {
                self.reset();
                None
            }
            _ => None,
        }
    }

    /// Handle an OTA_OFFER message.
    fn handle_offer(&mut self, payload: &[u8]) -> Option<OtaResponse> {
        if matches!(self.state, State::Active(_)) {
            return Some(make_reject(reason::BUSY));
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
    ) -> Option<OtaResponse> {
        // Extract current state, or bail if idle.
        let (next_chunk, _total_chunks) = match &self.state {
            State::Active(rx) => (rx.next_chunk, rx.offer.total_chunks),
            State::Idle => return None,
        };

        let chunk = OtaChunk::deserialize(payload)?;

        // Duplicate or old chunk — re-ACK.
        if chunk.index < next_chunk {
            return Some(make_ack(chunk.index, next_chunk));
        }

        // Out-of-order — request what we actually need.
        if chunk.index > next_chunk {
            return Some(make_ack(chunk.index, next_chunk));
        }

        // Correct next chunk — copy data into page buffer.
        let data_len = chunk.data_len as usize;

        let rx = match &mut self.state {
            State::Active(rx) => rx,
            _ => unreachable!(),
        };

        let offset = rx.page_offset as usize;
        let space = PAGE_BUF_SIZE - offset;
        let to_copy = data_len.min(space);

        self.page_buf_mut()[offset..offset + to_copy]
            .copy_from_slice(&chunk.data[..to_copy]);

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
                self.page_buf_mut()[..remainder]
                    .copy_from_slice(&chunk.data[to_copy..data_len]);
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

            // Verify CRC32.
            let computed_crc = compute_crc32(DFU_BASE, fw_size);

            if computed_crc == expected_crc {
                crate::boot_state::request_swap(flash);
                let mut buf = [0u8; 32];
                let len = ota_protocol::serialize_complete(&mut buf, computed_crc);
                self.reset();
                return Some(OtaResponse { data: buf, len });
            } else {
                self.reset();
                return Some(make_abort(reason::CRC_MISMATCH));
            }
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
            State::Idle => None,
        }
    }

    /// Whether the receiver is currently in a transfer.
    pub fn is_active(&self) -> bool {
        matches!(self.state, State::Active(_))
    }

    fn reset(&mut self) {
        self.state = State::Idle;
    }
}

// ── Free functions ──────────────────────────────────────────────────────────

/// Compute CRC32 over `size` bytes starting at `base_addr` in flash.
fn compute_crc32(base_addr: u32, size: u32) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    let base = base_addr as *const u8;
    for i in 0..size {
        let byte = unsafe { ptr::read_volatile(base.add(i as usize)) };
        crc ^= byte as u32;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320;
            } else {
                crc >>= 1;
            }
        }
    }
    !crc
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
