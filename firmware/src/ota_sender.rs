//! OTA firmware-update sender (initiator).
//!
//! Reads firmware from the DFU flash partition and pushes it chunk-by-chunk
//! to a target node over the mesh radio link.
//!
//! # Usage
//!
//! 1. Flash the new firmware `.bin` into the DFU partition (pages 64+) using
//!    probe-rs or another tool.
//! 2. Create an [`OtaSender`] with the firmware metadata.
//! 3. Call [`OtaSender::next_message`] in the main loop to get the next
//!    outgoing OTA message.
//! 4. Route incoming OTA responses to [`OtaSender::handle_message`].
//!
//! The sender follows stop-and-wait ARQ: it sends one chunk, waits for an
//! ACK, then sends the next.  Retransmission on timeout is handled
//! internally.

use core::ptr;

use crate::ota_protocol::{msg, CHUNK_DATA_SIZE, OtaChunk, OtaOffer};

// ── Flash constants (must match ota.rs) ─────────────────────────────────────

const FLASH_BASE: u32 = 0x0800_0000;
const PAGE_SIZE: u32 = 2048;

/// DFU staging partition base address.
const DFU_BASE: u32 = FLASH_BASE + 64 * PAGE_SIZE; // 0x0802_0000

// ── Sender state ────────────────────────────────────────────────────────────

/// Retransmit a chunk if no ACK arrives within this many calls to
/// `next_message`.  At ~1 ms per loop iteration this is roughly 2 seconds.
const RETRANSMIT_TIMEOUT: u32 = 2000;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Send the initial OTA_OFFER.
    Offering,
    /// Waiting for OTA_ACCEPT / OTA_REJECT after sending the offer.
    WaitAccept,
    /// Sending chunks (stop-and-wait).
    Sending,
    /// Transfer finished (success or failure).
    Done,
}

/// Result of a completed (or failed) OTA transfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtaResult {
    /// Target confirmed CRC match and will reboot.
    Success,
    /// Target rejected the offer (reason code).
    Rejected(u8),
    /// Target aborted the transfer (reason code).
    Aborted(u8),
}

/// OTA firmware-update sender.
pub struct OtaSender {
    offer: OtaOffer,
    phase: Phase,
    /// Current chunk index to (re-)transmit.
    current_chunk: u16,
    /// Countdown ticks until retransmit.
    retransmit_timer: u32,
    /// Final result once `phase == Done`.
    result: Option<OtaResult>,
}

/// An outgoing OTA message to send to the target node.
pub struct OtaMessage {
    pub data: [u8; 32],
    pub len: usize,
}

impl OtaSender {
    /// Create a new sender for firmware stored in the DFU partition.
    ///
    /// # Arguments
    /// - `firmware_size`: total firmware size in bytes (must fit in DFU partition).
    /// - `crc32`: expected CRC32 of the firmware data.
    /// - `version`: firmware version number.
    pub fn new(firmware_size: u32, crc32: u32, version: u16) -> Self {
        let total_chunks =
            firmware_size.div_ceil(CHUNK_DATA_SIZE as u32) as u16;
        Self {
            offer: OtaOffer {
                firmware_size,
                total_chunks,
                crc32,
                version,
            },
            phase: Phase::Offering,
            current_chunk: 0,
            retransmit_timer: 0,
            result: None,
        }
    }

    /// Compute the CRC32 of the firmware in the DFU partition.
    ///
    /// Call this before creating the sender to obtain the `crc32` parameter.
    pub fn compute_dfu_crc32(firmware_size: u32) -> u32 {
        let mut crc: u32 = 0xFFFF_FFFF;
        let base = DFU_BASE as *const u8;
        for i in 0..firmware_size {
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

    /// Get the next message to send, if any.
    ///
    /// Call this on every main-loop iteration.  Returns `Some(msg)` when there
    /// is something to transmit, `None` when waiting for a response.
    pub fn next_message(&mut self) -> Option<OtaMessage> {
        match self.phase {
            Phase::Offering => {
                self.phase = Phase::WaitAccept;
                self.retransmit_timer = RETRANSMIT_TIMEOUT;
                let mut buf = [0u8; 32];
                let len = self.offer.serialize(&mut buf);
                Some(OtaMessage { data: buf, len })
            }
            Phase::WaitAccept => {
                if self.retransmit_timer == 0 {
                    // Retransmit offer
                    self.retransmit_timer = RETRANSMIT_TIMEOUT;
                    let mut buf = [0u8; 32];
                    let len = self.offer.serialize(&mut buf);
                    Some(OtaMessage { data: buf, len })
                } else {
                    self.retransmit_timer = self.retransmit_timer.saturating_sub(1);
                    None
                }
            }
            Phase::Sending => {
                if self.retransmit_timer == 0 {
                    // (Re-)transmit current chunk
                    self.retransmit_timer = RETRANSMIT_TIMEOUT;
                    Some(self.build_chunk(self.current_chunk))
                } else {
                    self.retransmit_timer = self.retransmit_timer.saturating_sub(1);
                    None
                }
            }
            Phase::Done => None,
        }
    }

    /// Handle an incoming OTA response from the target node.
    ///
    /// Returns `Some(result)` when the transfer is finished.
    pub fn handle_message(&mut self, msg_data: &[u8]) -> Option<OtaResult> {
        if msg_data.is_empty() {
            return None;
        }

        let msg_type = msg_data[0];
        let payload = &msg_data[1..];

        match msg_type {
            msg::OTA_ACCEPT => {
                if self.phase == Phase::WaitAccept {
                    self.phase = Phase::Sending;
                    self.current_chunk = 0;
                    // Send first chunk immediately
                    self.retransmit_timer = 0;
                }
                None
            }
            msg::OTA_REJECT => {
                let reason = payload.first().copied().unwrap_or(0);
                self.phase = Phase::Done;
                self.result = Some(OtaResult::Rejected(reason));
                self.result
            }
            msg::OTA_CHUNK_ACK => {
                if self.phase == Phase::Sending && payload.len() >= 4 {
                    let _ack_index = u16::from_le_bytes([payload[0], payload[1]]);
                    let next_needed = u16::from_le_bytes([payload[2], payload[3]]);

                    if next_needed >= self.offer.total_chunks {
                        // All chunks acknowledged, wait for OTA_COMPLETE
                        self.current_chunk = next_needed;
                        self.retransmit_timer = RETRANSMIT_TIMEOUT;
                    } else {
                        self.current_chunk = next_needed;
                        // Send next chunk immediately
                        self.retransmit_timer = 0;
                    }
                }
                None
            }
            msg::OTA_COMPLETE => {
                self.phase = Phase::Done;
                self.result = Some(OtaResult::Success);
                self.result
            }
            msg::OTA_ABORT => {
                let reason = payload.first().copied().unwrap_or(0);
                self.phase = Phase::Done;
                self.result = Some(OtaResult::Aborted(reason));
                self.result
            }
            _ => None,
        }
    }

    /// Return current progress as (sent_chunks, total_chunks), or `None` if done.
    pub fn progress(&self) -> Option<(u16, u16)> {
        match self.phase {
            Phase::Done => None,
            _ => Some((self.current_chunk, self.offer.total_chunks)),
        }
    }

    /// Whether the transfer has finished.
    pub fn is_done(&self) -> bool {
        self.phase == Phase::Done
    }

    /// Get the final result after the transfer is done.
    pub fn result(&self) -> Option<OtaResult> {
        self.result
    }

    // ── Internal helpers ────────────────────────────────────────────────────

    /// Build an OTA_CHUNK message for the given chunk index by reading
    /// directly from the DFU flash partition.
    fn build_chunk(&self, index: u16) -> OtaMessage {
        let byte_offset = index as u32 * CHUNK_DATA_SIZE as u32;
        let remaining = self.offer.firmware_size.saturating_sub(byte_offset);
        let data_len = (remaining as usize).min(CHUNK_DATA_SIZE);

        let mut chunk_data = [0u8; CHUNK_DATA_SIZE];
        let src = (DFU_BASE + byte_offset) as *const u8;
        for (i, byte) in chunk_data.iter_mut().enumerate().take(data_len) {
            *byte = unsafe { ptr::read_volatile(src.add(i)) };
        }

        let chunk = OtaChunk {
            index,
            data: chunk_data,
            data_len: data_len as u8,
        };
        let mut buf = [0u8; 32];
        let len = chunk.serialize(&mut buf);
        OtaMessage { data: buf, len }
    }
}
