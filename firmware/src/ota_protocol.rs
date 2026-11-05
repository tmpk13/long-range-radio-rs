//! OTA firmware-update protocol message types and serialization.
//!
//! All messages fit within a 32-byte `embedded-nano-mesh` payload.
//! The first byte is the message type; remaining bytes are the payload.

/// Message type IDs (first byte of mesh payload).
pub mod msg {
    pub const OTA_OFFER: u8 = 0xF0;
    pub const OTA_ACCEPT: u8 = 0xF1;
    pub const OTA_REJECT: u8 = 0xF2;
    pub const OTA_CHUNK: u8 = 0xF3;
    pub const OTA_CHUNK_ACK: u8 = 0xF4;
    pub const OTA_ABORT: u8 = 0xF5;
    pub const OTA_COMPLETE: u8 = 0xF6;
}

/// Maximum data bytes per chunk (32 − 1 type − 2 index − 1 length = 27).
pub const CHUNK_DATA_SIZE: usize = 27;

/// Reject / abort reason codes.
pub mod reason {
    pub const NO_SPACE: u8 = 0x01;
    pub const BAD_VERSION: u8 = 0x02;
    pub const CRC_MISMATCH: u8 = 0x03;
    pub const FLASH_ERROR: u8 = 0x04;
    pub const TIMEOUT: u8 = 0x05;
    pub const BUSY: u8 = 0x06;
}

// ── Offer ───────────────────────────────────────────────────────────────────

/// Firmware offer sent from initiator → target.
#[derive(Debug, Clone, Copy)]
pub struct OtaOffer {
    pub firmware_size: u32,
    pub total_chunks: u16,
    pub crc32: u32,
    pub version: u16,
}

impl OtaOffer {
    /// Serialize into a 32-byte buffer (type prefix included). Returns slice length.
    pub fn serialize(&self, buf: &mut [u8; 32]) -> usize {
        buf[0] = msg::OTA_OFFER;
        buf[1..5].copy_from_slice(&self.firmware_size.to_le_bytes());
        buf[5..7].copy_from_slice(&self.total_chunks.to_le_bytes());
        buf[7..11].copy_from_slice(&self.crc32.to_le_bytes());
        buf[11..13].copy_from_slice(&self.version.to_le_bytes());
        13
    }

    /// Deserialize from raw bytes (excluding the type prefix byte).
    pub fn deserialize(data: &[u8]) -> Option<Self> {
        if data.len() < 12 {
            return None;
        }
        Some(Self {
            firmware_size: u32::from_le_bytes([data[0], data[1], data[2], data[3]]),
            total_chunks: u16::from_le_bytes([data[4], data[5]]),
            crc32: u32::from_le_bytes([data[6], data[7], data[8], data[9]]),
            version: u16::from_le_bytes([data[10], data[11]]),
        })
    }
}

// ── Chunk ───────────────────────────────────────────────────────────────────

/// A single firmware chunk: index + up to 28 bytes of data.
#[derive(Debug, Clone)]
pub struct OtaChunk {
    pub index: u16,
    pub data: [u8; CHUNK_DATA_SIZE],
    pub data_len: u8,
}

impl OtaChunk {
    pub fn serialize(&self, buf: &mut [u8; 32]) -> usize {
        buf[0] = msg::OTA_CHUNK;
        buf[1..3].copy_from_slice(&self.index.to_le_bytes());
        buf[3] = self.data_len;
        let end = 4 + self.data_len as usize;
        buf[4..end].copy_from_slice(&self.data[..self.data_len as usize]);
        end
    }

    pub fn deserialize(data: &[u8]) -> Option<Self> {
        if data.len() < 3 {
            return None;
        }
        let index = u16::from_le_bytes([data[0], data[1]]);
        let data_len = data[2].min(CHUNK_DATA_SIZE as u8);
        let payload = &data[3..];
        if payload.len() < data_len as usize {
            return None;
        }
        let mut chunk_data = [0u8; CHUNK_DATA_SIZE];
        chunk_data[..data_len as usize].copy_from_slice(&payload[..data_len as usize]);
        Some(Self {
            index,
            data: chunk_data,
            data_len,
        })
    }
}

// ── Chunk ACK ───────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy)]
pub struct OtaChunkAck {
    pub chunk_index: u16,
    pub next_needed: u16,
}

impl OtaChunkAck {
    pub fn serialize(&self, buf: &mut [u8; 32]) -> usize {
        buf[0] = msg::OTA_CHUNK_ACK;
        buf[1..3].copy_from_slice(&self.chunk_index.to_le_bytes());
        buf[3..5].copy_from_slice(&self.next_needed.to_le_bytes());
        5
    }
}

// ── Simple response messages ────────────────────────────────────────────────

/// Serialize a simple 1-byte-type message (ACCEPT).
pub fn serialize_accept(buf: &mut [u8; 32]) -> usize {
    buf[0] = msg::OTA_ACCEPT;
    1
}

/// Serialize a REJECT with reason.
pub fn serialize_reject(buf: &mut [u8; 32], reason: u8) -> usize {
    buf[0] = msg::OTA_REJECT;
    buf[1] = reason;
    2
}

/// Serialize an ABORT with reason.
pub fn serialize_abort(buf: &mut [u8; 32], reason: u8) -> usize {
    buf[0] = msg::OTA_ABORT;
    buf[1] = reason;
    2
}

/// Serialize OTA_COMPLETE with verified CRC.
pub fn serialize_complete(buf: &mut [u8; 32], crc32: u32) -> usize {
    buf[0] = msg::OTA_COMPLETE;
    buf[1..5].copy_from_slice(&crc32.to_le_bytes());
    5
}

// ── Helpers ─────────────────────────────────────────────────────────────────

/// Returns `true` if the first byte of a mesh payload is an OTA message type.
pub fn is_ota_message(data: &[u8]) -> bool {
    matches!(data.first(), Some(0xF0..=0xF6))
}
