//! UART frame protocol for basestation ↔ host communication.
//!
//! Frame format:
//!   `[SYNC 0xAA] [LEN_LO] [LEN_HI] [CMD] [PAYLOAD: LEN bytes] [CRC8]`
//!
//! LEN is the payload length only (0–256).  CRC8 covers CMD + PAYLOAD.

/// Frame sync byte.
pub const SYNC: u8 = 0xAA;

/// Maximum payload size per frame.
pub const MAX_PAYLOAD: usize = 256;

// ── Command IDs ────────────────────────────────────────────────────────────

/// Commands from host → basestation.
pub mod cmd {
    // OTA commands (0x01–0x0F)
    pub const START_OTA: u8 = 0x01;
    pub const FW_DATA: u8 = 0x02;
    pub const BEGIN_TRANSFER: u8 = 0x03;
    pub const QUERY_STATUS: u8 = 0x04;
    pub const ABORT_OTA: u8 = 0x05;

    // Data relay commands (0x10–0x1F)
    pub const SEND_MSG: u8 = 0x10;
    pub const SEND_BROADCAST: u8 = 0x11;
}

/// Responses from basestation → host.
pub mod resp {
    pub const ACK: u8 = 0x81;
    pub const NAK: u8 = 0x82;
    pub const PROGRESS: u8 = 0x83;
    pub const OTA_DONE: u8 = 0x84;

    // Data relay responses (0x90–0x9F)
    pub const RECV_MSG: u8 = 0x90;
}

/// OTA state codes (used in PROGRESS responses).
pub mod ota_state {
    pub const IDLE: u8 = 0x00;
    pub const RECEIVING_FW: u8 = 0x01;
    pub const TRANSFERRING: u8 = 0x02;
    pub const COMPLETE: u8 = 0x03;
    #[allow(dead_code)]
    pub const FAILED: u8 = 0x04;
}

/// NAK error codes.
pub mod err {
    pub const INVALID_STATE: u8 = 0x01;
    pub const BAD_SIZE: u8 = 0x02;
    pub const FLASH_ERROR: u8 = 0x03;
    pub const BAD_FRAME: u8 = 0x04;
    pub const NO_FW_DATA: u8 = 0x05;
}

// ── CRC-8 ──────────────────────────────────────────────────────────────────

/// CRC-8 with polynomial 0x07 (CRC-8/ITU).
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// ── Frame parser (byte-at-a-time state machine) ────────────────────────────

#[derive(Clone, Copy)]
enum ParseState {
    Sync,
    LenLo,
    LenHi,
    Data, // collects CMD + PAYLOAD
    Crc,
}

/// A parsed frame ready for processing.
pub struct Frame<'a> {
    pub cmd: u8,
    pub payload: &'a [u8],
}

/// Incremental frame parser.  Feed bytes one at a time via [`feed`].
pub struct FrameParser {
    state: ParseState,
    /// Buffer holding CMD + PAYLOAD.
    buf: [u8; MAX_PAYLOAD + 1],
    /// Total expected bytes in buf (1 cmd + len payload).
    expected: usize,
    /// Current write position in buf.
    pos: usize,
}

impl FrameParser {
    pub const fn new() -> Self {
        Self {
            state: ParseState::Sync,
            buf: [0u8; MAX_PAYLOAD + 1],
            expected: 0,
            pos: 0,
        }
    }

    /// Feed a single byte.  Returns `Some(Frame)` when a complete valid
    /// frame has been received.  The returned `Frame` borrows from `self`
    /// so it must be consumed before the next call to `feed`.
    pub fn feed(&mut self, byte: u8) -> bool {
        match self.state {
            ParseState::Sync => {
                if byte == SYNC {
                    self.state = ParseState::LenLo;
                }
            }
            ParseState::LenLo => {
                self.expected = byte as usize;
                self.state = ParseState::LenHi;
            }
            ParseState::LenHi => {
                self.expected |= (byte as usize) << 8;
                if self.expected > MAX_PAYLOAD {
                    self.state = ParseState::Sync;
                } else {
                    // We'll collect CMD (1 byte) + PAYLOAD (expected bytes)
                    self.expected += 1; // +1 for CMD byte
                    self.pos = 0;
                    self.state = ParseState::Data;
                }
            }
            ParseState::Data => {
                if self.pos < self.expected {
                    self.buf[self.pos] = byte;
                    self.pos += 1;
                }
                if self.pos >= self.expected {
                    self.state = ParseState::Crc;
                }
            }
            ParseState::Crc => {
                let computed = crc8(&self.buf[..self.expected]);
                self.state = ParseState::Sync;
                if computed == byte {
                    return true; // frame ready — call frame() to read it
                }
                // CRC mismatch: discard frame
            }
        }
        false
    }

    /// Get the last parsed frame.  Only valid immediately after `feed`
    /// returns `true`.
    pub fn frame(&self) -> Frame<'_> {
        Frame {
            cmd: self.buf[0],
            payload: &self.buf[1..self.expected],
        }
    }
}

// ── Frame builder (serialize outgoing frames) ──────────────────────────────

/// Scratch buffer for building outgoing frames.
/// Max frame size: 1 sync + 2 len + 1 cmd + 256 payload + 1 crc = 261
pub struct FrameBuf {
    pub buf: [u8; 261],
    pub len: usize,
}

impl FrameBuf {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; 261],
            len: 0,
        }
    }

    /// Build a frame with the given command and payload.
    pub fn build(&mut self, cmd: u8, payload: &[u8]) {
        let plen = payload.len().min(MAX_PAYLOAD);
        self.buf[0] = SYNC;
        self.buf[1] = plen as u8;
        self.buf[2] = (plen >> 8) as u8;
        self.buf[3] = cmd;
        self.buf[4..4 + plen].copy_from_slice(&payload[..plen]);

        // CRC over CMD + PAYLOAD
        let crc_data_end = 4 + plen;
        let crc = crc8(&self.buf[3..crc_data_end]);
        self.buf[crc_data_end] = crc;
        self.len = crc_data_end + 1;
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}
