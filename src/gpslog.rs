//! GPS + radio link logger (`gps-radio-log` cargo feature).
//!
//! An NMEA GPS receiver hangs off USART1: PB7 = RX (module TX -> MCU) at
//! the 9600 8N1 NMEA default.  The GPS is kept powered and active at all
//! times — no standby or duty cycling.  PB6 (USART1 TX) is left free; the
//! receiver runs with its factory sentence set.
//!
//! Every checksum-valid RMC/GGA sentence, every radio TX/RX event (with
//! packet RSSI, SNR and signal RSSI from the SX126x packet status) and a
//! periodic LoRa stats snapshot (received/CRC error/header error counts)
//! are appended as ASCII lines, timestamped with the millisecond uptime
//! counter, to a raw block region of the SD card:
//!
//! ```text
//! t=<ms> GPS $GPRMC,...*hh
//! t=<ms> RX len=<n> rssi=<dBm> snr=<dB> srssi=<dBm>
//! t=<ms> TX len=<n> ok=<0|1>
//! t=<ms> STATS rx=<n> crc_err=<n> hdr_err=<n>
//! ```
//!
//! There is no filesystem.  Block [`HEADER_LBA`] holds a magic plus the
//! next block to write; data blocks follow and wrap around when the
//! region is full.  Blocks are plain text padded with zero bytes, so the
//! log can be recovered with `dd` + `strings`.

use crate::board::{SdCard, SdError};
use core::cell::RefCell;
use cortex_m::interrupt::{CriticalSection, Mutex};
use stm32wlxx_hal::{
    embedded_hal::serial::Read,
    gpio::pins,
    pac,
    uart::{self, NoTx, Uart1},
};

/// NMEA line rate — the near-universal GPS module factory default.
pub const BAUD: u32 = 9_600;

/// Header block: magic + next data LBA.  Placed 1 MiB into the card to
/// stay clear of any partition table or filesystem metadata.
pub const HEADER_LBA: u32 = 2048;
/// First data block.
pub const DATA_START_LBA: u32 = HEADER_LBA + 1;
/// Data region size in blocks (1 GiB).  The log wraps when full.
pub const DATA_BLOCKS: u32 = 2 * 1024 * 1024;

const HEADER_MAGIC: [u8; 4] = *b"GRL1";
const BLOCK_LEN: usize = 512;

/// Flush a partially filled block this often, so at most a few seconds
/// of log are lost on power failure.
const SYNC_MS: u32 = 10_000;
/// Checkpoint the header every this many data blocks, keeping header
/// wear ~16x below the data rate.  Resume skips this far past the
/// stored pointer, so blocks written since the last checkpoint are
/// never overwritten.
const HEADER_EVERY_BLOCKS: u32 = 16;
/// Retry SD setup this often while the card is missing or failing.
const RETRY_MS: u32 = 10_000;

/// Radio events recorded by the driver, drained by the logger.
///
/// The queue depth covers a burst of mesh forwards between two logger
/// polls; the main loop drains every iteration, so overflow (oldest
/// event dropped) is only a theoretical concern.
pub mod events {
    use super::*;

    #[derive(Clone, Copy)]
    pub enum RadioEvent {
        Tx {
            len: u8,
            ok: bool,
        },
        Rx {
            len: u8,
            /// Packet RSSI (dBm).
            rssi_dbm: i16,
            /// Packet SNR in quarter dB (raw SX126x units).
            snr_qdb: i16,
            /// RSSI of the despread LoRa signal (dBm).
            signal_rssi_dbm: i16,
        },
    }

    const QUEUE_LEN: usize = 8;

    struct Queue {
        buf: [Option<RadioEvent>; QUEUE_LEN],
        head: usize,
        len: usize,
    }

    static QUEUE: Mutex<RefCell<Queue>> = Mutex::new(RefCell::new(Queue {
        buf: [None; QUEUE_LEN],
        head: 0,
        len: 0,
    }));

    pub fn push(ev: RadioEvent) {
        cortex_m::interrupt::free(|cs| {
            let mut q = QUEUE.borrow(cs).borrow_mut();
            if q.len == QUEUE_LEN {
                // Full: drop the oldest so recent events survive.
                q.head = (q.head + 1) % QUEUE_LEN;
                q.len -= 1;
            }
            let tail = (q.head + q.len) % QUEUE_LEN;
            q.buf[tail] = Some(ev);
            q.len += 1;
        })
    }

    pub fn pop() -> Option<RadioEvent> {
        cortex_m::interrupt::free(|cs| {
            let mut q = QUEUE.borrow(cs).borrow_mut();
            if q.len == 0 {
                return None;
            }
            let head = q.head;
            let ev = q.buf[head].take();
            q.head = (head + 1) % QUEUE_LEN;
            q.len -= 1;
            ev
        })
    }

    pub fn note_tx(len: u8, ok: bool) {
        push(RadioEvent::Tx { len, ok });
    }

    pub fn note_rx(len: u8, rssi_dbm: i16, snr_qdb: i16, signal_rssi_dbm: i16) {
        push(RadioEvent::Rx {
            len,
            rssi_dbm,
            snr_qdb,
            signal_rssi_dbm,
        });
    }
}

/// Longest NMEA sentence: 82 characters including "$" and CRLF.
const NMEA_MAX: usize = 82;

/// NMEA receiver on USART1: assembles lines, validates checksums and
/// keeps a little fix state for status reporting.
pub struct Gps {
    uart: Uart1<pins::B7, NoTx>,
    line: [u8; NMEA_MAX],
    len: usize,
    in_line: bool,
    /// GGA fix quality (0 = none, 1 = GPS, 2 = DGPS, ...).
    pub fix_quality: u8,
    /// GGA satellites in use.
    pub sats: u8,
}

impl Gps {
    pub fn new(
        usart1: pac::USART1,
        b7: pins::B7,
        rcc: &mut pac::RCC,
        cs: &CriticalSection,
    ) -> Self {
        let uart = Uart1::new(usart1, BAUD, uart::Clk::PClk, rcc).enable_rx(b7, cs);
        Self {
            uart,
            line: [0; NMEA_MAX],
            len: 0,
            in_line: false,
            fix_quality: 0,
            sats: 0,
        }
    }

    /// Clear sticky UART error flags (overrun keeps erroring until
    /// acknowledged in ICR).  Overruns are expected: other work in the
    /// main loop can block longer than one character time.
    fn clear_errors(&mut self) {
        // The HAL keeps the register block private; USART1 is owned by
        // `self.uart` so this access is exclusive.
        unsafe {
            let usart1 = &*pac::USART1::PTR;
            usart1.icr.write(|w| {
                w.orecf()
                    .set_bit()
                    .fecf()
                    .set_bit()
                    .pecf()
                    .set_bit()
                    .ncf()
                    .set_bit()
            });
        }
    }

    /// Drain the receiver.  Returns the first complete, checksum-valid
    /// RMC or GGA sentence (without CRLF); any further pending bytes
    /// stay in the UART for the next poll.
    pub fn poll_line(&mut self) -> Option<&str> {
        loop {
            let byte = match self.uart.read() {
                Ok(b) => b,
                Err(nb::Error::WouldBlock) => return None,
                Err(nb::Error::Other(_)) => {
                    // Lost bytes: drop the partial line and resync.
                    self.clear_errors();
                    self.in_line = false;
                    self.len = 0;
                    continue;
                }
            };
            match byte {
                b'$' => {
                    self.line[0] = b'$';
                    self.len = 1;
                    self.in_line = true;
                }
                b'\r' | b'\n' => {
                    if self.in_line && self.complete_line() {
                        self.in_line = false;
                        return core::str::from_utf8(&self.line[..self.len]).ok();
                    }
                    self.in_line = false;
                    self.len = 0;
                }
                _ if self.in_line => {
                    if self.len < NMEA_MAX {
                        self.line[self.len] = byte;
                        self.len += 1;
                    } else {
                        // Overlong garbage: resync on the next '$'.
                        self.in_line = false;
                        self.len = 0;
                    }
                }
                _ => {}
            }
        }
    }

    /// Validate and filter the assembled sentence; updates fix state.
    fn complete_line(&mut self) -> bool {
        let line = &self.line[..self.len];
        // "$xxYYY*hh" is the shortest sentence of interest.
        if line.len() < 9 {
            return false;
        }
        let star = self.len - 3;
        if line[star] != b'*' {
            return false;
        }
        let sum = line[1..star].iter().fold(0u8, |acc, &b| acc ^ b);
        let hex = |c: u8| match c {
            b'0'..=b'9' => Some(c - b'0'),
            b'A'..=b'F' => Some(c - b'A' + 10),
            b'a'..=b'f' => Some(c - b'a' + 10),
            _ => None,
        };
        let (Some(hi), Some(lo)) = (hex(line[star + 1]), hex(line[star + 2])) else {
            return false;
        };
        if sum != (hi << 4) | lo {
            return false;
        }
        // Keep position/velocity sentences from any talker (GP/GN/...).
        match &line[3..6] {
            b"GGA" => {
                self.update_fix();
                true
            }
            b"RMC" => true,
            _ => false,
        }
    }

    /// Pull fix quality and satellite count out of a validated GGA
    /// sentence: field 6 is quality, field 7 the satellites in use.
    fn update_fix(&mut self) {
        let mut field = 0usize;
        let mut quality = 0u8;
        let mut sats = 0u8;
        for &b in &self.line[..self.len] {
            if b == b',' {
                field += 1;
                continue;
            }
            if b.is_ascii_digit() {
                let d = b - b'0';
                if field == 6 {
                    quality = quality.saturating_mul(10).saturating_add(d);
                } else if field == 7 {
                    sats = sats.saturating_mul(10).saturating_add(d);
                }
            }
        }
        self.fix_quality = quality;
        self.sats = sats;
    }
}

/// Append-only log over raw SD blocks with a fixed header block.
struct BlockLog {
    buf: [u8; BLOCK_LEN],
    pos: usize,
    /// Block currently being filled.
    lba: u32,
    /// Unsynced bytes in `buf`.
    dirty: bool,
    ready: bool,
}

impl BlockLog {
    const fn new() -> Self {
        Self {
            buf: [0; BLOCK_LEN],
            pos: 0,
            lba: DATA_START_LBA,
            dirty: false,
            ready: false,
        }
    }

    /// Read the header and pick the resume point.  Up to
    /// [`HEADER_EVERY_BLOCKS`] blocks past the stored pointer may hold
    /// data written since the last checkpoint, so resume skips past
    /// them (plus one for a partial tail) rather than overwriting.
    fn setup(&mut self, sd: &mut SdCard) -> Result<(), SdError> {
        if sd.kind().is_none() {
            sd.init()?;
        }
        let mut block = [0u8; BLOCK_LEN];
        sd.read_block(HEADER_LBA, &mut block)?;
        let stored = if block[..4] == HEADER_MAGIC {
            u32::from_le_bytes([block[4], block[5], block[6], block[7]])
        } else {
            DATA_START_LBA
        };
        let in_range = (DATA_START_LBA..DATA_START_LBA + DATA_BLOCKS).contains(&stored);
        self.lba = if in_range {
            DATA_START_LBA + (stored - DATA_START_LBA + HEADER_EVERY_BLOCKS + 1) % DATA_BLOCKS
        } else {
            DATA_START_LBA
        };
        self.pos = 0;
        self.buf.fill(0);
        self.dirty = false;
        self.write_header(sd)?;
        self.ready = true;
        Ok(())
    }

    fn write_header(&mut self, sd: &mut SdCard) -> Result<(), SdError> {
        let mut block = [0u8; BLOCK_LEN];
        block[..4].copy_from_slice(&HEADER_MAGIC);
        block[4..8].copy_from_slice(&self.lba.to_le_bytes());
        sd.write_block(HEADER_LBA, &block)
    }

    /// Append bytes, flushing full blocks as they fill.
    fn append(&mut self, sd: &mut SdCard, data: &[u8]) -> Result<(), SdError> {
        if !self.ready {
            return Ok(());
        }
        for &b in data {
            self.buf[self.pos] = b;
            self.pos += 1;
            self.dirty = true;
            if self.pos == BLOCK_LEN {
                self.flush_block(sd)?;
            }
        }
        Ok(())
    }

    fn flush_block(&mut self, sd: &mut SdCard) -> Result<(), SdError> {
        sd.write_block(self.lba, &self.buf)?;
        self.lba += 1;
        if self.lba >= DATA_START_LBA + DATA_BLOCKS {
            self.lba = DATA_START_LBA;
        }
        self.pos = 0;
        self.buf.fill(0);
        self.dirty = false;
        if (self.lba - DATA_START_LBA) % HEADER_EVERY_BLOCKS == 0 {
            self.write_header(sd)?;
        }
        Ok(())
    }

    /// Persist a partially filled block in place (zero padded; the same
    /// block is rewritten as it fills further).
    fn sync(&mut self, sd: &mut SdCard) -> Result<(), SdError> {
        if !self.ready || !self.dirty {
            return Ok(());
        }
        sd.write_block(self.lba, &self.buf)?;
        self.dirty = false;
        Ok(())
    }
}

/// Format `args` into `buf`, returning the written prefix
/// (truncation-safe: overflow drops the tail, never panics).
fn fmt_line<'a>(buf: &'a mut [u8], args: core::fmt::Arguments) -> &'a [u8] {
    struct W<'b> {
        buf: &'b mut [u8],
        len: usize,
    }
    impl core::fmt::Write for W<'_> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let n = s.len().min(self.buf.len() - self.len);
            self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
            self.len += n;
            Ok(())
        }
    }
    let mut w = W { buf, len: 0 };
    let _ = core::fmt::write(&mut w, args);
    let len = w.len;
    &buf[..len]
}

/// The whole feature: GPS receiver plus SD block logger.
pub struct GpsRadioLog {
    pub gps: Gps,
    log: BlockLog,
    next_sync_ms: u32,
    next_retry_ms: u32,
    last_fix_quality: u8,
}

impl GpsRadioLog {
    pub fn new(
        usart1: pac::USART1,
        b7: pins::B7,
        rcc: &mut pac::RCC,
        cs: &CriticalSection,
    ) -> Self {
        Self {
            gps: Gps::new(usart1, b7, rcc, cs),
            log: BlockLog::new(),
            next_sync_ms: 0,
            next_retry_ms: 0,
            last_fix_quality: 0,
        }
    }

    /// Drive the logger: bring up the SD region, drain the GPS and the
    /// radio event queue, and periodically sync the partial block.
    /// Call every main loop iteration.
    pub fn poll(&mut self, sd: &mut SdCard, now_ms: u32) {
        if !self.log.ready && now_ms.wrapping_sub(self.next_retry_ms) < i32::MAX as u32 {
            self.next_retry_ms = now_ms.wrapping_add(RETRY_MS);
            if sd.card_present() {
                match self.log.setup(sd) {
                    Ok(()) => rtt_target::rprintln!(
                        "gps-radio-log: logging at block {}",
                        self.log.lba
                    ),
                    Err(e) => debug_println!("gps-radio-log: SD setup failed: {:?}", e),
                }
            }
        }

        let mut line = [0u8; 128];

        while let Some(ev) = events::pop() {
            let text = match ev {
                events::RadioEvent::Tx { len, ok } => fmt_line(
                    &mut line,
                    format_args!("t={} TX len={} ok={}\n", now_ms, len, ok as u8),
                ),
                events::RadioEvent::Rx {
                    len,
                    rssi_dbm,
                    snr_qdb,
                    signal_rssi_dbm,
                } => {
                    // Quarter-dB SNR printed as a signed decimal.
                    let snr_cdb = snr_qdb as i32 * 25;
                    let sign = if snr_cdb < 0 { "-" } else { "" };
                    let mag = snr_cdb.unsigned_abs();
                    fmt_line(
                        &mut line,
                        format_args!(
                            "t={} RX len={} rssi={} snr={}{}.{:02} srssi={}\n",
                            now_ms,
                            len,
                            rssi_dbm,
                            sign,
                            mag / 100,
                            mag % 100,
                            signal_rssi_dbm
                        ),
                    )
                }
            };
            self.write(sd, text);
        }

        let mut got_sentence = false;
        if let Some(sentence) = self.gps.poll_line() {
            let text = fmt_line(&mut line, format_args!("t={} GPS {}\n", now_ms, sentence));
            got_sentence = true;
            self.write(sd, text);
        }
        if got_sentence && self.gps.fix_quality != self.last_fix_quality {
            self.last_fix_quality = self.gps.fix_quality;
            rtt_target::rprintln!(
                "GPS fix quality {} ({} sats)",
                self.gps.fix_quality,
                self.gps.sats
            );
        }

        if now_ms.wrapping_sub(self.next_sync_ms) < i32::MAX as u32 {
            self.next_sync_ms = now_ms.wrapping_add(SYNC_MS);
            if let Err(e) = self.log.sync(sd) {
                self.sd_failed(e);
            }
        }
    }

    /// Log a LoRa stats snapshot (see [`crate::radio::Sx1262Driver::lora_stats`]).
    pub fn log_stats(&mut self, sd: &mut SdCard, now_ms: u32, stats: (u16, u16, u16)) {
        let mut line = [0u8; 64];
        let text = fmt_line(
            &mut line,
            format_args!(
                "t={} STATS rx={} crc_err={} hdr_err={}\n",
                now_ms, stats.0, stats.1, stats.2
            ),
        );
        self.write(sd, text);
    }

    fn write(&mut self, sd: &mut SdCard, text: &[u8]) {
        if let Err(e) = self.log.append(sd, text) {
            self.sd_failed(e);
        }
    }

    /// Drop back to not-ready so the retry path re-runs card init;
    /// covers card removal and transient SPI errors alike.
    fn sd_failed(&mut self, e: SdError) {
        debug_println!("gps-radio-log: SD write failed: {:?}", e);
        self.log.ready = false;
        self.log.dirty = false;
    }
}
