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

/// Bytes drained from USART1 per [`poll_line`](Gps::poll_line) call. A fix
/// at the 1 Hz default emits a few hundred bytes a second, so the main
/// loop's polling never approaches this in normal use; the cap exists for
/// the opposite case, where no module is attached and the floating RX pin
/// streams noise that never completes a sentence. Bounding the drain keeps
/// that from monopolizing the loop and starving the radio receive, so a node
/// with no GPS still hears the network.
const DRAIN_BUDGET: usize = 512;

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
    /// Latest latitude in signed decimal degrees (north positive).
    pub lat_deg: f32,
    /// Latest longitude in signed decimal degrees (east positive).
    pub lon_deg: f32,
    /// Whether `lat_deg`/`lon_deg` hold a currently valid fix.
    pub has_pos: bool,
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
            lat_deg: 0.0,
            lon_deg: 0.0,
            has_pos: false,
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
        for _ in 0..DRAIN_BUDGET {
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
        None
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
            b"RMC" => {
                self.update_rmc();
                true
            }
            _ => false,
        }
    }

    /// Pull fix quality, satellite count and position out of a validated
    /// GGA sentence: field 2/3 latitude, field 4/5 longitude, field 6
    /// quality, field 7 satellites in use.
    fn update_fix(&mut self) {
        let line = &self.line[..self.len];
        let quality = parse_u8_field(nth_field(line, 6));
        let sats = parse_u8_field(nth_field(line, 7));
        let pos = if quality > 0 {
            parse_coord(nth_field(line, 2), nth_field(line, 3))
                .zip(parse_coord(nth_field(line, 4), nth_field(line, 5)))
        } else {
            None
        };
        self.fix_quality = quality;
        self.sats = sats;
        match pos {
            Some((lat, lon)) => {
                self.lat_deg = lat;
                self.lon_deg = lon;
                self.has_pos = true;
            }
            // A quality-0 GGA reports no fix; drop any stale position.
            None if quality == 0 => self.has_pos = false,
            None => {}
        }
    }

    /// Pull position out of a validated RMC sentence: field 2 is the
    /// A/V status, field 3/4 latitude, field 5/6 longitude.  RMC carries
    /// no satellite count, so `sats`/`fix_quality` are left to GGA.
    fn update_rmc(&mut self) {
        let line = &self.line[..self.len];
        let active = nth_field(line, 2).first() == Some(&b'A');
        let pos = if active {
            parse_coord(nth_field(line, 3), nth_field(line, 4))
                .zip(parse_coord(nth_field(line, 5), nth_field(line, 6)))
        } else {
            None
        };
        match pos {
            Some((lat, lon)) => {
                self.lat_deg = lat;
                self.lon_deg = lon;
                self.has_pos = true;
            }
            None if !active => self.has_pos = false,
            None => {}
        }
    }

    /// One-line status for the display combining the satellite count,
    /// the RSSI of the last received radio packet, and how long ago that
    /// packet arrived, e.g. "08 -95dBm 1m23s".  `since_rx` is the elapsed
    /// milliseconds since the last packet, or `None` when nothing has been
    /// heard yet (rendered "08 no rx").
    pub fn fmt_status_line<'a>(
        &self,
        buf: &'a mut [u8; 24],
        rssi: i16,
        since_rx: Option<u32>,
    ) -> &'a str {
        let bytes = match since_rx {
            Some(ms) => {
                let secs = ms / 1000;
                fmt_line(
                    buf,
                    format_args!("{:02} {}dBm {}m{:02}s", self.sats, rssi, secs / 60, secs % 60),
                )
            }
            None => fmt_line(buf, format_args!("{:02} no rx", self.sats)),
        };
        core::str::from_utf8(bytes).unwrap_or("")
    }

    /// Latitude for the display: magnitude and an N/S suffix, e.g.
    /// "48.11730 N".
    pub fn fmt_lat<'a>(&self, buf: &'a mut [u8; 16]) -> &'a str {
        let (mag, hemi) = if self.lat_deg < 0.0 {
            (-self.lat_deg, 'S')
        } else {
            (self.lat_deg, 'N')
        };
        core::str::from_utf8(fmt_line(buf, format_args!("{:.5} {}", mag, hemi))).unwrap_or("")
    }

    /// Longitude for the display: magnitude and an E/W suffix, e.g.
    /// "11.51670 E".
    pub fn fmt_lon<'a>(&self, buf: &'a mut [u8; 16]) -> &'a str {
        let (mag, hemi) = if self.lon_deg < 0.0 {
            (-self.lon_deg, 'W')
        } else {
            (self.lon_deg, 'E')
        };
        core::str::from_utf8(fmt_line(buf, format_args!("{:.5} {}", mag, hemi))).unwrap_or("")
    }
}

/// The `n`th comma-separated field of an NMEA sentence (field 0 is the
/// `$xxYYY` id).  Any trailing `*hh` checksum is stripped from the last
/// field.  Returns an empty slice when the field is absent.
fn nth_field(line: &[u8], n: usize) -> &[u8] {
    let mut idx = 0usize;
    let mut start = 0usize;
    let mut i = 0usize;
    while i < line.len() {
        match line[i] {
            b',' => {
                if idx == n {
                    return &line[start..i];
                }
                idx += 1;
                start = i + 1;
            }
            b'*' => break,
            _ => {}
        }
        i += 1;
    }
    if idx == n {
        &line[start..i]
    } else {
        &[]
    }
}

/// Parse a run of ASCII digits, ignoring any other bytes.
fn parse_u8_field(field: &[u8]) -> u8 {
    let mut v = 0u8;
    for &b in field {
        if b.is_ascii_digit() {
            v = v.saturating_mul(10).saturating_add(b - b'0');
        }
    }
    v
}

/// Convert an NMEA `ddmm.mmmm` (lat) or `dddmm.mmmm` (lon) coordinate to
/// signed decimal degrees, applying the N/S/E/W hemisphere field.
fn parse_coord(field: &[u8], hemi: &[u8]) -> Option<f32> {
    let dot = field.iter().position(|&b| b == b'.')?;
    // Need at least one degree digit plus the two whole-minute digits.
    if dot < 3 {
        return None;
    }
    let deg_end = dot - 2;
    let mut deg = 0f32;
    for &b in &field[..deg_end] {
        if !b.is_ascii_digit() {
            return None;
        }
        deg = deg * 10.0 + (b - b'0') as f32;
    }
    let mut min = 0f32;
    let mut scale = 0f32; // 0 until the decimal point is seen
    for &b in &field[deg_end..] {
        if b == b'.' {
            scale = 1.0;
            continue;
        }
        if !b.is_ascii_digit() {
            return None;
        }
        let d = (b - b'0') as f32;
        if scale == 0.0 {
            min = min * 10.0 + d;
        } else {
            scale *= 10.0;
            min += d / scale;
        }
    }
    let mut deg = deg + min / 60.0;
    if matches!(hemi.first(), Some(b'S') | Some(b'W')) {
        deg = -deg;
    }
    Some(deg)
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
                    Ok(()) => {
                        rtt_target::rprintln!("gps-radio-log: logging at block {}", self.log.lba);
                    }
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
            // Mirror each radio event to RTT as well; RX lines carry the
            // packet RSSI/SNR and signal RSSI.
            if let Ok(s) = core::str::from_utf8(text) {
                rtt_target::rprintln!("{}", s.trim_end());
            }
            self.write(sd, text);
        }

        let mut got_sentence = false;
        if let Some(sentence) = self.gps.poll_line() {
            rtt_target::rprintln!("GPS {}", sentence);
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
