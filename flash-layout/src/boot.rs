//! The boot-state log and the bootloader's slot swap.
//!
//! # The log
//!
//! The boot state lives on [`STATE_PAGE`](crate::STATE_PAGE) as an
//! append-only list of 8-byte records, one per double-word, the flash's
//! programming unit.  The last record that decodes is the current state.
//! Only the app's [`Log::request_swap`] erases the page, while nothing is
//! pending: a power cut there just loses the request.  Every other change
//! programs the next erased double-word, so no cut can make a pending swap
//! read as idle.
//!
//! A cut while a double-word is being programmed leaves it undefined (RM0461
//! 3.3.6).  It may read back as garbage, which the record's 16-bit check
//! rejects, or with a two-bit ECC error, which raises an NMI (RM0461 3.3.3);
//! the [`Flash`] implementations turn that into [`Unreadable`].  Either way
//! the scan skips it, and the state is the last record before it.
//!
//! # The swap
//!
//! A swap is `3 * slot_pages` steps, three page copies per page `p`:
//!
//! | Step  | Copy                  |
//! |-------|-----------------------|
//! | `3p`  | active `p` to scratch |
//! | `3p+1`| DFU `p` to active `p` |
//! | `3p+2`| scratch to DFU `p`    |
//!
//! After each copy the bootloader appends `Swap(steps done)`.  A resume runs
//! exactly the next step, and each step's source is untouched until a later
//! step runs: a cut mid-copy, or between a copy and its record, only means
//! the step runs again, over a destination it erases first.  The resume
//! never reads a page that may be half-written.
//!
//! A revert copies DFU `p` to active `p` for every page, recording
//! `Revert(pages done)` after each; the DFU slot is never written, so every
//! revert step can run again too.
//!
//! The format is a contract between the bootloader and the app.  It
//! replaces the single erase-and-rewrite record of the first bootloader,
//! whose words never carry this module's mark: an old bootloader sees a new
//! app's requests as nothing pending, and this one sees an old app's as the
//! same, so mixing the two never swaps anything.  Flash both together.

use crate::crc32::Crc32;
use crate::{
    ACTIVE_BASE, ACTIVE_PAGE, ACTIVE_PAGES, DFU_PAGE, MAX_FW_SIZE, PAGE_SIZE, RAM_BASE, RAM_SIZE,
    SCRATCH_PAGE, STATE_PAGE,
};

#[cfg(test)]
mod tests;

/// An erased double-word.
pub const ERASED: u64 = u64::MAX;

/// The pages the log and the swap work on.
#[derive(Clone, Copy, Debug)]
pub struct Geometry {
    /// Double-words per slot page.
    pub page_dws: usize,
    /// Pages per slot.
    pub slot_pages: u32,
    /// First page of the active slot.
    pub active_page: u32,
    /// First page of the DFU slot.
    pub dfu_page: u32,
    /// The page that holds one active page while it is swapped.
    pub scratch_page: u32,
    /// The log's page.
    pub state_page: u32,
    /// Records the log page holds.
    pub log_slots: usize,
}

impl Geometry {
    /// Page copies in a whole swap.
    pub const fn swap_steps(&self) -> u32 {
        3 * self.slot_pages
    }

    /// Bytes in a slot.
    pub const fn slot_bytes(&self) -> u32 {
        self.slot_pages * self.page_dws as u32 * 8
    }
}

/// The STM32WLE5JC's pages (see the crate docs for the map).
pub const GEOMETRY: Geometry = Geometry {
    page_dws: (PAGE_SIZE / 8) as usize,
    slot_pages: ACTIVE_PAGES,
    active_page: ACTIVE_PAGE,
    dfu_page: DFU_PAGE,
    scratch_page: SCRATCH_PAGE,
    state_page: STATE_PAGE,
    log_slots: (PAGE_SIZE / 8) as usize,
};

/// A double-word the flash's ECC could not correct.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Unreadable;

/// Why a flash operation did not complete.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    /// It failed or did not verify; the flash can still be used.
    Failed,
    /// The flash is gone, power lost: stop at once.  Only the host tests'
    /// simulated flash reports it.
    Halted,
    /// The log page has no erased double-word left.
    Full,
}

/// Flash access for the log and the swap, by page number and double-word
/// index within the page.
pub trait Flash {
    /// Read a double-word; `Err` when the flash's ECC rejected it.
    fn read(&mut self, page: u32, index: usize) -> Result<u64, Unreadable>;
    /// Program an erased double-word.
    fn program(&mut self, page: u32, index: usize, value: u64) -> Result<(), Fault>;
    /// Erase a page.
    fn erase(&mut self, page: u32) -> Result<(), Fault>;
    /// Make page `dst` a copy of page `src` and verify it.  A source
    /// double-word the ECC rejects is copied as whatever it reads: only a
    /// slot's unused tail can hold one, left there by an earlier transfer
    /// that was cut short.
    fn copy(&mut self, src: u32, dst: u32) -> Result<(), Fault>;
    /// Called before each page of work; the bootloader feeds its watchdog.
    fn tick(&mut self) {}
}

/// Top byte of every record's low word.  Erased flash and the four state
/// words of the first bootloader (`0x4F4B_4F4B`, `0x5357_4150`,
/// `0x444F_4E45`, `0x5245_5654`) never carry it.
const MARK: u32 = 0xB7;

/// One log record.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Record {
    /// Size in bytes of the image staged in the DFU slot.
    ImageSize(u32),
    /// CRC-32 of the staged image.
    ImageCrc(u32),
    /// Swap steps done.  `Swap(0)`, written by the app, asks the bootloader
    /// to verify the staged image and swap it in.
    Swap(u32),
    /// The swap finished; the new image runs on trial until it confirms.
    Swapped,
    /// Pages copied back from the DFU slot after a failed trial.
    Revert(u32),
    /// Nothing pending: the running image confirmed itself, or a revert
    /// finished.
    Confirmed,
    /// The bootloader refused the staged image; one of the `REFUSED_*`
    /// reasons.
    Refused(u32),
}

/// The swap request names no image.
pub const REFUSED_NO_IMAGE: u32 = 1;
/// The image is empty or larger than a slot.
pub const REFUSED_SIZE: u32 = 2;
/// The staged image does not start with this app's vector table.
pub const REFUSED_VECTORS: u32 = 3;
/// The flash's ECC rejected part of the staged image.
pub const REFUSED_UNREADABLE: u32 = 4;
/// The staged image's CRC-32 does not match the request.
pub const REFUSED_CRC: u32 = 5;

impl Record {
    const fn parts(self) -> (u8, u32) {
        match self {
            Record::ImageSize(v) => (1, v),
            Record::ImageCrc(v) => (2, v),
            Record::Swap(v) => (3, v),
            Record::Swapped => (4, 0),
            Record::Revert(v) => (5, v),
            Record::Confirmed => (6, 0),
            Record::Refused(v) => (7, v),
        }
    }

    /// The double-word that stores this record: the low word holds the
    /// mark, the kind and a 16-bit check of kind and argument; the high word
    /// holds the argument.
    pub fn encode(self) -> u64 {
        let (kind, arg) = self.parts();
        let low = (MARK << 24) | ((kind as u32) << 16) | check(kind, arg) as u32;
        ((arg as u64) << 32) | low as u64
    }

    /// The record a double-word stores, if it is one.
    pub fn decode(dw: u64) -> Option<Record> {
        let low = dw as u32;
        let arg = (dw >> 32) as u32;
        let kind = (low >> 16) as u8;
        if low >> 24 != MARK || low as u16 != check(kind, arg) {
            return None;
        }
        Some(match kind {
            1 => Record::ImageSize(arg),
            2 => Record::ImageCrc(arg),
            3 => Record::Swap(arg),
            4 if arg == 0 => Record::Swapped,
            5 => Record::Revert(arg),
            6 if arg == 0 => Record::Confirmed,
            7 => Record::Refused(arg),
            _ => return None,
        })
    }
}

/// CRC-16/CCITT-FALSE over the kind byte and the argument, little-endian.
/// It catches every single-bit error, and a torn write passes it about once
/// in 65536.
fn check(kind: u8, arg: u32) -> u16 {
    let a = arg.to_le_bytes();
    let mut crc: u16 = 0xFFFF;
    for b in [kind, a[0], a[1], a[2], a[3]] {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// The image a swap request stages.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Image {
    /// Size in bytes.
    pub size: u32,
    /// CRC-32 of those bytes.
    pub crc: u32,
}

/// What the log says to do.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Nothing pending: boot the active slot.
    Idle,
    /// Swap the staged image in, `steps` copies already done.
    Swap {
        /// Steps done.
        steps: u32,
        /// The staged image, from the records before `Swap(0)`.
        image: Option<Image>,
    },
    /// The swapped-in image has not confirmed itself.  Seen at boot, it reset
    /// before confirming, so the old image goes back.
    Trial,
    /// Copy the old image back, `pages` already done.
    Revert {
        /// Pages done.
        pages: u32,
    },
}

/// What the bootloader did on this boot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Nothing was pending.
    Idle,
    /// It swapped a staged image in; the image now runs on trial.
    Trial,
    /// It refused the staged image (a `REFUSED_*` reason) and kept the
    /// running one.
    Refused(u32),
    /// A trial image reset before confirming; the old image is back.
    Reverted,
}

/// The log as read from flash, with where the next record goes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Log {
    state: State,
    size: Option<u32>,
    crc: Option<u32>,
    next: usize,
}

impl Log {
    const EMPTY: Log = Log {
        state: State::Idle,
        size: None,
        crc: None,
        next: 0,
    };

    /// Read the log.  Every slot is read: a torn write can leave a slot that
    /// reads as erased in front of later records.
    pub fn scan(f: &mut impl Flash, g: &Geometry) -> Log {
        let mut log = Log::EMPTY;
        for slot in 0..g.log_slots {
            match f.read(g.state_page, slot) {
                Ok(ERASED) => {}
                Ok(dw) => {
                    log.next = slot + 1;
                    if let Some(r) = Record::decode(dw) {
                        log.apply(r);
                    }
                }
                Err(Unreadable) => log.next = slot + 1,
            }
        }
        log
    }

    /// The current state.
    pub fn state(&self) -> State {
        self.state
    }

    /// Record slots used, torn ones included.
    pub fn used(&self) -> usize {
        self.next
    }

    fn apply(&mut self, r: Record) {
        match r {
            Record::ImageSize(size) => {
                self.size = Some(size);
                self.crc = None;
                self.state = State::Idle;
            }
            Record::ImageCrc(crc) => {
                self.crc = Some(crc);
                self.state = State::Idle;
            }
            Record::Swap(steps) => {
                let image = match (self.size, self.crc) {
                    (Some(size), Some(crc)) => Some(Image { size, crc }),
                    _ => None,
                };
                self.state = State::Swap { steps, image };
            }
            Record::Swapped => self.state = State::Trial,
            Record::Revert(pages) => self.state = State::Revert { pages },
            Record::Confirmed | Record::Refused(_) => self.state = State::Idle,
        }
    }

    /// Program `r` into the next erased slot and check it reads back; a slot
    /// that does not take the record is skipped.
    pub fn append(&mut self, f: &mut impl Flash, g: &Geometry, r: Record) -> Result<(), Fault> {
        let dw = r.encode();
        while self.next < g.log_slots {
            let slot = self.next;
            self.next += 1;
            if f.read(g.state_page, slot) != Ok(ERASED) {
                continue;
            }
            match f.program(g.state_page, slot, dw) {
                Ok(()) => {}
                Err(Fault::Halted) => return Err(Fault::Halted),
                Err(_) => continue,
            }
            if f.read(g.state_page, slot) == Ok(dw) {
                self.apply(r);
                return Ok(());
            }
        }
        Err(Fault::Full)
    }

    /// [`append`](Self::append), and on a full page erase it and keep only
    /// `r`.  That opens the erase window the append-only format exists to
    /// avoid, but a page fills only if more than two dozen writes failed: a
    /// swap and a revert together use 229 of its 256 slots.
    fn append_or_compact(
        &mut self,
        f: &mut impl Flash,
        g: &Geometry,
        r: Record,
    ) -> Result<(), Fault> {
        match self.append(f, g, r) {
            Err(Fault::Full) => {
                f.erase(g.state_page)?;
                *self = Log::EMPTY;
                self.append(f, g, r)
            }
            other => other,
        }
    }

    /// The app's request to swap in the image it staged in the DFU slot.
    /// Only while nothing is pending: it erases the page first.
    pub fn request_swap(
        &mut self,
        f: &mut impl Flash,
        g: &Geometry,
        image: Image,
    ) -> Result<(), Fault> {
        if self.state != State::Idle {
            return Err(Fault::Failed);
        }
        f.erase(g.state_page)?;
        *self = Log::EMPTY;
        self.append(f, g, Record::ImageSize(image.size))?;
        self.append(f, g, Record::ImageCrc(image.crc))?;
        self.append(f, g, Record::Swap(0))
    }

    /// The app's confirmation that a trial image works; does nothing in any
    /// other state.
    pub fn confirm(&mut self, f: &mut impl Flash, g: &Geometry) -> Result<(), Fault> {
        if self.state == State::Trial {
            self.append_or_compact(f, g, Record::Confirmed)?;
        }
        Ok(())
    }

    /// Carry out what the log asks for: the bootloader calls this on every
    /// boot.  On `Err` the work stopped part way and the log says where;
    /// running again (after a reset) picks it up.
    pub fn run(&mut self, f: &mut impl Flash, g: &Geometry) -> Result<Outcome, Fault> {
        match self.state {
            State::Idle => Ok(Outcome::Idle),
            State::Swap { steps: 0, image } => {
                if let Err(reason) = verify(f, g, image) {
                    self.append_or_compact(f, g, Record::Refused(reason))?;
                    return Ok(Outcome::Refused(reason));
                }
                self.swap(f, g, 0)?;
                Ok(Outcome::Trial)
            }
            State::Swap { steps, .. } => {
                self.swap(f, g, steps)?;
                Ok(Outcome::Trial)
            }
            State::Trial => {
                self.revert(f, g, 0)?;
                Ok(Outcome::Reverted)
            }
            State::Revert { pages } => {
                self.revert(f, g, pages)?;
                Ok(Outcome::Reverted)
            }
        }
    }

    fn swap(&mut self, f: &mut impl Flash, g: &Geometry, from: u32) -> Result<(), Fault> {
        for step in from..g.swap_steps() {
            let page = step / 3;
            let (src, dst) = match step % 3 {
                0 => (g.active_page + page, g.scratch_page),
                1 => (g.dfu_page + page, g.active_page + page),
                _ => (g.scratch_page, g.dfu_page + page),
            };
            f.tick();
            f.copy(src, dst)?;
            self.append_or_compact(f, g, Record::Swap(step + 1))?;
        }
        self.append_or_compact(f, g, Record::Swapped)
    }

    fn revert(&mut self, f: &mut impl Flash, g: &Geometry, from: u32) -> Result<(), Fault> {
        for page in from..g.slot_pages {
            f.tick();
            f.copy(g.dfu_page + page, g.active_page + page)?;
            self.append_or_compact(f, g, Record::Revert(page + 1))?;
        }
        self.append_or_compact(f, g, Record::Confirmed)
    }
}

/// Check a staged image before the first swap step: its size, its vector
/// table, and its CRC-32.  `Err` carries a `REFUSED_*` reason.
pub fn verify(f: &mut impl Flash, g: &Geometry, image: Option<Image>) -> Result<(), u32> {
    let image = image.ok_or(REFUSED_NO_IMAGE)?;
    if image.size < 8 || image.size > g.slot_bytes() {
        return Err(REFUSED_SIZE);
    }
    let first = f.read(g.dfu_page, 0).map_err(|_| REFUSED_UNREADABLE)?;
    if !vector_table_ok(first) {
        return Err(REFUSED_VECTORS);
    }
    let mut crc = Crc32::new();
    let mut left = image.size as usize;
    let mut page = g.dfu_page;
    while left > 0 {
        f.tick();
        for index in 0..g.page_dws {
            if left == 0 {
                break;
            }
            let dw = f.read(page, index).map_err(|_| REFUSED_UNREADABLE)?;
            let n = left.min(8);
            crc.update(&dw.to_le_bytes()[..n]);
            left -= n;
        }
        page += 1;
    }
    if crc.finish() == image.crc {
        Ok(())
    } else {
        Err(REFUSED_CRC)
    }
}

/// Whether a slot's first double-word looks like an app vector table: an
/// initial stack pointer inside SRAM and a Thumb reset vector inside the
/// active slot, where every app is linked.
pub fn vector_table_ok(first: u64) -> bool {
    let sp = first as u32;
    let reset = (first >> 32) as u32;
    let entry = reset & !1;
    let sp_ok = sp > RAM_BASE && sp <= RAM_BASE + RAM_SIZE && sp % 4 == 0;
    let reset_ok = reset & 1 == 1 && entry >= ACTIVE_BASE && entry < ACTIVE_BASE + MAX_FW_SIZE;
    sp_ok && reset_ok
}

/// The word the bootloader leaves in a TAMP backup register before it jumps
/// to the app, saying what it did on this boot.  Backup registers keep their
/// value through a system reset (RM0461 31.6.10).  The app clears the word
/// when it reads it, so an app that finds none was started some other way:
/// by probe-rs, which sets the program counter itself.
pub mod handoff {
    use super::Outcome;

    /// TAMP_BKPxR index of the word.
    pub const REGISTER: usize = 0;

    const MAGIC: u32 = 0xB007;

    /// The word for an outcome.
    pub fn encode(outcome: Outcome) -> u32 {
        let (code, reason) = match outcome {
            Outcome::Idle => (1, 0),
            Outcome::Trial => (2, 0),
            Outcome::Refused(r) => (3, r.min(0xFF)),
            Outcome::Reverted => (4, 0),
        };
        (MAGIC << 16) | (code << 8) | reason
    }

    /// The outcome a word holds, if it is a handoff word.
    pub fn decode(word: u32) -> Option<Outcome> {
        if word >> 16 != MAGIC {
            return None;
        }
        let reason = word & 0xFF;
        match (word >> 8) & 0xFF {
            1 => Some(Outcome::Idle),
            2 => Some(Outcome::Trial),
            3 => Some(Outcome::Refused(reason)),
            4 => Some(Outcome::Reverted),
            _ => None,
        }
    }
}
