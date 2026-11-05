extern crate std;

use super::*;
use crate::crc32::Crc32;
use std::collections::HashSet;
use std::format;
use std::vec;
use std::vec::Vec;

// -- A flash that loses power on demand --------------------------------------

/// What a power cut leaves in the double-word or page it interrupts.
/// RM0461 3.3.6 only says the contents are not guaranteed, so every
/// possibility is tried.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Torn {
    /// The operation had not started.
    Before,
    /// The operation finished; power went right after it.
    After,
    /// Some bits changed: garbage that reads without an ECC error.
    Garbage,
    /// A two-bit ECC error: reads raise the NMI.
    Unreadable,
}

const TORN: [Torn; 4] = [Torn::Before, Torn::After, Torn::Garbage, Torn::Unreadable];

struct Sim {
    pages: Vec<Vec<u64>>,
    /// Double-words with an uncorrectable ECC error.
    bad: HashSet<(u32, usize)>,
    /// Double-words that read erased but fail to program, like a cell a torn
    /// write left marginal.
    refuse: HashSet<(u32, usize)>,
    /// Erases and programs left before the power fails; `None`: it does not.
    fuel: Option<u64>,
    torn: Torn,
    dead: bool,
    /// Erases and programs done so far.
    ops: u64,
}

impl Sim {
    fn new(g: &Geometry) -> Sim {
        let pages = (0..=g.state_page)
            .map(|p| {
                let n = if p == g.state_page {
                    g.log_slots
                } else {
                    g.page_dws
                };
                vec![ERASED; n]
            })
            .collect();
        Sim {
            pages,
            bad: HashSet::new(),
            refuse: HashSet::new(),
            fuel: None,
            torn: Torn::Before,
            dead: false,
            ops: 0,
        }
    }

    /// Count one erase or program; `Some` when the power fails on it.
    fn spend(&mut self) -> Option<Torn> {
        if let Some(left) = self.fuel {
            if left == 0 {
                self.fuel = None;
                self.dead = true;
                return Some(self.torn);
            }
            self.fuel = Some(left - 1);
        }
        self.ops += 1;
        None
    }

    fn slot(&self, g: &Geometry, first_page: u32) -> Vec<u64> {
        (first_page..first_page + g.slot_pages)
            .flat_map(|p| self.pages[p as usize].iter().copied())
            .collect()
    }

    fn load_slot(&mut self, g: &Geometry, first_page: u32, image: &[u64]) {
        for (i, chunk) in image.chunks(g.page_dws).enumerate() {
            self.pages[first_page as usize + i].copy_from_slice(chunk);
        }
    }
}

impl Flash for Sim {
    fn read(&mut self, page: u32, index: usize) -> Result<u64, Unreadable> {
        if self.bad.contains(&(page, index)) {
            Err(Unreadable)
        } else {
            Ok(self.pages[page as usize][index])
        }
    }

    fn program(&mut self, page: u32, index: usize, value: u64) -> Result<(), Fault> {
        if self.dead {
            return Err(Fault::Halted);
        }
        let old = self.pages[page as usize][index];
        if let Some(torn) = self.spend() {
            let cell = &mut self.pages[page as usize][index];
            match torn {
                Torn::Before => {}
                Torn::After => *cell = old & value,
                // Some of the bits the write would clear are still set.
                Torn::Garbage => *cell = old & (value | (!value & 0x5555_5555_5555_5555)),
                Torn::Unreadable => {
                    self.bad.insert((page, index));
                }
            }
            return Err(Fault::Halted);
        }
        // The flash refuses to program a double-word that is not erased.
        if old != ERASED
            || self.bad.contains(&(page, index))
            || self.refuse.contains(&(page, index))
        {
            return Err(Fault::Failed);
        }
        self.pages[page as usize][index] = value;
        Ok(())
    }

    fn erase(&mut self, page: u32) -> Result<(), Fault> {
        if self.dead {
            return Err(Fault::Halted);
        }
        if let Some(torn) = self.spend() {
            let n = self.pages[page as usize].len();
            for i in 0..n {
                let cell = &mut self.pages[page as usize][i];
                if *cell == ERASED {
                    continue;
                }
                match torn {
                    Torn::Before => {}
                    Torn::After => {
                        *cell = ERASED;
                        self.bad.remove(&(page, i));
                    }
                    // Some of the cleared bits are set again.
                    Torn::Garbage => *cell |= !*cell & 0xAAAA_AAAA_AAAA_AAAA,
                    Torn::Unreadable => {
                        self.bad.insert((page, i));
                    }
                }
            }
            return Err(Fault::Halted);
        }
        for cell in self.pages[page as usize].iter_mut() {
            *cell = ERASED;
        }
        self.bad.retain(|&(p, _)| p != page);
        Ok(())
    }

    // The bootloader's copy: read the source as stored (ECC errors ignored),
    // erase the destination, program it, verify.
    fn copy(&mut self, src: u32, dst: u32) -> Result<(), Fault> {
        let buf = self.pages[src as usize].clone();
        self.erase(dst)?;
        for (i, &v) in buf.iter().enumerate() {
            if v != ERASED {
                self.program(dst, i, v)?;
            }
        }
        for (i, &v) in buf.iter().enumerate() {
            if self.read(dst, i) != Ok(v) {
                return Err(Fault::Failed);
            }
        }
        Ok(())
    }
}

// -- Fixtures -----------------------------------------------------------------

/// Three pages of four double-words per slot: small enough to cut the power
/// at every operation, twice over.
const TINY: Geometry = Geometry {
    page_dws: 4,
    slot_pages: 3,
    active_page: 0,
    dfu_page: 3,
    scratch_page: 6,
    state_page: 7,
    log_slots: 32,
};

/// A slot image with a plausible vector table, some erased double-words
/// (an unused tail looks like that) and content unique to `seed`.
fn image(g: &Geometry, seed: u64) -> Vec<u64> {
    let n = g.slot_pages as usize * g.page_dws;
    let mut s = seed | 1;
    let mut v: Vec<u64> = (0..n)
        .map(|_| {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            s
        })
        .collect();
    let sp = (RAM_BASE + RAM_SIZE) as u64;
    let reset = (ACTIVE_BASE + 0x1C1) as u64;
    v[0] = (reset << 32) | sp;
    v[n - 2] = ERASED;
    v
}

fn crc_of(image: &[u64], size: u32) -> u32 {
    let bytes: Vec<u8> = image.iter().flat_map(|dw| dw.to_le_bytes()).collect();
    let mut c = Crc32::new();
    c.update(&bytes[..size as usize]);
    c.finish()
}

struct Case {
    g: Geometry,
    old: Vec<u64>,
    new: Vec<u64>,
    image: Image,
}

impl Case {
    fn new(g: Geometry) -> Case {
        let old = image(&g, 0x0123_4567_89AB_CDEF);
        let new = image(&g, 0xFEDC_BA98_7654_3210);
        // A size that ends inside a double-word.
        let size = g.slot_bytes() - 13;
        let crc = crc_of(&new, size);
        Case {
            g,
            old,
            new,
            image: Image { size, crc },
        }
    }

    /// The old image running, the new one staged, and the log of an earlier
    /// update that was confirmed, so the request's erase has records to tear.
    fn sim(&self) -> Sim {
        let g = &self.g;
        let mut sim = Sim::new(g);
        sim.load_slot(g, g.active_page, &self.old);
        sim.load_slot(g, g.dfu_page, &self.new);
        sim.pages[g.scratch_page as usize].fill(0x0BAD_F00D_0BAD_F00D);
        let mut log = Log::scan(&mut sim, g);
        for r in [
            Record::ImageSize(64),
            Record::ImageCrc(0x1234_5678),
            Record::Swap(0),
            Record::Swap(1),
            Record::Swapped,
            Record::Confirmed,
        ] {
            log.append(&mut sim, g, r).unwrap();
        }
        sim
    }

    fn staged(&self) -> Sim {
        let mut sim = self.sim();
        let mut log = Log::scan(&mut sim, &self.g);
        log.request_swap(&mut sim, &self.g, self.image).unwrap();
        sim
    }

    fn assert_slots(&self, sim: &Sim, active: &[u64], dfu: &[u64], what: &str) {
        assert!(
            sim.slot(&self.g, self.g.active_page) == active,
            "{what}: active slot"
        );
        assert!(
            sim.slot(&self.g, self.g.dfu_page) == dfu,
            "{what}: DFU slot"
        );
    }

    /// After cut boots of a staged swap: the new image swapped in and on
    /// trial, or, when the power failed after the swap's last record but
    /// before the jump, swapped in and already reverted, since the trial
    /// cannot tell that its image never ran.  That revert may itself have
    /// been cut after its last record, leaving the final boot nothing to do.
    fn assert_swap_finished(&self, sim: &mut Sim, outcome: Outcome, what: &str) {
        match outcome {
            Outcome::Trial => {
                self.assert_slots(sim, &self.new, &self.old, what);
                assert_eq!(Log::scan(sim, &self.g).state(), State::Trial, "{what}");
            }
            Outcome::Reverted | Outcome::Idle => {
                self.assert_slots(sim, &self.old, &self.old, what);
                assert_eq!(Log::scan(sim, &self.g).state(), State::Idle, "{what}");
            }
            other => panic!("{what}: {other:?}"),
        }
    }
}

/// One boot of the bootloader's logic.
fn boot(sim: &mut Sim, g: &Geometry) -> Result<Outcome, Fault> {
    let mut log = Log::scan(sim, g);
    log.run(sim, g)
}

/// Boot with the power failing after `fuel` operations, once per cut, then
/// once more with steady power; the first boot that finishes gives the
/// outcome.
fn boot_through(sim: &mut Sim, g: &Geometry, cuts: &[(u64, Torn)]) -> Outcome {
    for &(fuel, torn) in cuts {
        sim.fuel = Some(fuel);
        sim.torn = torn;
        let result = boot(sim, g);
        sim.fuel = None;
        match result {
            Ok(outcome) => return outcome,
            Err(Fault::Halted) => sim.dead = false,
            Err(e) => panic!("boot failed with {e:?}"),
        }
    }
    boot(sim, g).expect("a boot with steady power finishes")
}

/// Erases and programs in one uncut boot.
fn ops_of_boot(sim: &mut Sim, g: &Geometry) -> u64 {
    let before = sim.ops;
    boot(sim, g).unwrap();
    sim.ops - before
}

// -- Records -----------------------------------------------------------------

const ALL: [Record; 9] = [
    Record::ImageSize(49_152),
    Record::ImageCrc(0xCBF4_3926),
    Record::Swap(0),
    Record::Swap(168),
    Record::Swapped,
    Record::Revert(56),
    Record::Confirmed,
    Record::Refused(REFUSED_CRC),
    Record::ImageSize(u32::MAX),
];

#[test]
fn records_round_trip() {
    for r in ALL {
        assert_eq!(Record::decode(r.encode()), Some(r));
    }
}

#[test]
fn no_single_bit_error_decodes() {
    for r in ALL {
        let dw = r.encode();
        for bit in 0..64 {
            assert_eq!(Record::decode(dw ^ (1 << bit)), None, "{r:?} bit {bit}");
        }
    }
}

#[test]
fn foreign_words_do_not_decode() {
    assert_eq!(Record::decode(ERASED), None);
    assert_eq!(Record::decode(0), None);
    // The first bootloader's state words, with an erased progress word and
    // with page progress in it.
    for word in [0x4F4B_4F4Bu64, 0x5357_4150, 0x444F_4E45, 0x5245_5654] {
        assert_eq!(Record::decode(0xFFFF_FFFF_0000_0000 | word), None);
        assert_eq!(Record::decode((7 << 32) | word), None);
    }
}

#[test]
fn torn_records_seldom_decode() {
    // Every way of leaving some of a record's cleared bits set, over the low
    // 16 cleared bits of each record: anything that decodes is the record
    // itself (the write had in fact finished) or nothing.
    for r in ALL {
        let dw = r.encode();
        let zeros: Vec<u32> = (0..64).filter(|b| dw & (1 << b) == 0).take(16).collect();
        let mut wrong = 0;
        for mask in 1u32..(1 << zeros.len()) {
            let mut torn = dw;
            for (i, b) in zeros.iter().enumerate() {
                if mask & (1 << i) != 0 {
                    torn |= 1 << b;
                }
            }
            if let Some(other) = Record::decode(torn) {
                assert_ne!(other, r);
                wrong += 1;
            }
        }
        assert!(wrong <= 4, "{r:?}: {wrong} torn forms decode");
    }
}

// -- The log -----------------------------------------------------------------

#[test]
fn scan_skips_torn_unreadable_and_erased_slots() {
    let g = TINY;
    let mut sim = Sim::new(&g);
    let s = g.state_page as usize;
    sim.pages[s][0] = Record::ImageSize(100).encode();
    sim.pages[s][1] = Record::ImageCrc(7).encode();
    sim.pages[s][2] = Record::Swap(0).encode() | 0x0F00; // torn
    sim.pages[s][3] = Record::Swap(0).encode();
    sim.bad.insert((g.state_page, 4));
    // Slot 5 reads erased: a torn write that never cleared a bit.
    sim.pages[s][6] = Record::Swap(4).encode();
    let log = Log::scan(&mut sim, &g);
    assert_eq!(
        log.state(),
        State::Swap {
            steps: 4,
            image: Some(Image { size: 100, crc: 7 })
        }
    );
    assert_eq!(log.used(), 7);
}

#[test]
fn append_moves_past_a_slot_that_does_not_take_the_record() {
    let g = TINY;
    let mut sim = Sim::new(&g);
    sim.pages[g.state_page as usize][0] = Record::Swapped.encode();
    // Reads erased, but the flash refuses to program it.
    sim.refuse.insert((g.state_page, 1));
    let mut log = Log::scan(&mut sim, &g);
    assert_eq!(log.state(), State::Trial);
    assert_eq!(log.used(), 1);
    log.append(&mut sim, &g, Record::Confirmed).unwrap();
    assert_eq!(log.used(), 3);
    assert_eq!(Log::scan(&mut sim, &g).state(), State::Idle);
}

#[test]
fn request_only_while_idle() {
    let case = Case::new(TINY);
    let mut sim = case.staged();
    let mut log = Log::scan(&mut sim, &case.g);
    assert_eq!(
        log.request_swap(&mut sim, &case.g, case.image),
        Err(Fault::Failed)
    );
}

#[test]
fn swap_then_confirm() {
    let case = Case::new(TINY);
    let g = &case.g;
    let mut sim = case.staged();
    assert_eq!(boot(&mut sim, g), Ok(Outcome::Trial));
    case.assert_slots(&sim, &case.new, &case.old, "after the swap");
    let mut log = Log::scan(&mut sim, g);
    assert_eq!(log.state(), State::Trial);
    log.confirm(&mut sim, g).unwrap();
    assert_eq!(Log::scan(&mut sim, g).state(), State::Idle);
    assert_eq!(boot(&mut sim, g), Ok(Outcome::Idle));
    case.assert_slots(&sim, &case.new, &case.old, "after the confirm");
}

#[test]
fn trial_that_resets_reverts() {
    let case = Case::new(TINY);
    let g = &case.g;
    let mut sim = case.staged();
    assert_eq!(boot(&mut sim, g), Ok(Outcome::Trial));
    assert_eq!(boot(&mut sim, g), Ok(Outcome::Reverted));
    case.assert_slots(&sim, &case.old, &case.old, "after the revert");
    assert_eq!(Log::scan(&mut sim, g).state(), State::Idle);
    assert_eq!(boot(&mut sim, g), Ok(Outcome::Idle));
}

#[test]
fn confirm_outside_a_trial_writes_nothing() {
    let case = Case::new(TINY);
    let mut sim = case.sim();
    let mut log = Log::scan(&mut sim, &case.g);
    let used = log.used();
    log.confirm(&mut sim, &case.g).unwrap();
    assert_eq!(Log::scan(&mut sim, &case.g).used(), used);
}

// -- Refusals ----------------------------------------------------------------

fn refused_with(case: &Case, mut sim: Sim) -> Outcome {
    let g = &case.g;
    let outcome = boot(&mut sim, g).unwrap();
    case.assert_slots(&sim, &case.old, &case.new, "a refused image moves nothing");
    assert_eq!(Log::scan(&mut sim, g).state(), State::Idle);
    assert_eq!(boot(&mut sim, g), Ok(Outcome::Idle));
    outcome
}

#[test]
fn refuses_a_wrong_crc() {
    let mut case = Case::new(TINY);
    case.image.crc ^= 1;
    let sim = case.staged();
    assert_eq!(refused_with(&case, sim), Outcome::Refused(REFUSED_CRC));
}

#[test]
fn refuses_a_bad_size() {
    for size in [0, 7, TINY.slot_bytes() + 1] {
        let mut case = Case::new(TINY);
        case.image.size = size;
        let sim = case.staged();
        assert_eq!(refused_with(&case, sim), Outcome::Refused(REFUSED_SIZE));
    }
}

#[test]
fn refuses_an_image_without_vectors() {
    let mut case = Case::new(TINY);
    case.new[0] = 0x0800_0001_2001_0000; // reset vector in the bootloader
    case.image.crc = crc_of(&case.new, case.image.size);
    let sim = case.staged();
    assert_eq!(refused_with(&case, sim), Outcome::Refused(REFUSED_VECTORS));
}

#[test]
fn refuses_an_unreadable_image() {
    let case = Case::new(TINY);
    let mut sim = case.staged();
    sim.bad.insert((case.g.dfu_page + 1, 2));
    let outcome = boot(&mut sim, &case.g).unwrap();
    assert_eq!(outcome, Outcome::Refused(REFUSED_UNREADABLE));
}

#[test]
fn refuses_a_request_without_an_image() {
    let case = Case::new(TINY);
    let g = &case.g;
    let mut sim = case.sim();
    sim.erase(g.state_page).unwrap();
    let mut log = Log::scan(&mut sim, g);
    log.append(&mut sim, g, Record::Swap(0)).unwrap();
    assert_eq!(refused_with(&case, sim), Outcome::Refused(REFUSED_NO_IMAGE));
}

#[test]
fn vector_tables() {
    let good = ((ACTIVE_BASE as u64 + 0x1C1) << 32) | (RAM_BASE + RAM_SIZE) as u64;
    assert!(vector_table_ok(good));
    assert!(!vector_table_ok(ERASED));
    // An even (ARM, not Thumb) reset vector.
    assert!(!vector_table_ok(good - (1 << 32)));
    // A stack pointer past the end of SRAM.
    assert!(!vector_table_ok(good + 4));
    // A reset vector past the active slot.
    assert!(!vector_table_ok(
        ((ACTIVE_BASE + MAX_FW_SIZE + 1) as u64) << 32 | (RAM_BASE + RAM_SIZE) as u64
    ));
}

// -- Power cuts ----------------------------------------------------------------

#[test]
fn swap_survives_any_single_cut() {
    let case = Case::new(TINY);
    let g = &case.g;
    let total = ops_of_boot(&mut case.staged(), g);
    let mut trials = 0;
    for fuel in 0..=total {
        for torn in TORN {
            let what = format!("cut at {fuel}, {torn:?}");
            let mut sim = case.staged();
            let outcome = boot_through(&mut sim, g, &[(fuel, torn)]);
            case.assert_swap_finished(&mut sim, outcome, &what);
            if outcome == Outcome::Trial {
                trials += 1;
                // The trial image resets before confirming.
                assert_eq!(boot(&mut sim, g), Ok(Outcome::Reverted), "{what}");
                case.assert_slots(&sim, &case.old, &case.old, &what);
            }
        }
    }
    // Only a cut right after the last record skips the trial.
    assert_eq!(trials, (total + 1) * 4 - 1);
}

#[test]
fn swap_survives_any_two_cuts() {
    let case = Case::new(TINY);
    let g = &case.g;
    let total = ops_of_boot(&mut case.staged(), g);
    for first in 0..total {
        for t1 in TORN {
            for second in 0..=total {
                for t2 in TORN {
                    let what = format!("cuts at {first} {t1:?}, {second} {t2:?}");
                    let mut sim = case.staged();
                    let outcome = boot_through(&mut sim, g, &[(first, t1), (second, t2)]);
                    case.assert_swap_finished(&mut sim, outcome, &what);
                }
            }
        }
    }
}

#[test]
fn revert_survives_any_two_cuts() {
    let case = Case::new(TINY);
    let g = &case.g;
    let trial = || {
        let mut sim = case.staged();
        assert_eq!(boot(&mut sim, g), Ok(Outcome::Trial));
        sim
    };
    let total = ops_of_boot(&mut trial(), g);
    for first in 0..=total {
        for t1 in TORN {
            for second in 0..=total {
                for t2 in TORN {
                    let what = format!("cuts at {first} {t1:?}, {second} {t2:?}");
                    let mut sim = trial();
                    let outcome = boot_through(&mut sim, g, &[(first, t1), (second, t2)]);
                    // Idle: a cut came right after the revert's last record,
                    // and the next boot found nothing left to do.
                    assert!(
                        matches!(outcome, Outcome::Reverted | Outcome::Idle),
                        "{what}: {outcome:?}"
                    );
                    case.assert_slots(&sim, &case.old, &case.old, &what);
                    assert_eq!(Log::scan(&mut sim, g).state(), State::Idle, "{what}");
                }
            }
        }
    }
}

#[test]
fn a_cut_request_swaps_whole_or_not_at_all() {
    let case = Case::new(TINY);
    let g = &case.g;
    let total = {
        let mut sim = case.sim();
        let mut log = Log::scan(&mut sim, g);
        let before = sim.ops;
        log.request_swap(&mut sim, g, case.image).unwrap();
        sim.ops - before
    };
    for fuel in 0..total {
        for torn in TORN {
            let what = format!("request cut at {fuel}, {torn:?}");
            let mut sim = case.sim();
            sim.fuel = Some(fuel);
            sim.torn = torn;
            let mut log = Log::scan(&mut sim, g);
            assert_eq!(
                log.request_swap(&mut sim, g, case.image),
                Err(Fault::Halted)
            );
            sim.fuel = None;
            sim.dead = false;
            match boot(&mut sim, g).unwrap() {
                Outcome::Idle => case.assert_slots(&sim, &case.old, &case.new, &what),
                Outcome::Trial => case.assert_slots(&sim, &case.new, &case.old, &what),
                other => panic!("{what}: {other:?}"),
            }
        }
    }
}

#[test]
fn a_cut_confirm_keeps_the_image_or_reverts_it() {
    let case = Case::new(TINY);
    let g = &case.g;
    for torn in TORN {
        let what = format!("confirm cut, {torn:?}");
        let mut sim = case.staged();
        assert_eq!(boot(&mut sim, g), Ok(Outcome::Trial));
        sim.fuel = Some(0);
        sim.torn = torn;
        let mut log = Log::scan(&mut sim, g);
        assert_eq!(log.confirm(&mut sim, g), Err(Fault::Halted));
        sim.dead = false;
        match boot(&mut sim, g).unwrap() {
            Outcome::Idle => case.assert_slots(&sim, &case.new, &case.old, &what),
            Outcome::Reverted => case.assert_slots(&sim, &case.old, &case.old, &what),
            other => panic!("{what}: {other:?}"),
        }
    }
}

// -- The real geometry ---------------------------------------------------------

#[test]
fn full_size_swap_and_revert_fit_the_log() {
    let case = Case::new(GEOMETRY);
    let g = &case.g;
    let mut sim = case.staged();
    assert_eq!(boot(&mut sim, g), Ok(Outcome::Trial));
    case.assert_slots(&sim, &case.new, &case.old, "full-size swap");
    assert_eq!(Log::scan(&mut sim, g).used(), 3 + 168 + 1);
    assert_eq!(boot(&mut sim, g), Ok(Outcome::Reverted));
    case.assert_slots(&sim, &case.old, &case.old, "full-size revert");
    let used = Log::scan(&mut sim, g).used();
    assert_eq!(used, 3 + 168 + 1 + 56 + 1);
    assert!(
        used + 24 <= g.log_slots,
        "too little room for failed writes"
    );
}

#[test]
fn full_size_swap_survives_cuts_across_it() {
    let case = Case::new(GEOMETRY);
    let g = &case.g;
    let total = ops_of_boot(&mut case.staged(), g);
    // Every 997th operation: around a hundred cuts spread over the swap.
    for fuel in (0..total).step_by(997) {
        for torn in TORN {
            let what = format!("cut at {fuel}, {torn:?}");
            let mut sim = case.staged();
            let outcome = boot_through(&mut sim, g, &[(fuel, torn)]);
            case.assert_swap_finished(&mut sim, outcome, &what);
        }
    }
}

// -- The handoff word -------------------------------------------------------------

#[test]
fn handoff_round_trips() {
    for o in [
        Outcome::Idle,
        Outcome::Trial,
        Outcome::Refused(REFUSED_CRC),
        Outcome::Reverted,
    ] {
        assert_eq!(handoff::decode(handoff::encode(o)), Some(o));
    }
    assert_eq!(handoff::decode(0), None);
    assert_eq!(handoff::decode(0xFFFF_FFFF), None);
}
