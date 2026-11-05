//! Bootloader for the Wio-E5 (STM32WLE5JC): carries out what the boot-state
//! log asks for, then starts the app in the active slot.
//!
//! The log, the swap and the revert are `flash_layout::boot`; this file is
//! the chip's side of them: the flash, the watchdog, and the jump.
//!
//! - When a swap or a revert is pending, the independent watchdog (IWDG) is
//!   started before the flash is touched and fed before each page: a hang
//!   anywhere in the work resets the chip, and the log lets the next boot
//!   carry on at the step that hung.
//! - The watchdog is always started before the jump, with the app's own
//!   5 s timeout.  An app that dies in its first instructions, before its
//!   own `watchdog::start`, is reset too, and while it is on trial the next
//!   boot reverts it.
//! - An active slot without a vector table is never jumped to.  The
//!   bootloader then waits for a debugger, with the watchdog off whenever no
//!   work started it this boot.
//! - The flash ECC raises an NMI on a two-bit error (RM0461 3.3.3), which a
//!   power cut during a write can leave behind.  The handler here records
//!   it, and the read reports the double-word as unreadable instead of the
//!   chip spinning in the default handler.
//! - Before the jump it leaves the app a TAMP backup register word saying
//!   what it did this boot (`flash_layout::boot::handoff`).

#![no_std]
#![no_main]

use panic_halt as _;

use core::ptr;
use core::sync::atomic::{AtomicBool, Ordering};

use cortex_m_rt::{entry, exception};
use flash_layout::boot::{self, Fault, Flash, Log, Outcome, State, Unreadable, ERASED, GEOMETRY};
use flash_layout::{ACTIVE_BASE, FLASH_BASE, PAGE_SIZE};

// -- Registers (RM0461) ------------------------------------------------------

const FLASH_REGS: u32 = 0x5800_4000;
const FLASH_KEYR: *mut u32 = (FLASH_REGS + 0x08) as *mut u32;
const FLASH_SR: *mut u32 = (FLASH_REGS + 0x10) as *mut u32;
const FLASH_CR: *mut u32 = (FLASH_REGS + 0x14) as *mut u32;
const FLASH_ECCR: *mut u32 = (FLASH_REGS + 0x18) as *mut u32;

const SR_EOP: u32 = 1 << 0;
/// OPERR, PROGERR, WRPERR, PGAERR, SIZERR, PGSERR, MISSERR, FASTERR, RDERR
/// and OPTVERR; each is cleared by writing 1.
const SR_ERRORS: u32 = (1 << 1) | (0x7F << 3) | (1 << 14) | (1 << 15);
const SR_BSY: u32 = 1 << 16;

const CR_PG: u32 = 1 << 0;
const CR_PER: u32 = 1 << 1;
/// PNB, the page to erase, sits in bits 9:3.
const CR_PNB_SHIFT: u32 = 3;
const CR_STRT: u32 = 1 << 16;
const CR_LOCK: u32 = 1 << 31;

const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;

const ECCR_ECCCIE: u32 = 1 << 24;
const ECCR_ECCC: u32 = 1 << 30;
const ECCR_ECCD: u32 = 1 << 31;

const IWDG_KR: *mut u32 = 0x4000_3000 as *mut u32;
const IWDG_PR: *mut u32 = 0x4000_3004 as *mut u32;
const IWDG_RLR: *mut u32 = 0x4000_3008 as *mut u32;
const IWDG_SR: *mut u32 = 0x4000_300C as *mut u32;

/// RCC_APB1ENR1: RTCAPBEN (bit 10) clocks the TAMP registers.
const RCC_APB1ENR1: *mut u32 = 0x5800_0058 as *mut u32;
/// PWR_CR1: DBP (bit 8) lifts the backup domain's write protection.
const PWR_CR1: *mut u32 = 0x5800_0400 as *mut u32;
/// TAMP_BKP0R; the backup registers follow it a word apart.
const TAMP_BKP0R: *mut u32 = 0x4000_B100 as *mut u32;

// -- Flash -------------------------------------------------------------------

fn reg_read(r: *mut u32) -> u32 {
    unsafe { ptr::read_volatile(r) }
}

fn reg_write(r: *mut u32, v: u32) {
    unsafe { ptr::write_volatile(r, v) }
}

fn flash_unlock() {
    if reg_read(FLASH_CR) & CR_LOCK != 0 {
        reg_write(FLASH_KEYR, FLASH_KEY1);
        reg_write(FLASH_KEYR, FLASH_KEY2);
    }
}

fn flash_lock() {
    reg_write(FLASH_CR, reg_read(FLASH_CR) | CR_LOCK);
}

fn wait_idle() {
    while reg_read(FLASH_SR) & SR_BSY != 0 {}
}

/// Steps 1-3 of RM0461's erase and program sequences: no operation under
/// way, and no error flags left from an earlier one (they would set PGSERR).
fn begin_op() {
    wait_idle();
    reg_write(FLASH_SR, SR_ERRORS | SR_EOP);
}

fn end_op() -> Result<(), Fault> {
    wait_idle();
    reg_write(FLASH_CR, 0);
    let sr = reg_read(FLASH_SR);
    reg_write(FLASH_SR, sr & (SR_ERRORS | SR_EOP));
    if sr & SR_ERRORS != 0 {
        Err(Fault::Failed)
    } else {
        Ok(())
    }
}

fn erase_page(page: u32) -> Result<(), Fault> {
    begin_op();
    reg_write(FLASH_CR, CR_PER | (page << CR_PNB_SHIFT));
    reg_write(FLASH_CR, CR_PER | (page << CR_PNB_SHIFT) | CR_STRT);
    end_op()
}

fn program_dw(addr: u32, value: u64) -> Result<(), Fault> {
    begin_op();
    reg_write(FLASH_CR, CR_PG);
    // Two consecutive word writes, low word first, launch the programming.
    unsafe {
        ptr::write_volatile(addr as *mut u32, value as u32);
        ptr::write_volatile((addr + 4) as *mut u32, (value >> 32) as u32);
    }
    end_op()
}

/// Set by the NMI handler when the flash ECC found a two-bit error.
static ECC_ERROR: AtomicBool = AtomicBool::new(false);

// cortex-m-rt makes NMI handlers unsafe: an NMI preempts critical sections.
// This one touches only the flash ECC register and an atomic.
#[exception]
unsafe fn NonMaskableInt() {
    let eccr = reg_read(FLASH_ECCR);
    // ECCC counts too: with it latched by an earlier corrected error the
    // register may not show the two-bit error that raised this NMI.
    if eccr & (ECCR_ECCD | ECCR_ECCC) != 0 {
        ECC_ERROR.store(true, Ordering::SeqCst);
        clear_ecc_flags();
        return;
    }
    // Nothing else raises an NMI here (no clock security system, no SRAM2
    // parity check); spin, and the watchdog resets the chip if it runs.
    loop {}
}

/// Clear ECCC and ECCD (write 1), keeping the interrupt enable.  While either
/// is set, FLASH_ECCR records no new error (RM0461 3.3.3), so a corrected
/// single-bit error left from an earlier read would keep a later two-bit
/// error from setting ECCD.
fn clear_ecc_flags() {
    let eccr = reg_read(FLASH_ECCR);
    if eccr & (ECCR_ECCD | ECCR_ECCC) != 0 {
        reg_write(FLASH_ECCR, (eccr & ECCR_ECCCIE) | ECCR_ECCD | ECCR_ECCC);
    }
}

/// Read a double-word as stored, and whether the ECC rejected it.  The NMI
/// is taken before the barrier completes, so the flag is current.
fn read_dw(addr: u32) -> (u64, bool) {
    clear_ecc_flags();
    ECC_ERROR.store(false, Ordering::SeqCst);
    let (lo, hi) = unsafe {
        (
            ptr::read_volatile(addr as *const u32),
            ptr::read_volatile((addr + 4) as *const u32),
        )
    };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
    (
        ((hi as u64) << 32) | lo as u64,
        ECC_ERROR.load(Ordering::SeqCst),
    )
}

fn dw_addr(page: u32, index: usize) -> u32 {
    FLASH_BASE + page * PAGE_SIZE + index as u32 * 8
}

/// The chip's flash, for `flash_layout::boot`.
struct Chip;

impl Flash for Chip {
    fn read(&mut self, page: u32, index: usize) -> Result<u64, Unreadable> {
        match read_dw(dw_addr(page, index)) {
            (_, true) => Err(Unreadable),
            (v, false) => Ok(v),
        }
    }

    fn program(&mut self, page: u32, index: usize, value: u64) -> Result<(), Fault> {
        program_dw(dw_addr(page, index), value)
    }

    fn erase(&mut self, page: u32) -> Result<(), Fault> {
        erase_page(page)
    }

    fn copy(&mut self, src: u32, dst: u32) -> Result<(), Fault> {
        const DWS: usize = (PAGE_SIZE / 8) as usize;
        let mut buf = [ERASED; DWS];
        for (i, dw) in buf.iter_mut().enumerate() {
            *dw = read_dw(dw_addr(src, i)).0;
        }
        // A few attempts: erase, program, and read every double-word back.
        for _ in 0..3 {
            if erase_page(dst).is_err() {
                continue;
            }
            let programmed = buf
                .iter()
                .enumerate()
                .all(|(i, &v)| v == ERASED || program_dw(dw_addr(dst, i), v).is_ok());
            if programmed
                && buf
                    .iter()
                    .enumerate()
                    .all(|(i, &v)| read_dw(dw_addr(dst, i)) == (v, false))
            {
                return Ok(());
            }
        }
        Err(Fault::Failed)
    }

    fn tick(&mut self) {
        iwdg_feed();
    }
}

// -- Watchdog ----------------------------------------------------------------

/// Start the IWDG with a 5 s timeout, or set that timeout on one already
/// running: LSI of about 32 kHz (29.5-34 kHz over temperature, DS13105
/// Table 58) divided by 64 (PR = 4), counting down from 2500.  The app's
/// `watchdog::start` programs the same.
fn iwdg_start() {
    reg_write(IWDG_KR, 0xCCCC);
    reg_write(IWDG_KR, 0x5555);
    reg_write(IWDG_PR, 4);
    reg_write(IWDG_RLR, 2500);
    // PVU and RVU only: WVU belongs to the window register, never written.
    while reg_read(IWDG_SR) & 0x3 != 0 {}
    reg_write(IWDG_KR, 0xAAAA);
}

fn iwdg_feed() {
    reg_write(IWDG_KR, 0xAAAA);
}

// -- Handoff and jump ----------------------------------------------------------

fn leave_handoff(outcome: Outcome) {
    reg_write(RCC_APB1ENR1, reg_read(RCC_APB1ENR1) | (1 << 10));
    // Read back: the clock must be running before TAMP is written.
    let _ = reg_read(RCC_APB1ENR1);
    reg_write(PWR_CR1, reg_read(PWR_CR1) | (1 << 8));
    let bkp = unsafe { TAMP_BKP0R.add(boot::handoff::REGISTER) };
    reg_write(bkp, boot::handoff::encode(outcome));
}

/// Set VTOR and jump to the application in the ACTIVE partition.
///
/// # Safety
/// The ACTIVE partition must contain a valid vector table.
unsafe fn jump_to_app() -> ! {
    let vtor = ACTIVE_BASE;
    let sp = ptr::read_volatile(vtor as *const u32);
    let reset = ptr::read_volatile((vtor + 4) as *const u32);

    // Set VTOR to application vector table
    const SCB_VTOR: *mut u32 = 0xE000_ED08 as *mut u32;
    ptr::write_volatile(SCB_VTOR, vtor);

    // Barriers
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Set MSP and jump via inline assembly
    core::arch::asm!(
        "msr MSP, {sp}",
        "bx {entry}",
        sp = in(reg) sp,
        entry = in(reg) reset,
        options(noreturn),
    )
}

// -- Entry point ---------------------------------------------------------------

#[entry]
fn main() -> ! {
    flash_unlock();

    let g = &GEOMETRY;
    let mut chip = Chip;
    let mut log = Log::scan(&mut chip, g);
    let work = log.state() != State::Idle;
    if work {
        iwdg_start();
    }
    let outcome = match log.run(&mut chip, g) {
        Ok(outcome) => outcome,
        // The work stopped part way and the log says where.  The watchdog,
        // started for the work, resets the chip and the next boot resumes.
        Err(_) => loop {},
    };

    flash_lock();

    let (first, unreadable) = read_dw(ACTIVE_BASE);
    if unreadable || !boot::vector_table_ok(first) {
        // Nothing to run.  If work started the watchdog it resets the chip
        // once; the next boot has nothing pending and waits here with the
        // watchdog off, so a probe can attach.
        loop {
            cortex_m::asm::nop();
        }
    }

    iwdg_start();
    leave_handoff(outcome);
    unsafe { jump_to_app() }
}
