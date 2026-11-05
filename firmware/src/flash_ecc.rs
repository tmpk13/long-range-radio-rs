//! Flash reads that survive an uncorrectable ECC error.
//!
//! The flash stores eight ECC bits per double-word and raises an NMI on a
//! two-bit error (RM0461 3.3.3).  A power cut while a double-word or a page
//! was being written can leave one behind (RM0461 3.3.6): in the boot-state
//! log, in a half-written page of a staged image.  cortex-m-rt's default NMI
//! handler spins, so without this module reading such a double-word would
//! hang the app until the watchdog fired, at the same place on every boot.
//!
//! The handler below records the error and returns; [`read_dw`] checks the
//! record after its read.  Any other NMI source (none is enabled: no clock
//! security system, no SRAM2 parity check) still spins.

use core::sync::atomic::{AtomicBool, Ordering};

pub use flash_layout::boot::Unreadable;

/// Set by the NMI handler on a two-bit ECC error.
static ECC_ERROR: AtomicBool = AtomicBool::new(false);

/// Read the double-word at `addr` (8-byte aligned).
pub fn read_dw(addr: u32) -> Result<u64, Unreadable> {
    let (v, bad) = read_dw_raw(addr);
    if bad { Err(Unreadable) } else { Ok(v) }
}

/// Read the double-word at `addr` as stored, and whether the ECC rejected it.
pub fn read_dw_raw(addr: u32) -> (u64, bool) {
    ECC_ERROR.store(false, Ordering::SeqCst);
    // SAFETY: an aligned read of flash; a bad ECC raises the NMI handled below.
    let (lo, hi) = unsafe {
        (
            core::ptr::read_volatile(addr as *const u32),
            core::ptr::read_volatile((addr + 4) as *const u32),
        )
    };
    // The NMI is taken before the barrier completes, so the flag is current.
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
    (
        ((hi as u64) << 32) | lo as u64,
        ECC_ERROR.load(Ordering::SeqCst),
    )
}

// cortex-m-rt makes NMI handlers unsafe: an NMI preempts critical sections.
// This one touches only the flash ECC register and an atomic.
#[cfg(all(target_arch = "arm", target_os = "none"))]
#[cortex_m_rt::exception]
unsafe fn NonMaskableInt() {
    // SAFETY: FLASH_ECCR is only read and its write-1-to-clear flags written.
    let eccr = unsafe { &(*stm32wlxx_hal::pac::FLASH::PTR).eccr };
    if eccr.read().eccd().bit_is_set() {
        ECC_ERROR.store(true, Ordering::SeqCst);
        eccr.modify(|_, w| w.eccd().set_bit().eccc().set_bit());
        return;
    }
    loop {}
}
