//! Independent watchdog (IWDG): resets the chip when the main loop stops.
//!
//! The IWDG runs from the LSI oscillator, independent of the system clock,
//! and once started it cannot be stopped until the next reset.
//!
//! - The bootloader starts it before jumping to the app, with the same 5 s
//!   timeout, so the app is covered from its first instruction.  [`start`]
//!   programs it again, and starts it when probe-rs ran the app without the
//!   bootloader.
//! - The main loop feeds it once per pass ([`feed`]).  A pass that blocks
//!   longer resets the chip, and `reset_record` reports the loop phase it
//!   was in at the next boot.  5 s is the nominal timeout; the LSI's
//!   29.5-34 kHz over temperature (DS13105 Table 58) makes it 4.7-5.4 s.
//! - Waits below the main loop that carry their own deadline feed it through
//!   [`feed_now`].
//! - A watchdog reset of an image on trial (just swapped in by an update)
//!   makes the bootloader revert it; `supervisor` decides when the image has
//!   run long enough to keep.
//! - With the `debug` feature it stops counting while a debugger halts the
//!   core, so a halt at a breakpoint or a caught fault is not reset away
//!   under the debugger.

use stm32wlxx_hal::pac;

// IWDG key register magic values
const KEY_ENABLE: u16 = 0xCCCC;
const KEY_RELOAD: u16 = 0xAAAA;
const KEY_UNLOCK: u16 = 0x5555;

/// Start the IWDG with the given timeout in milliseconds, or set that
/// timeout on one already running.
///
/// The LSI clock is ~32 kHz.  With prescaler /64 the counter ticks at
/// ~500 Hz, giving a maximum timeout of ~8 s (reload 0xFFF = 4095).
///
/// Common values:
/// - 5000 ms -> reload ~ 2500
/// - 8000 ms -> reload ~ 4000
///
/// Clamps to the hardware maximum of 4095.
pub fn start(iwdg: &pac::IWDG, timeout_ms: u32) {
    if cfg!(feature = "debug") {
        // SAFETY: DBGMCU_APB1FZR1 only changes what the watchdog does while
        // a debugger halts the core; nothing else in the app writes it.
        unsafe {
            (*pac::DBGMCU::PTR)
                .apb1fzr1
                .modify(|_, w| w.dbg_iwdg_stop().set_bit())
        };
    }

    // Compute reload value: LSI ~ 32 kHz, prescaler = 64 -> tick = 500 Hz
    let ticks = (timeout_ms * 500) / 1000;
    let reload = ticks.min(4095) as u16;

    // Enable IWDG
    iwdg.kr.write(|w| unsafe { w.key().bits(KEY_ENABLE) });
    // Unlock PR and RLR
    iwdg.kr.write(|w| unsafe { w.key().bits(KEY_UNLOCK) });
    // Prescaler = /64 (PR = 4)
    iwdg.pr.write(|w| w.pr().bits(4));
    // Reload value
    iwdg.rlr.write(|w| w.rl().bits(reload));
    // Wait for prescaler (PVU) and reload (RVU) updates only.
    // Checking all bits would also wait on WVU (window value update),
    // which may never clear if WINR was not written, causing a hang.
    while iwdg.sr.read().bits() & 0x03 != 0 {}
    // Initial reload
    iwdg.kr.write(|w| unsafe { w.key().bits(KEY_RELOAD) });
}

/// Feed (reload) the watchdog counter, preventing a reset.
///
/// Must be called at least once per timeout period.
#[inline]
pub fn feed(iwdg: &pac::IWDG) {
    iwdg.kr.write(|w| unsafe { w.key().bits(KEY_RELOAD) });
}

/// Feed the watchdog without holding the peripheral.
///
/// The key register is write-only and the reload key is idempotent, so this
/// cannot disturb whatever the owner of the [`pac::IWDG`] is doing, and a
/// write before [`start`] is simply ignored.
///
/// It exists for the bounded wait loops that sit below the main loop and
/// cannot reach the peripheral: the blocking LoRa transmit and the SD card
/// transfers.  Both carry their own deadline, so feeding from inside them
/// says "still waiting on something that will time out", not "trust me".
/// Loops with no deadline of their own - [`Sx1262Driver::wait_on_busy`] in
/// particular - are deliberately left unfed, because a radio that never
/// releases BUSY is exactly what the watchdog is for.
///
/// [`Sx1262Driver::wait_on_busy`]: crate::radio::Sx1262Driver
#[inline]
pub fn feed_now() {
    // SAFETY: a single write of the reload key to a write-only register.
    unsafe {
        (*pac::IWDG::PTR).kr.write(|w| w.key().bits(KEY_RELOAD));
    }
}
