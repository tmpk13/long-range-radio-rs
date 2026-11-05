//! Why the chip last reset, and where the main loop was when it did.
//!
//! The reset flags in RCC_CSR say which reset fired; this module adds what
//! they cannot: why a software reset was asked for, where a panic happened,
//! and which part of the loop was running.  All of it lives in TAMP backup
//! registers, which a system reset leaves alone (RM0461 31.6.10) and power
//! loss clears.
//!
//! [`take`] runs once at boot: it reads everything, clears the RCC_CSR
//! flags (they otherwise accumulate until power is lost, so a later reading
//! cannot tell which reset came last) and the one-shot records, and counts
//! the boot.
//!
//! | TAMP_BKPxR | Holds                                                      |
//! |------------|------------------------------------------------------------|
//! | 0          | the bootloader's handoff word (`flash_layout::boot::handoff`) |
//! | 1          | boots since power-on                                       |
//! | 2          | the loop phase, written before each blocking step          |
//! | 3          | a panic's line, tagged                                     |
//! | 4          | a panic's file: the first four bytes of its base name      |
//! | 5          | the reason for a software reset, tagged                    |
//! | 6          | watchdog resets in a row that hit during I2C               |

use flash_layout::boot::{Outcome, handoff};
use stm32wlxx_hal::pac;

const BKP_BOOTS: usize = 1;
const BKP_PHASE: usize = 2;
const BKP_PANIC_LINE: usize = 3;
const BKP_PANIC_FILE: usize = 4;
const BKP_SOFT_RESET: usize = 5;
const BKP_I2C_STRIKES: usize = 6;

const PANIC_TAG: u32 = 0xDEAD_0000;
const SOFT_TAG: u32 = 0x5EF7_0000;

/// What the main loop was doing.  The number is what telemetry reports.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Phase {
    /// `init`, before the loop.
    Init = 1,
    /// Display init, retry or drawing (I2C).
    Display = 2,
    /// The I2C sensor probe or sweep.
    Sensors = 3,
    /// The RS-485 soil probe read.
    Soil = 4,
    /// GPS and SD card logging.
    GpsLog = 5,
    /// The MPPT charger step.
    Charger = 6,
    /// The mesh update: radio receive and transmit.
    Mesh = 7,
    /// Handling an OTA message, flash writes included.
    Ota = 8,
    /// Building and queueing the heartbeat.
    Heartbeat = 9,
    /// Between loop passes, in the 1 ms delay.
    Idle = 10,
}

impl Phase {
    fn from_code(code: u32) -> Option<Phase> {
        Some(match code {
            1 => Phase::Init,
            2 => Phase::Display,
            3 => Phase::Sensors,
            4 => Phase::Soil,
            5 => Phase::GpsLog,
            6 => Phase::Charger,
            7 => Phase::Mesh,
            8 => Phase::Ota,
            9 => Phase::Heartbeat,
            10 => Phase::Idle,
            _ => return None,
        })
    }

    /// The phase drives the I2C bus.
    pub fn is_i2c(self) -> bool {
        matches!(self, Phase::Display | Phase::Sensors)
    }
}

/// Why the app asked for a reset.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum SoftReset {
    /// A verified image is staged: reboot so the bootloader swaps it in.
    Ota = 1,
    /// The radio stopped transmitting and a re-init did not help.
    RadioStall = 2,
}

/// Where a panic happened.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PanicSite {
    /// Source line.
    pub line: u16,
    /// The first four bytes of the source file's base name.
    pub file: [u8; 4],
}

/// The reset that started this boot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cause {
    /// Power came up (power-on and brown-out resets set the same flag).
    PowerOn,
    /// The NRST pin: the reset button, or a probe.
    Pin,
    /// The independent watchdog: something blocked the main loop.
    Watchdog,
    /// A panic; the watchdog then reset the spinning handler.
    Panic,
    /// The app asked for it; `None` for a software reset with no recorded
    /// reason, such as a debugger's.
    Software(Option<SoftReset>),
    /// An illegal low-power mode entry.
    LowPower,
    /// The window watchdog (not used by this firmware).
    WindowWatchdog,
    /// An option-byte load.
    OptionBytes,
    /// No flag set.
    Unknown,
}

impl Cause {
    /// The number telemetry reports as `W=`.
    pub fn code(self) -> u8 {
        match self {
            Cause::Unknown => 0,
            Cause::PowerOn => 1,
            Cause::Pin => 2,
            Cause::Watchdog => 3,
            Cause::Panic => 4,
            Cause::Software(Some(SoftReset::Ota)) => 5,
            Cause::Software(Some(SoftReset::RadioStall)) => 6,
            Cause::Software(None) => 7,
            Cause::LowPower => 8,
            Cause::WindowWatchdog => 9,
            Cause::OptionBytes => 10,
        }
    }
}

// RCC_CSR reset flags (RM0461, RCC_CSR).
const CSR_LPWRRSTF: u32 = 1 << 31;
const CSR_WWDGRSTF: u32 = 1 << 30;
const CSR_IWDGRSTF: u32 = 1 << 29;
const CSR_SFTRSTF: u32 = 1 << 28;
const CSR_BORRSTF: u32 = 1 << 27;
const CSR_PINRSTF: u32 = 1 << 26;
const CSR_OBLRSTF: u32 = 1 << 25;

/// Which reset started this boot.  Every internal reset source drives NRST
/// low as well (RM0461 6.1.2), so PINRSTF comes with the real cause and is
/// tested last.  A recorded panic outranks the watchdog reset that ended it.
pub fn classify(csr: u32, soft: Option<SoftReset>, panicked: bool) -> Cause {
    if panicked {
        Cause::Panic
    } else if csr & CSR_IWDGRSTF != 0 {
        Cause::Watchdog
    } else if csr & CSR_WWDGRSTF != 0 {
        Cause::WindowWatchdog
    } else if csr & CSR_LPWRRSTF != 0 {
        Cause::LowPower
    } else if csr & CSR_SFTRSTF != 0 {
        Cause::Software(soft)
    } else if csr & CSR_OBLRSTF != 0 {
        Cause::OptionBytes
    } else if csr & CSR_BORRSTF != 0 {
        Cause::PowerOn
    } else if csr & CSR_PINRSTF != 0 {
        Cause::Pin
    } else {
        Cause::Unknown
    }
}

/// What [`take`] found.
#[derive(Clone, Copy, Debug)]
pub struct Record {
    /// The reset that started this boot.
    pub cause: Cause,
    /// Boots since power-on, this one included.
    pub boots: u32,
    /// The loop phase when the reset hit, if one was recorded.
    pub phase: Option<Phase>,
    /// Where the last run panicked.
    pub panic: Option<PanicSite>,
    /// What the bootloader did this boot; `None` when it did not start the
    /// app.
    pub handoff: Option<Outcome>,
    /// Watchdog resets in a row that hit during I2C, this one included.
    pub i2c_strikes: u32,
}

impl Record {
    /// The number telemetry reports as `B=`: 0 when the bootloader did not
    /// start the app, then idle 1, trial 2, refused 3, reverted 4.
    pub fn boot_code(&self) -> u8 {
        match self.handoff {
            None => 0,
            Some(Outcome::Idle) => 1,
            Some(Outcome::Trial) => 2,
            Some(Outcome::Refused(_)) => 3,
            Some(Outcome::Reverted) => 4,
        }
    }
}

fn tamp() -> &'static pac::tamp::RegisterBlock {
    // SAFETY: the backup registers are only touched through this module,
    // and every access is a single volatile read or write.
    unsafe { &*pac::TAMP::PTR }
}

fn bkp_read(i: usize) -> u32 {
    tamp().bkpr[i].read().bits()
}

fn bkp_write(i: usize, v: u32) {
    tamp().bkpr[i].write(|w| w.bits(v));
}

/// Read and reset the record of the last run; call once, early in `init`.
/// Clocks the TAMP registers and lifts the backup domain's write protection
/// (PWR_CR1.DBP), which stays lifted for [`set_phase`] and the panic
/// handler.
pub fn take(rcc: &mut pac::RCC) -> Record {
    rcc.apb1enr1.modify(|_, w| w.rtcapben().set_bit());
    // Read back: the clock must be running before TAMP is accessed.
    let _ = rcc.apb1enr1.read();
    // SAFETY: DBP only gates writes to the backup domain.
    unsafe { (*pac::PWR::PTR).cr1.modify(|_, w| w.dbp().set_bit()) };

    let csr = rcc.csr.read().bits();
    rcc.csr.modify(|_, w| w.rmvf().set_bit());

    let soft = match bkp_read(BKP_SOFT_RESET) {
        v if v & 0xFFFF_0000 == SOFT_TAG && v as u16 == SoftReset::Ota as u16 => {
            Some(SoftReset::Ota)
        }
        v if v & 0xFFFF_0000 == SOFT_TAG && v as u16 == SoftReset::RadioStall as u16 => {
            Some(SoftReset::RadioStall)
        }
        _ => None,
    };
    let line = bkp_read(BKP_PANIC_LINE);
    let panic = (line & 0xFFFF_0000 == PANIC_TAG).then(|| PanicSite {
        line: line as u16,
        file: bkp_read(BKP_PANIC_FILE).to_le_bytes(),
    });
    let phase = Phase::from_code(bkp_read(BKP_PHASE));
    let handoff = handoff::decode(bkp_read(handoff::REGISTER));
    let boots = bkp_read(BKP_BOOTS).wrapping_add(1);
    let cause = classify(csr, soft, panic.is_some());

    let strikes = match (cause, phase) {
        (Cause::Watchdog, Some(p)) if p.is_i2c() => bkp_read(BKP_I2C_STRIKES).saturating_add(1),
        // Any other reset keeps the count; only working I2C clears it.
        _ => bkp_read(BKP_I2C_STRIKES),
    };

    for i in [
        handoff::REGISTER,
        BKP_PHASE,
        BKP_PANIC_LINE,
        BKP_PANIC_FILE,
        BKP_SOFT_RESET,
    ] {
        bkp_write(i, 0);
    }
    bkp_write(BKP_BOOTS, boots);
    bkp_write(BKP_I2C_STRIKES, strikes);
    set_phase(Phase::Init);

    Record {
        cause,
        boots,
        phase,
        panic,
        handoff,
        i2c_strikes: strikes,
    }
}

/// Note what the loop is about to do; one register write.
pub fn set_phase(phase: Phase) {
    bkp_write(BKP_PHASE, phase as u32);
}

/// Forget the I2C watchdog strikes: the bus worked.
pub fn clear_i2c_strikes() {
    bkp_write(BKP_I2C_STRIKES, 0);
}

/// Record a panic's site for the next boot to report.
pub fn record_panic(line: u32, file: &str) {
    let name = file.rsplit(['/', '\\']).next().unwrap_or(file).as_bytes();
    let mut four = [b'?'; 4];
    for (d, s) in four.iter_mut().zip(name) {
        *d = *s;
    }
    bkp_write(BKP_PANIC_FILE, u32::from_le_bytes(four));
    bkp_write(BKP_PANIC_LINE, PANIC_TAG | line.min(0xFFFF));
}

/// Record why, then reset the chip.
pub fn software_reset(reason: SoftReset) -> ! {
    bkp_write(BKP_SOFT_RESET, SOFT_TAG | reason as u32);
    cortex_m::peripheral::SCB::sys_reset()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pin_flag_comes_with_every_reset_and_loses() {
        assert_eq!(classify(CSR_PINRSTF, None, false), Cause::Pin);
        assert_eq!(
            classify(CSR_PINRSTF | CSR_IWDGRSTF, None, false),
            Cause::Watchdog
        );
        assert_eq!(
            classify(CSR_PINRSTF | CSR_SFTRSTF, Some(SoftReset::Ota), false),
            Cause::Software(Some(SoftReset::Ota))
        );
        assert_eq!(
            classify(CSR_PINRSTF | CSR_BORRSTF, None, false),
            Cause::PowerOn
        );
        assert_eq!(
            classify(CSR_PINRSTF | CSR_LPWRRSTF, None, false),
            Cause::LowPower
        );
    }

    #[test]
    fn a_debugger_reset_has_no_reason() {
        assert_eq!(
            classify(CSR_PINRSTF | CSR_SFTRSTF, None, false),
            Cause::Software(None)
        );
    }

    #[test]
    fn a_recorded_panic_outranks_the_watchdog() {
        assert_eq!(
            classify(CSR_PINRSTF | CSR_IWDGRSTF, None, true),
            Cause::Panic
        );
    }

    #[test]
    fn no_flags() {
        assert_eq!(classify(0, None, false), Cause::Unknown);
    }

    #[test]
    fn codes_are_distinct() {
        let all = [
            Cause::Unknown,
            Cause::PowerOn,
            Cause::Pin,
            Cause::Watchdog,
            Cause::Panic,
            Cause::Software(Some(SoftReset::Ota)),
            Cause::Software(Some(SoftReset::RadioStall)),
            Cause::Software(None),
            Cause::LowPower,
            Cause::WindowWatchdog,
            Cause::OptionBytes,
        ];
        for (i, a) in all.iter().enumerate() {
            for b in &all[i + 1..] {
                assert_ne!(a.code(), b.code());
            }
        }
    }
}
