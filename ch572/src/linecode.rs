//! CDC line coding to CH572 UART register values.
//!
//! The UART runs from Fsys with the pre-divider R8_UART_DIV at 1, so the
//! bit rate is Fsys * 2 / 16 / DL = Fsys / (8 * DL) (datasheet 9.3.1;
//! WCH's UART_BaudRateCfg rounds the same way).

use crate::regs::{RB_LCR_PAR_EN, RB_LCR_STOP_BIT};
use usbd_serial::{ParityType, StopBits};

/// Register values for one line setting.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UartSetting {
    /// R16_UART_DL.
    pub dl: u16,
    /// R8_UART_LCR.
    pub lcr: u8,
}

/// Rate used when the host asks for 0 bit/s, which CDC allows.
pub const DEFAULT_BAUD: u32 = 115_200;

pub fn uart_setting(
    fsys_hz: u32,
    baud: u32,
    data_bits: u8,
    parity: ParityType,
    stop_bits: StopBits,
) -> UartSetting {
    let baud = if baud == 0 { DEFAULT_BAUD } else { baud };
    let dl = ((fsys_hz as u64 + 4 * baud as u64) / (8 * baud as u64)).clamp(1, u16::MAX as u64);
    // RB_LCR_WORD_SZ: 00 = 5 bits up to 11 = 8 bits.  CDC also allows 16,
    // which the UART cannot do; 8 is the closest.
    let word = match data_bits {
        5..=8 => data_bits - 5,
        _ => 3,
    };
    // RB_LCR_PAR_MOD: 00 odd, 01 even, 10 mark, 11 space.
    let parity = match parity {
        ParityType::None => 0,
        ParityType::Odd => RB_LCR_PAR_EN,
        ParityType::Even => RB_LCR_PAR_EN | 0x10,
        ParityType::Mark => RB_LCR_PAR_EN | 0x20,
        ParityType::Space => RB_LCR_PAR_EN | 0x30,
    };
    // One stop bit or two; the UART has no 1.5.
    let stop = match stop_bits {
        StopBits::One => 0,
        StopBits::OnePointFive | StopBits::Two => RB_LCR_STOP_BIT,
    };
    UartSetting {
        dl: dl as u16,
        lcr: word | parity | stop,
    }
}

/// The bit rate a divisor actually produces.
pub fn actual_baud(fsys_hz: u32, dl: u16) -> u32 {
    fsys_hz / (8 * dl as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    const FSYS: u32 = crate::FSYS_HZ;

    fn error_ppm(baud: u32) -> i64 {
        let s = uart_setting(FSYS, baud, 8, ParityType::None, StopBits::One);
        let actual = actual_baud(FSYS, s.dl) as i64;
        (actual - baud as i64) * 1_000_000 / baud as i64
    }

    #[test]
    fn common_rates_are_within_half_a_percent() {
        // The UART tolerates 2% on receive; the Wio-E5's USART is set to
        // the nominal rate, so the bridge's own error is the whole budget.
        for baud in [9_600, 19_200, 38_400, 57_600, 115_200, 230_400, 460_800] {
            let ppm = error_ppm(baud);
            assert!(ppm.abs() < 5_000, "{baud}: {ppm} ppm");
        }
    }

    #[test]
    fn eight_n_one_and_framing_bits() {
        let s = uart_setting(FSYS, 115_200, 8, ParityType::None, StopBits::One);
        assert_eq!(s.lcr, 0x03);
        assert_eq!(s.dl, 109);
        let s = uart_setting(FSYS, 9_600, 7, ParityType::Even, StopBits::Two);
        assert_eq!(s.lcr, 0x02 | 0x08 | 0x10 | 0x04);
        let s = uart_setting(FSYS, 9_600, 16, ParityType::Space, StopBits::OnePointFive);
        assert_eq!(s.lcr, 0x03 | 0x08 | 0x30 | 0x04);
    }

    #[test]
    fn zero_and_extreme_rates_stay_in_range() {
        let s = uart_setting(FSYS, 0, 8, ParityType::None, StopBits::One);
        assert_eq!(
            s,
            uart_setting(FSYS, DEFAULT_BAUD, 8, ParityType::None, StopBits::One)
        );
        assert_eq!(
            uart_setting(FSYS, 1, 8, ParityType::None, StopBits::One).dl,
            u16::MAX
        );
        assert_eq!(
            uart_setting(FSYS, 100_000_000, 8, ParityType::None, StopBits::One).dl,
            1
        );
    }
}
