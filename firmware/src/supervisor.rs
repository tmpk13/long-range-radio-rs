//! Health decisions the main loop takes on top of the watchdog: when an image
//! on trial has proven itself, when a radio that stopped transmitting needs a
//! re-init or a reset, and how long to keep off an I2C bus that hung the
//! loop.
//!
//! The watchdog only proves that the loop keeps turning.  A node whose loop
//! turns while its radio sends nothing is just as lost; the
//! `SendingQueueIsFull` failure (notes/mesh.md) was one, every broadcast
//! refused while the loop fed the watchdog.  [`RadioWatch`] catches that.
//!
//! Pure functions of `platform::millis()` timestamps, which wrap after 49
//! days: every comparison is a wrapping difference.

/// A trial image is kept once it has run this long...
pub const CONFIRM_AFTER_MS: u32 = 30 * 60_000;
/// ...with a transmission completed this recently...
pub const TX_RECENT_MS: u32 = 5 * 60_000;
/// ...and a packet heard since boot, or this much uptime without one.  A
/// node with nobody in range cannot show that it still receives, and
/// reverting would not make it hear anyone.
pub const CONFIRM_ALONE_MS: u32 = 6 * 60 * 60_000;

/// No completed transmission for this long (the heartbeat goes out every
/// 4-7 s) re-initializes the radio...
pub const TX_STALL_MS: u32 = 10 * 60_000;
/// ...and none for this long after the re-init resets the chip.
pub const RESET_AFTER_REINIT_MS: u32 = 5 * 60_000;
/// No packet for this long, once one has been heard, re-initializes the
/// radio, at most once per this period: a receiver left out of RX looks
/// like that.  Silence alone never resets the chip.
pub const RX_SILENCE_MS: u32 = 6 * 60 * 60_000;

/// Milliseconds from `since` to `now`.
fn elapsed(now: u32, since: u32) -> u32 {
    now.wrapping_sub(since)
}

/// `a` is later than `b`.
fn later(a: u32, b: u32) -> bool {
    (a.wrapping_sub(b) as i32) > 0
}

/// Whether an image on trial has run healthy long enough to keep.  `boot` is
/// when the loop started; `last_tx_ok` when a transmission last completed;
/// `heard` whether any packet has arrived since boot.
pub fn confirm_ready(now: u32, boot: u32, last_tx_ok: Option<u32>, heard: bool) -> bool {
    let up = elapsed(now, boot);
    let tx_fresh = last_tx_ok.is_some_and(|t| elapsed(now, t) <= TX_RECENT_MS);
    up >= CONFIRM_AFTER_MS && tx_fresh && (heard || up >= CONFIRM_ALONE_MS)
}

/// What [`RadioWatch::poll`] wants done.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RadioAction {
    /// The radio is fine.
    Nothing,
    /// Re-initialize the radio.
    Reinit,
    /// Reset the chip, recording why.
    Reset,
}

/// Watches the radio's completed transmissions and receptions.
#[derive(Clone, Copy, Debug)]
pub struct RadioWatch {
    boot: u32,
    tx_reinit_at: Option<u32>,
    rx_reinit_at: Option<u32>,
}

impl RadioWatch {
    /// Start watching at `now`.
    pub fn new(now: u32) -> Self {
        Self {
            boot: now,
            tx_reinit_at: None,
            rx_reinit_at: None,
        }
    }

    /// Decide from when a transmission last completed and a packet last
    /// arrived (`None`: not since boot).
    pub fn poll(&mut self, now: u32, last_tx_ok: Option<u32>, last_rx: Option<u32>) -> RadioAction {
        if let Some(at) = self.tx_reinit_at {
            if last_tx_ok.is_some_and(|t| later(t, at)) {
                self.tx_reinit_at = None;
            } else if elapsed(now, at) >= RESET_AFTER_REINIT_MS {
                return RadioAction::Reset;
            } else {
                return RadioAction::Nothing;
            }
        } else if elapsed(now, last_tx_ok.unwrap_or(self.boot)) >= TX_STALL_MS {
            self.tx_reinit_at = Some(now);
            return RadioAction::Reinit;
        }

        if let Some(rx) = last_rx {
            let since = match self.rx_reinit_at {
                Some(r) if later(r, rx) => r,
                _ => rx,
            };
            if elapsed(now, since) >= RX_SILENCE_MS {
                self.rx_reinit_at = Some(now);
                return RadioAction::Reinit;
            }
        }
        RadioAction::Nothing
    }
}

/// How long to keep off the I2C bus after `strikes` watchdog resets in a
/// row that hit during I2C: an hour, doubling per strike, at most a day.
pub fn i2c_hold_ms(strikes: u32) -> u32 {
    const HOUR: u32 = 60 * 60_000;
    match strikes {
        0 => 0,
        n => HOUR.saturating_mul(1 << (n - 1).min(5)).min(24 * HOUR),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MIN: u32 = 60_000;

    #[test]
    fn confirm_needs_uptime_a_fresh_transmission_and_a_packet() {
        let boot = 1_000;
        let now = boot + CONFIRM_AFTER_MS;
        assert!(confirm_ready(now, boot, Some(now - MIN), true));
        // Too early.
        assert!(!confirm_ready(now - 1, boot, Some(now - MIN), true));
        // No transmission, or a stale one.
        assert!(!confirm_ready(now, boot, None, true));
        assert!(!confirm_ready(
            now,
            boot,
            Some(now - TX_RECENT_MS - 1),
            true
        ));
        // Nothing heard yet, until the node has been alone long enough.
        assert!(!confirm_ready(now, boot, Some(now - MIN), false));
        let alone = boot + CONFIRM_ALONE_MS;
        assert!(confirm_ready(alone, boot, Some(alone - MIN), false));
    }

    #[test]
    fn confirm_across_the_millis_wrap() {
        let boot = u32::MAX - 10 * MIN;
        let now = boot.wrapping_add(CONFIRM_AFTER_MS);
        assert!(confirm_ready(now, boot, Some(now.wrapping_sub(MIN)), true));
    }

    #[test]
    fn a_stalled_transmitter_is_reinitialized_then_reset() {
        let boot = 5;
        let mut w = RadioWatch::new(boot);
        assert_eq!(
            w.poll(boot + TX_STALL_MS - 1, None, None),
            RadioAction::Nothing
        );
        let reinit = boot + TX_STALL_MS;
        assert_eq!(w.poll(reinit, None, None), RadioAction::Reinit);
        assert_eq!(
            w.poll(reinit + RESET_AFTER_REINIT_MS - 1, None, None),
            RadioAction::Nothing
        );
        assert_eq!(
            w.poll(reinit + RESET_AFTER_REINIT_MS, None, None),
            RadioAction::Reset
        );
    }

    #[test]
    fn a_transmission_after_the_reinit_recovers() {
        let mut w = RadioWatch::new(0);
        let last = 3 * MIN;
        let reinit = last + TX_STALL_MS;
        assert_eq!(w.poll(reinit, Some(last), None), RadioAction::Reinit);
        // The transmission before the re-init does not count.
        assert_eq!(
            w.poll(reinit + RESET_AFTER_REINIT_MS, Some(last), None),
            RadioAction::Reset
        );
        let mut w = RadioWatch::new(0);
        assert_eq!(w.poll(reinit, Some(last), None), RadioAction::Reinit);
        let ok = reinit + MIN;
        assert_eq!(w.poll(ok + MIN, Some(ok), None), RadioAction::Nothing);
        assert_eq!(
            w.poll(reinit + RESET_AFTER_REINIT_MS, Some(ok), None),
            RadioAction::Nothing
        );
        // A later stall starts over at a re-init.
        let stall = ok + TX_STALL_MS;
        assert_eq!(w.poll(stall, Some(ok), None), RadioAction::Reinit);
    }

    #[test]
    fn regular_transmissions_keep_it_quiet() {
        let mut w = RadioWatch::new(0);
        for t in (0..20 * TX_STALL_MS).step_by(5_000) {
            assert_eq!(w.poll(t, Some(t), None), RadioAction::Nothing);
        }
    }

    #[test]
    fn receive_silence_reinitializes_once_per_period_and_never_resets() {
        let mut w = RadioWatch::new(0);
        let heard = 2 * MIN;
        let mut reinits = 0;
        let mut t = heard;
        while t < heard + 3 * RX_SILENCE_MS + 1 {
            match w.poll(t, Some(t), Some(heard)) {
                RadioAction::Reinit => reinits += 1,
                RadioAction::Reset => panic!("silence reset the chip"),
                RadioAction::Nothing => {}
            }
            t += MIN;
        }
        assert_eq!(reinits, 3);
        // Nothing heard since boot: being alone is normal.
        let mut w = RadioWatch::new(0);
        assert_eq!(
            w.poll(2 * RX_SILENCE_MS, Some(2 * RX_SILENCE_MS), None),
            RadioAction::Nothing
        );
    }

    #[test]
    fn stall_detection_across_the_millis_wrap() {
        let boot = u32::MAX - MIN;
        let mut w = RadioWatch::new(boot);
        assert_eq!(
            w.poll(boot.wrapping_add(TX_STALL_MS - 1), None, None),
            RadioAction::Nothing
        );
        assert_eq!(
            w.poll(boot.wrapping_add(TX_STALL_MS), None, None),
            RadioAction::Reinit
        );
    }

    #[test]
    fn i2c_hold_doubles_to_a_day() {
        const HOUR: u32 = 60 * MIN;
        assert_eq!(i2c_hold_ms(0), 0);
        assert_eq!(i2c_hold_ms(1), HOUR);
        assert_eq!(i2c_hold_ms(2), 2 * HOUR);
        assert_eq!(i2c_hold_ms(5), 16 * HOUR);
        assert_eq!(i2c_hold_ms(6), 24 * HOUR);
        assert_eq!(i2c_hold_ms(u32::MAX), 24 * HOUR);
    }
}
