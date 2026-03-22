//! Platform helper functions for RP2040.
//!
//! Uses the RP2040's free-running 64-bit microsecond timer
//! which is always accessible via memory-mapped registers.

/// Microseconds since boot, read from the RP2040 hardware timer.
pub fn micros() -> u64 {
    // The RP2040 TIMER is a free-running 64-bit microsecond counter.
    // Reading TIMELR first latches TIMEHR for an atomic 64-bit read.
    let timer = unsafe { &*rp2040_hal::pac::TIMER::ptr() };
    let lo = timer.timelr().read().bits();
    let hi = timer.timehr().read().bits();
    ((hi as u64) << 32) | lo as u64
}

/// Milliseconds since boot.
pub fn millis() -> u32 {
    (micros() / 1000) as u32
}

/// Random number in `[min, max)`.
///
/// Uses a simple xorshift32 PRNG seeded from the hardware timer.
pub fn random(min: i32, max: i32) -> i32 {
    use core::sync::atomic::{AtomicU32, Ordering};

    static STATE: AtomicU32 = AtomicU32::new(0);

    let mut s = STATE.load(Ordering::Relaxed);
    if s == 0 {
        s = micros() as u32;
        if s == 0 {
            s = 1;
        }
    }
    // xorshift32
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    STATE.store(s, Ordering::Relaxed);

    if max <= min {
        return min;
    }
    min + (s as i32).unsigned_abs() as i32 % (max - min)
}
