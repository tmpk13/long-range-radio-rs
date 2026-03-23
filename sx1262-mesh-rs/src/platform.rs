//! Platform helper functions for STM32WLE5.

/// System clock frequency in Hz (default MSI = 4 MHz).
/// Update this if you reconfigure the clock tree.
pub const SYSCLK_HZ: u32 = 4_000_000;

/// Milliseconds since boot (via DWT cycle counter).
///
/// The DWT counter is 32-bit and wraps at ~1073 s @ 4 MHz.
/// Enable the cycle counter in init before calling this:
/// ```ignore
/// cx.core.DCB.enable_trace();
/// cx.core.DWT.enable_cycle_counter();
/// ```
pub fn millis() -> u32 {
    let cycles = cortex_m::peripheral::DWT::cycle_count();
    cycles / (SYSCLK_HZ / 1000)
}

/// Random number in `[min, max)`.
///
/// Uses a simple xorshift32 PRNG seeded from the DWT cycle counter.
pub fn random(min: i32, max: i32) -> i32 {
    use core::sync::atomic::{AtomicU32, Ordering};

    static STATE: AtomicU32 = AtomicU32::new(0);

    let mut s = STATE.load(Ordering::Relaxed);
    if s == 0 {
        s = cortex_m::peripheral::DWT::cycle_count();
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
