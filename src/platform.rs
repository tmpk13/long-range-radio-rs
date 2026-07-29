//! Platform helper functions for STM32WLE5.

/// System clock frequency in Hz (default MSI = 4 MHz).
/// Update this if you reconfigure the clock tree.
///
/// With the `board` feature the MSI is raised to 16 MHz early in init
/// (see `board::raise_sysclk`) so TIM1 has enough resolution for the
/// 100 kHz buck PWM.
#[cfg(not(feature = "board"))]
pub const SYSCLK_HZ: u32 = 4_000_000;
#[cfg(feature = "board")]
pub const SYSCLK_HZ: u32 = 16_000_000;

/// Milliseconds since boot (via DWT cycle counter).
///
/// Uses wrapping subtraction to handle the DWT counter rollover at ~1073 s
/// @ 4 MHz, producing a monotonically-increasing u32 that wraps only at
/// ~49 days.  Enable the cycle counter in init before calling this:
/// ```ignore
/// cx.core.DCB.enable_trace();
/// cx.core.DWT.enable_cycle_counter();
/// ```
pub fn millis() -> u32 {
    use core::sync::atomic::{AtomicU32, Ordering};

    const TICKS_PER_MS: u32 = SYSCLK_HZ / 1000; // 4000 at 4 MHz

    static PREV_CYCLES: AtomicU32 = AtomicU32::new(0);
    static ACCUM_MS: AtomicU32 = AtomicU32::new(0);
    static LEFTOVER: AtomicU32 = AtomicU32::new(0);

    let current = cortex_m::peripheral::DWT::cycle_count();
    let prev = PREV_CYCLES.swap(current, Ordering::Relaxed);
    // wrapping_sub correctly handles the DWT counter rolling over from
    // u32::MAX back to 0.
    let elapsed = current.wrapping_sub(prev);
    let leftover = LEFTOVER.load(Ordering::Relaxed);
    let total = leftover.saturating_add(elapsed);
    let new_ms = total / TICKS_PER_MS;
    LEFTOVER.store(total % TICKS_PER_MS, Ordering::Relaxed);
    ACCUM_MS.fetch_add(new_ms, Ordering::Relaxed).wrapping_add(new_ms)
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
    // Reduce in unsigned space. Going through `(s as i32).unsigned_abs()`
    // returns 2^31 for exactly one state value, which casts back to a negative
    // i32 and, since `%` keeps the sign of its left operand, produced a result
    // below `min` - a repeat jitter that reads as already due.
    let span = max.wrapping_sub(min) as u32;
    min.wrapping_add((s % span) as i32)
}
