//! Platform helper functions.

use esp_hal::time::Instant;

/// Milliseconds since boot.
pub fn millis() -> u32 {
    Instant::now().duration_since_epoch().as_millis() as u32
}

/// Random number in `[min, max)`.
///
/// Uses a simple xorshift32 PRNG seeded from the system timer.
pub fn random(min: i32, max: i32) -> i32 {
    use core::sync::atomic::{AtomicU32, Ordering};

    static STATE: AtomicU32 = AtomicU32::new(0);

    let mut s = STATE.load(Ordering::Relaxed);
    if s == 0 {
        // Seed from the timer
        s = Instant::now().duration_since_epoch().as_micros() as u32;
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
