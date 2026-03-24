#![no_std]

/// Prints only when the `debug` cargo feature is enabled.
#[macro_export]
macro_rules! debug_println {
    ($($arg:tt)*) => {
        if cfg!(feature = "debug") {
            rtt_target::rprintln!($($arg)*)
        }
    };
}

pub mod config;
pub mod platform;
pub mod radio;
