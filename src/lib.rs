#![no_std]

/// Prints only when the `debug` cargo feature is enabled.
#[macro_export]
macro_rules! debug_println {
    ($($arg:tt)*) => {
        if cfg!(feature = "debug") {
            // Semicolon required: `rprintln!` expands to a block ending in a
            // trailing semicolon, which is an error in expression position.
            rtt_target::rprintln!($($arg)*);
        }
    };
}

#[cfg(feature = "board")]
pub mod board;
pub mod boot_state;
pub mod config;
#[cfg(feature = "gps-radio-log")]
pub mod gpslog;
pub mod io;
pub mod node;
pub mod ota;
pub mod ota_protocol;
pub mod ota_sender;
pub mod platform;
pub mod radio;
#[cfg(feature = "rs422")]
pub mod rs422;
#[cfg(feature = "sensor")]
pub mod sensors;
pub mod watchdog;

pub use embedded_nano_mesh::{LifeTimeType, SendError};
pub use io::LoraIo;
pub use node::{MeshMessage, MeshNode};
pub use ota::OtaReceiver;
pub use ota_sender::OtaSender;
