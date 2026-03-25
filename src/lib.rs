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

pub mod boot_state;
pub mod config;
pub mod io;
pub mod node;
pub mod ota;
pub mod ota_protocol;
pub mod ota_sender;
pub mod platform;
pub mod radio;
pub mod watchdog;

pub use embedded_nano_mesh::{LifeTimeType, SendError};
pub use io::LoraIo;
pub use node::{MeshMessage, MeshNode};
pub use ota::OtaReceiver;
pub use ota_sender::OtaSender;
