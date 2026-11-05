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

// Both features need USART1: on the Forest Sensor Platform its pins go to
// the RS-485 transceiver, which `rs422` drives and `gps-radio-log` only
// listens through.
#[cfg(all(feature = "rs422", feature = "gps-radio-log"))]
compile_error!("features `rs422` and `gps-radio-log` both use USART1; enable one of them");

#[cfg(feature = "board")]
pub mod board;
pub mod boot_state;
pub mod config;
pub mod flash_ecc;
#[cfg(feature = "gps-radio-log")]
pub mod gpslog;
pub mod i2c_bus;
pub mod io;
pub mod node;
pub mod ota;
pub mod ota_protocol;
pub mod ota_sender;
pub mod platform;
pub mod radio;
pub mod reset_record;
#[cfg(feature = "rs422")]
pub mod rs422;
#[cfg(feature = "sensor")]
pub mod sensors;
pub mod supervisor;
pub mod watchdog;

/// The panic handler of both apps: record where it happened for the next
/// boot's reset report (`reset_record`), print it over RTT, and spin until
/// the watchdog resets the chip.  Spinning rather than resetting at once
/// keeps a panic that recurs on every boot to one reset per watchdog
/// timeout, slow enough for a probe to attach.
#[cfg(all(target_arch = "arm", target_os = "none"))]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    let (line, file) = info
        .location()
        .map(|l| (l.line(), l.file()))
        .unwrap_or((0, ""));
    reset_record::record_panic(line, file);
    rtt_target::rprintln!("{}", info);
    loop {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}

pub use embedded_nano_mesh::{LifeTimeType, SendError};
pub use io::LoraIo;
pub use node::{MeshMessage, MeshNode};
pub use ota::OtaReceiver;
pub use ota_sender::OtaSender;
