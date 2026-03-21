//! RadioDriver trait and FFI trampoline functions.

use core::ffi::c_void;

/// Trait that users implement to bridge their radio hardware to RadioHead.
///
/// All packets are raw over-the-air format: a 4-byte RadioHead header
/// (`[TO, FROM, ID, FLAGS]`) followed by the payload.  The C++ routing
/// stack handles header construction and parsing internally.
pub trait RadioDriver {
    /// Poll for a received packet (non-blocking).
    ///
    /// If a packet is available, write it into `buf` and return
    /// `Some((total_bytes_written, rssi_dbm))`.
    /// If nothing is available, return `None`.
    fn poll_recv(&mut self, buf: &mut [u8]) -> Option<(u8, i16)>;

    /// Transmit a raw packet.  Should block until transmission is complete.
    fn send(&mut self, data: &[u8]) -> bool;

    /// Maximum raw packet size the radio supports (including the 4-byte
    /// RadioHead header).  For SX1262 this is typically 255.
    fn max_message_length(&self) -> u8;
}

// ---- Trampoline functions -------------------------------------------
// These are monomorphised for each concrete `D: RadioDriver` and passed
// to the C++ side as function pointers.

pub(crate) unsafe extern "C" fn trampoline_poll_recv<D: RadioDriver>(
    ctx: *mut c_void,
    buf: *mut u8,
    len: *mut u8,
    rssi: *mut i16,
) -> bool {
    let driver = unsafe { &mut *(ctx as *mut D) };
    let max = unsafe { *len } as usize;
    let slice = unsafe { core::slice::from_raw_parts_mut(buf, max) };

    match driver.poll_recv(slice) {
        Some((n, r)) => {
            unsafe {
                *len = n;
                *rssi = r;
            }
            true
        }
        None => false,
    }
}

pub(crate) unsafe extern "C" fn trampoline_send<D: RadioDriver>(
    ctx: *mut c_void,
    data: *const u8,
    len: u8,
) -> bool {
    let driver = unsafe { &mut *(ctx as *mut D) };
    let slice = unsafe { core::slice::from_raw_parts(data, len as usize) };
    driver.send(slice)
}

pub(crate) unsafe extern "C" fn trampoline_max_message_length<D: RadioDriver>(
    ctx: *mut c_void,
) -> u8 {
    let driver = unsafe { &*(ctx as *const D) };
    driver.max_message_length()
}
