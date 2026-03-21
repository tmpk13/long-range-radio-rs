//! Raw FFI bindings to the C shim (csrc/rh_shim.h).

#![allow(dead_code)]

use core::ffi::c_void;

/// Driver callback vtable passed to the C++ RustDriver.
#[repr(C)]
pub struct RhDriverVtable {
    pub poll_recv: unsafe extern "C" fn(
        ctx: *mut c_void,
        buf: *mut u8,
        len: *mut u8,
        rssi: *mut i16,
    ) -> bool,
    pub send:
        unsafe extern "C" fn(ctx: *mut c_void, data: *const u8, len: u8) -> bool,
    pub max_message_length: unsafe extern "C" fn(ctx: *mut c_void) -> u8,
}

extern "C" {
    // Lifecycle
    pub fn rh_create(vtable: *const RhDriverVtable, ctx: *mut c_void, this_address: u8);
    pub fn rh_init() -> bool;

    // Mesh
    pub fn rh_mesh_send(buf: *mut u8, len: u8, dest: u8, flags: u8) -> u8;
    pub fn rh_mesh_recv(
        buf: *mut u8,
        len: *mut u8,
        source: *mut u8,
        dest: *mut u8,
        id: *mut u8,
        flags: *mut u8,
    ) -> bool;
    pub fn rh_mesh_recv_timeout(
        buf: *mut u8,
        len: *mut u8,
        timeout: u16,
        source: *mut u8,
        dest: *mut u8,
        id: *mut u8,
        flags: *mut u8,
    ) -> bool;

    // Reliable datagram (point-to-point)
    pub fn rh_reliable_send(buf: *mut u8, len: u8, address: u8) -> bool;
    pub fn rh_reliable_recv(
        buf: *mut u8,
        len: *mut u8,
        from: *mut u8,
        to: *mut u8,
        id: *mut u8,
        flags: *mut u8,
    ) -> bool;

    // Configuration
    pub fn rh_set_this_address(address: u8);
    pub fn rh_reliable_set_timeout(timeout_ms: u16);
    pub fn rh_reliable_set_retries(retries: u8);
    pub fn rh_router_add_route(dest: u8, next_hop: u8);
    pub fn rh_router_delete_route(dest: u8) -> bool;
    pub fn rh_router_clear_routes();
    pub fn rh_router_set_max_hops(max_hops: u8);

    // Statistics
    pub fn rh_driver_rx_good() -> u16;
    pub fn rh_driver_rx_bad() -> u16;
    pub fn rh_driver_tx_good() -> u16;
    pub fn rh_driver_last_rssi() -> i16;
    pub fn rh_reliable_retransmissions() -> u32;
}
