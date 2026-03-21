//! Safe wrappers around the RadioHead mesh routing stack.

use crate::driver::{
    trampoline_max_message_length, trampoline_poll_recv, trampoline_send, RadioDriver,
};
use crate::ffi;
use core::ffi::c_void;

/// RadioHead router error codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MeshError {
    InvalidLength = 1,
    NoRoute = 2,
    Timeout = 3,
    NoReply = 4,
    UnableToDeliver = 5,
}

impl MeshError {
    fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => None, // success
            1 => Some(Self::InvalidLength),
            2 => Some(Self::NoRoute),
            3 => Some(Self::Timeout),
            4 => Some(Self::NoReply),
            5 => Some(Self::UnableToDeliver),
            _ => Some(Self::UnableToDeliver),
        }
    }
}

/// A received mesh message.
#[derive(Debug)]
pub struct Message {
    pub len: u8,
    pub source: u8,
    pub dest: u8,
    pub id: u8,
    pub flags: u8,
}

/// Handle to the RadioHead mesh networking stack.
///
/// There can only be **one** `RhMesh` instance at a time (backed by
/// static C++ storage).  The `driver` reference must remain valid and
/// un-moved for the lifetime of this handle.
pub struct RhMesh {
    _not_send: core::marker::PhantomData<*mut ()>,
}

impl RhMesh {
    /// Create and initialise the mesh stack.
    ///
    /// # Safety
    ///
    /// * `driver` must remain valid and pinned for the lifetime of this
    ///   `RhMesh`.  In practice, store it in a `static` or leak it.
    /// * Only one `RhMesh` may exist at a time.
    pub unsafe fn new<D: RadioDriver>(driver: &mut D, this_address: u8) -> Self {
        let vtable = ffi::RhDriverVtable {
            poll_recv: trampoline_poll_recv::<D>,
            send: trampoline_send::<D>,
            max_message_length: trampoline_max_message_length::<D>,
        };
        unsafe {
            ffi::rh_create(&vtable, driver as *mut D as *mut c_void, this_address);
        }
        Self {
            _not_send: core::marker::PhantomData,
        }
    }

    /// Initialise the underlying RadioHead stack.  Call once after `new()`.
    pub fn init(&mut self) -> bool {
        unsafe { ffi::rh_init() }
    }

    // ---- Mesh send / recv -------------------------------------------

    /// Send a message to `dest` via the mesh network.
    ///
    /// Blocks until acknowledged by the next hop (not the final destination).
    /// Performs automatic route discovery if no route is known.
    pub fn send(&mut self, data: &[u8], dest: u8) -> Result<(), MeshError> {
        self.send_with_flags(data, dest, 0)
    }

    /// Send with application-level flags (lower 4 bits only).
    pub fn send_with_flags(
        &mut self,
        data: &[u8],
        dest: u8,
        flags: u8,
    ) -> Result<(), MeshError> {
        let code = unsafe {
            ffi::rh_mesh_send(data.as_ptr() as *mut u8, data.len() as u8, dest, flags)
        };
        match MeshError::from_code(code) {
            None => Ok(()),
            Some(e) => Err(e),
        }
    }

    /// Non-blocking receive.  Call this frequently in your main loop.
    ///
    /// Returns `Some(msg)` if a message addressed to this node was
    /// received; the payload is written into `buf[..msg.len]`.
    /// Also handles routing of messages destined for other nodes.
    pub fn recv(&mut self, buf: &mut [u8]) -> Option<Message> {
        let mut len = buf.len() as u8;
        let mut source = 0u8;
        let mut dest = 0u8;
        let mut id = 0u8;
        let mut flags = 0u8;

        let ok = unsafe {
            ffi::rh_mesh_recv(
                buf.as_mut_ptr(),
                &mut len,
                &mut source,
                &mut dest,
                &mut id,
                &mut flags,
            )
        };

        if ok {
            Some(Message {
                len,
                source,
                dest,
                id,
                flags,
            })
        } else {
            None
        }
    }

    /// Blocking receive with timeout in milliseconds.
    pub fn recv_timeout(&mut self, buf: &mut [u8], timeout_ms: u16) -> Option<Message> {
        let mut len = buf.len() as u8;
        let mut source = 0u8;
        let mut dest = 0u8;
        let mut id = 0u8;
        let mut flags = 0u8;

        let ok = unsafe {
            ffi::rh_mesh_recv_timeout(
                buf.as_mut_ptr(),
                &mut len,
                timeout_ms,
                &mut source,
                &mut dest,
                &mut id,
                &mut flags,
            )
        };

        if ok {
            Some(Message {
                len,
                source,
                dest,
                id,
                flags,
            })
        } else {
            None
        }
    }

    // ---- Reliable datagram (point-to-point, bypasses routing) --------

    /// Send directly to `address` with ACK/retry (no mesh routing).
    pub fn reliable_send(&mut self, data: &[u8], address: u8) -> bool {
        unsafe { ffi::rh_reliable_send(data.as_ptr() as *mut u8, data.len() as u8, address) }
    }

    // ---- Route management -------------------------------------------

    /// Manually add a route: to reach `dest`, send via `next_hop`.
    pub fn add_route(&mut self, dest: u8, next_hop: u8) {
        unsafe { ffi::rh_router_add_route(dest, next_hop) }
    }

    /// Delete the route to `dest`.  Returns true if the route existed.
    pub fn delete_route(&mut self, dest: u8) -> bool {
        unsafe { ffi::rh_router_delete_route(dest) }
    }

    /// Clear the entire routing table.
    pub fn clear_routes(&mut self) {
        unsafe { ffi::rh_router_clear_routes() }
    }

    /// Set the maximum hop count before a routed message is dropped.
    pub fn set_max_hops(&mut self, max_hops: u8) {
        unsafe { ffi::rh_router_set_max_hops(max_hops) }
    }

    // ---- Configuration ----------------------------------------------

    /// Change this node's address.
    pub fn set_this_address(&mut self, address: u8) {
        unsafe { ffi::rh_set_this_address(address) }
    }

    /// Set the retransmit timeout (ms) for reliable datagrams.
    pub fn set_timeout(&mut self, timeout_ms: u16) {
        unsafe { ffi::rh_reliable_set_timeout(timeout_ms) }
    }

    /// Set the maximum number of retransmit attempts.
    pub fn set_retries(&mut self, retries: u8) {
        unsafe { ffi::rh_reliable_set_retries(retries) }
    }

    // ---- Statistics -------------------------------------------------

    pub fn rx_good(&self) -> u16 {
        unsafe { ffi::rh_driver_rx_good() }
    }
    pub fn rx_bad(&self) -> u16 {
        unsafe { ffi::rh_driver_rx_bad() }
    }
    pub fn tx_good(&self) -> u16 {
        unsafe { ffi::rh_driver_tx_good() }
    }
    pub fn last_rssi(&self) -> i16 {
        unsafe { ffi::rh_driver_last_rssi() }
    }
    pub fn retransmissions(&self) -> u32 {
        unsafe { ffi::rh_reliable_retransmissions() }
    }
}
