//! Packet-oriented radio driver trait.
//!
//! Board-specific crates implement this for their radio hardware
//! (SX1262, RFM95, etc.), keeping the mesh layer board-agnostic.

/// A packet-oriented radio interface.
///
/// This is the abstraction boundary between the mesh layer and
/// specific radio hardware.
pub trait PacketRadio {
    /// Error type for radio operations.
    type Error: core::fmt::Debug;

    /// Poll for a received packet (non-blocking).
    ///
    /// If a packet is available, write it into `buf` and return
    /// `Ok(Some((bytes_written, rssi_dbm)))`.
    /// If nothing is available, return `Ok(None)`.
    fn poll_recv(&mut self, buf: &mut [u8]) -> Result<Option<(usize, i16)>, Self::Error>;

    /// Transmit a raw packet. Blocks until transmission completes.
    fn send(&mut self, data: &[u8]) -> Result<(), Self::Error>;

    /// Maximum packet size in bytes.
    fn max_packet_len(&self) -> usize;
}
