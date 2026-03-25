//! Compile-time radio and mesh configuration.

/// Polling loop timeout for TX completion (ms).
///
/// `send()` blocks polling the IRQ register until `TxDone` fires or this
/// deadline is reached.  For SF7/BW125 a ~40-byte packet takes ~50–80 ms
/// on air, so 150 ms gives comfortable headroom while failing fast.
pub const TX_POLL_TIMEOUT_MS: u64 = 150;

/// Chip-level TX timeout passed to `SetTx` (ms).
///
/// The SX1262 will abort TX and raise a Timeout IRQ if this expires.
/// Must be longer than `TX_POLL_TIMEOUT_MS` so the polling loop always
/// exits first and we remain in control of the state machine.
pub const TX_CHIP_TIMEOUT_MS: u64 = 300;

/// How long the mesh node listens before transmitting queued packets (ms).
///
/// Must exceed the on-air time of the longest expected packet so that a node
/// can detect a concurrent transmission before it starts its own.  At
/// SF7/BW125 a ~40-byte nano-mesh packet takes ~75–100 ms on air; 200 ms
/// gives a 2× margin.
pub const MESH_LISTEN_PERIOD_MS: u32 = 200;

/// Hop-count lifetime for broadcast packets.
///
/// Each hop decrements the counter; a packet with lifetime 0 is not
/// forwarded.  For a 2-node direct link, 1 is sufficient — the originator
/// transmits once and the peer receives it without re-broadcasting.
/// Increase if you add intermediate nodes that need to forward packets.
pub const BROADCAST_LIFETIME: u8 = 1;

/// Current firmware version.
///
/// Bumped on each release.  The OTA receiver rejects offers whose version
/// is less than or equal to this value (downgrade prevention).
/// Set at compile time via the `FW_VERSION` environment variable, e.g.:
///   FW_VERSION=2 cargo build --release
/// Defaults to 1 if not specified.
pub const FIRMWARE_VERSION: u16 = {
    match option_env!("FW_VERSION") {
        Some(s) => {
            let bytes = s.as_bytes();
            assert!(!bytes.is_empty(), "FW_VERSION must not be empty");
            let mut i = 0;
            let mut n: u16 = 0;
            while i < bytes.len() {
                let d = bytes[i];
                assert!(d >= b'0' && d <= b'9', "FW_VERSION must be a number 0-65535");
                let next = n as u32 * 10 + (d - b'0') as u32;
                assert!(next <= 65535, "FW_VERSION must be 0-65535");
                n = next as u16;
                i += 1;
            }
            n
        }
        None => 1,
    }
};

/// This node's mesh address.
/// Set at compile time via the `ADDRESS` environment variable, e.g.:
///   ADDRESS=2 cargo run --release
/// Defaults to 1 if not specified.
pub const THIS_ADDRESS: u8 = {
    match option_env!("ADDRESS") {
        Some(s) => {
            let bytes = s.as_bytes();
            assert!(!bytes.is_empty(), "ADDRESS must not be empty");
            let mut i = 0;
            let mut n: u8 = 0;
            while i < bytes.len() {
                let d = bytes[i];
                assert!(d >= b'0' && d <= b'9', "ADDRESS must be a number 0-255");
                let next = n as u16 * 10 + (d - b'0') as u16;
                assert!(next <= 255, "ADDRESS must be 0-255");
                n = next as u8;
                i += 1;
            }
            n
        }
        None => 1,
    }
};
