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
