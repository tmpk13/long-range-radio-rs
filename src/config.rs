//! Compile-time radio and mesh configuration.

/// LoRa range/airtime preset: spreading factor, bandwidth and coding rate.
///
/// Higher presets trade throughput and on-air time for receiver sensitivity
/// (link budget). On-air time grows roughly as 2^SF, so the TX timeouts and
/// the mesh listen window below are scaled to match the selected preset.
///
/// | Preset    | SF   | BW       | CR  | Rx sens.  | ~Range | Airtime ~40 B |
/// |-----------|------|----------|-----|-----------|--------|---------------|
/// | `Fast`    | SF7  | 125 kHz  | 4/5 | -124 dBm  | 1x     | ~0.1 s        |
/// | `Long`    | SF10 | 125 kHz  | 4/5 | -132 dBm  | ~2.5x  | ~0.6 s        |
/// | `Max`     | SF12 | 125 kHz  | 4/8 | -137 dBm  | ~4x    | ~3.3 s        |
/// | `Extreme` | SF12 | 62.5 kHz | 4/8 | -140 dBm  | ~5.5x  | ~6.6 s        |
///
/// Every preset transmits at the maximum +22 dBm; only the modulation
/// changes. Both ends of a link must use the same preset to communicate.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum RadioPreset {
    /// SF7 / BW125 / CR4-5 - fastest, shortest range.
    Fast,
    /// SF10 / BW125 / CR4-5 - +8 dB sensitivity, ~2.5x range.
    Long,
    /// SF12 / BW125 / CR4-8 - +12.5 dB sensitivity, ~4x range.
    Max,
    /// SF12 / BW62.5 / CR4-8 - +15.5 dB sensitivity, ~5.5x range; needs the TCXO.
    Extreme,
}

impl RadioPreset {
    /// Human-readable name for boot logging.
    pub const fn name(self) -> &'static str {
        match self {
            RadioPreset::Fast => "fast (SF7/BW125/CR4-5)",
            RadioPreset::Long => "long (SF10/BW125/CR4-5)",
            RadioPreset::Max => "max (SF12/BW125/CR4-8)",
            RadioPreset::Extreme => "extreme (SF12/BW62.5/CR4-8)",
        }
    }
}

/// Active LoRa preset selecting the range/airtime trade-off.
///
/// Set at compile time via the `LORA_PRESET` environment variable, e.g.:
///   LORA_PRESET=max cargo run --release
/// Valid values: `fast` (default), `long`, `max`, `extreme`.
pub const RADIO_PRESET: RadioPreset = match option_env!("LORA_PRESET") {
    None => RadioPreset::Fast,
    Some(s) => match s.as_bytes() {
        b"fast" => RadioPreset::Fast,
        b"long" => RadioPreset::Long,
        b"max" => RadioPreset::Max,
        b"extreme" => RadioPreset::Extreme,
        _ => panic!("LORA_PRESET must be one of: fast, long, max, extreme"),
    },
};

/// Whether the receiver runs at boosted gain.
///
/// The SX126x powers up with the RxGain register selecting power-saving gain;
/// boosting it buys roughly 2 dB of sensitivity, which is worth about the
/// same as one step of [`RADIO_PRESET`] but costs no airtime at all. What it
/// does cost is receive current, continuously, on a node that is listening
/// most of the time - so on solar or battery it is a trade to opt into rather
/// than inherit, and the default matches the chip.
///
/// Set at compile time via the `RX_BOOST` environment variable, e.g.:
///   RX_BOOST=1 cargo run --release
/// Valid values: `0` (default), `1`.
pub const RX_BOOST: bool = match option_env!("RX_BOOST") {
    None => false,
    Some(s) => match s.as_bytes() {
        b"0" => false,
        b"1" => true,
        _ => panic!("RX_BOOST must be 0 or 1"),
    },
};

/// Polling loop timeout for TX completion (ms).
///
/// `send()` blocks polling the IRQ register until `TxDone` fires or this
/// deadline is reached. Scaled per [`RADIO_PRESET`] to cover the packet
/// on-air time (which grows ~2^SF) with headroom while still failing fast.
pub const TX_POLL_TIMEOUT_MS: u64 = match RADIO_PRESET {
    RadioPreset::Fast => 150,
    RadioPreset::Long => 1_000,
    RadioPreset::Max => 4_000,
    RadioPreset::Extreme => 7_500,
};

/// Chip-level TX timeout passed to `SetTx` (ms).
///
/// The SX1262 will abort TX and raise a Timeout IRQ if this expires.
/// Must be longer than `TX_POLL_TIMEOUT_MS` so the polling loop always
/// exits first and we remain in control of the state machine.
pub const TX_CHIP_TIMEOUT_MS: u64 = match RADIO_PRESET {
    RadioPreset::Fast => 300,
    RadioPreset::Long => 1_300,
    RadioPreset::Max => 4_500,
    RadioPreset::Extreme => 8_200,
};

/// How long the mesh node listens before transmitting queued packets (ms).
///
/// Must exceed the on-air time of the longest expected packet so that a node
/// can detect a concurrent transmission before it starts its own. Scaled per
/// [`RADIO_PRESET`] alongside the packet airtime.
pub const MESH_LISTEN_PERIOD_MS: u32 = match RADIO_PRESET {
    RadioPreset::Fast => 200,
    RadioPreset::Long => 900,
    RadioPreset::Max => 4_000,
    RadioPreset::Extreme => 7_000,
};

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
                assert!(
                    d >= b'0' && d <= b'9',
                    "FW_VERSION must be a number 0-65535"
                );
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
