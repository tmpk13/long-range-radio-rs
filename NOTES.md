# Development Notes

## Debug Mode

A `debug` cargo feature gates verbose output via a `debug_println!` macro.
`#[cfg]` on expressions is not stable Rust, so the macro uses `cfg!()` inside
a block (evaluates at compile time but keeps the expression valid in all
positions including match arms).

Enable with:
```
cargo run --release --features debug
```

## TX Packet Params Bug

When sending, the packet params must be set with the **actual payload length**
and the **full set of LoRa params** (header type, invert IQ, etc.) to match
init config — not just length and CRC. Omitting fields caused the chip to
enter TX but never fire TxDone, producing a 4s+ spin before timeout.

## Async / Non-Blocking TX

The mesh layer calls `send()` synchronously and blocks until the packet is
acknowledged (retries, ACKs). Making the radio driver non-blocking wouldn't
help because the layer above it still blocks. A proper fix would require:

- Replacing the mesh library with one that supports a poll/yield model, or
- Running the mesh on a dedicated thread/core (ESP32-C3 is single-core, so
  not available here)

The practical fix is keeping the blocking model with a short TX timeout
(~500 ms for SF7/BW125 which completes in <100 ms in practice).

## Seeed XIAO SX1262 Module — TCXO Required

The Seeed XIAO SX1262 module uses a TCXO (temperature-compensated crystal
oscillator) powered via DIO3. If `tcxo_opts` is set to `None` in the sx126x
config, the radio initialises successfully (SPI works, chip enters StbyRC,
no calibration errors) but **can never transmit or receive RF** — the PLL has
no reference clock.

Symptom: every TX times out, `tx_good` stays 0, `rx_good` stays 0.
Fix: configure `tcxo_opts: Some((TcxoVoltage::Volt1_8, TcxoDelay::from_ms(10)))`.

The 1.8 V / 10 ms values are correct for the Seeed module.

## Diagnosing a Dead Radio (all-0xFF SPI responses)

If `get_status()` returns `0b11111111` and all error flags are true, the SX1262
is not responding over SPI at all — MISO is floating high. Common causes:

1. Module not seated — reseat it physically
2. Wrong board flashed (binary for a different address/config)
3. Genuine hardware fault — swap the module between boards to isolate

A healthy init shows `chip_mode: Some(StbyRC)` with all error flags false.

## sx126x Crate: Missing wait_on_busy() — TxDone Never Fires

The sx126x 0.3.0 crate's `set_standby()`, `set_tx()`, and `set_rx()` do **not**
call `wait_on_busy()` internally. The SX1262 datasheet states that any SPI
command sent while BUSY is high is silently ignored by the chip.

Critical path in `send()`:

1. `set_tx()` is issued — the chip starts TCXO startup (10 ms per
   `tcxo_opts`) with BUSY high.
2. Without `wait_on_busy()`, the polling loop immediately calls
   `get_irq_status()` while BUSY is still high.
3. The chip silently ignores those reads; TxDone fires and completes
   inside the BUSY window and is never seen by the poll loop.
4. 150/500 ms software timeout fires; TX appears to have failed.

Symptom: every `send()` times out despite `chip_mode: Some(TX)` appearing in
the debug status; `command_status: None` (RFU/0b001) in the status byte is a
secondary indicator that the chip was still busy when the status was read.

Fix: call `self.radio.wait_on_busy()` after `set_standby()`, after `set_tx()`
(before the IRQ polling loop), and after `set_rx()` wherever it is called.
The same applies to `poll_recv()` — wait after `set_rx()` before reading IRQ.

## TX/RX Timing: Standby Gap and Listen Period

After TX completes the SX1262 returns to standby automatically. If the driver
leaves `rx_active = false` and returns, the radio sits in standby until the next
`poll_recv` call. During that window `embedded-nano-mesh`'s listen period timer
is ticking but the radio isn't actually listening — so both nodes can believe the
channel is idle and transmit simultaneously.

Fix: at the end of `send()`, immediately call `set_rx(continuous_rx)` and set
`rx_active = true` so the listen period measures real channel activity from the
moment TX ends.

Second factor: `MESH_LISTEN_PERIOD_MS = 50` was shorter than the ~75-100 ms
on-air time of a nano-mesh packet at SF7/BW125. A listen window shorter than one
packet's air time cannot reliably detect a concurrent transmission. Raised to
200 ms (~ 2× air time).

Third factor: `lifetime = 3` on a 2-node link caused each broadcast to generate
up to 6 TX events in rapid succession (originator + forwards), multiplying the
collision window. Set `BROADCAST_LIFETIME = 1` for direct neighbours; increase
if intermediate forwarding hops are needed.

## Broadcast Collision Risk

Both nodes boot and start a 10 s TX timer simultaneously. Because LoRa is
half-duplex, simultaneous TX means both packets are lost. Mitigations:

- Stagger first TX by address: `next_tx = now + address * N seconds`
- Add random jitter to each subsequent interval (e.g. 0–3 s)
- `embedded-nano-mesh` has a built-in listen period before transmitting,
  which also helps break symmetry

## Architecture

The radio driver implements a `PacketRadio` trait used by the mesh layer.
The driver manages RX/TX state — entering continuous RX on the first poll,
transitioning to standby before TX, then polling the IRQ register for
`TxDone` or `Timeout` rather than blocking on the DIO1 pin.
