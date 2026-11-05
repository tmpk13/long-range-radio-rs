# Mesh timing

## TX/RX Timing: Standby Gap and Listen Period

After TX completes the SX1262 returns to standby automatically. If the driver
leaves `rx_active = false` and returns, the radio sits in standby until the next
`poll_recv` call. During that window `embedded-nano-mesh`'s listen period timer
is ticking but the radio isn't actually listening - so both nodes can believe the
channel is idle and transmit simultaneously.

Fix: at the end of `send()`, immediately call `set_rx(continuous_rx)` and set
`rx_active = true` so the listen period measures real channel activity from the
moment TX ends.

Second factor: `MESH_LISTEN_PERIOD_MS = 50` was shorter than the ~75-100 ms
on-air time of a nano-mesh packet at SF7/BW125. A listen window shorter than one
packet's air time cannot reliably detect a concurrent transmission. Raised to
200 ms (~ 2x air time).

Third factor: `lifetime = 3` on a 2-node link caused each broadcast to generate
up to 6 TX events in rapid succession (originator + forwards), multiplying the
collision window. Set `BROADCAST_LIFETIME = 1` for direct neighbors; increase
if intermediate forwarding hops are needed.

## DWT millis() Wrap -> SendingQueueIsFull After ~18 Minutes

The original `millis()` implementation divided the raw 32-bit DWT cycle
counter by `SYSCLK_HZ / 1000`.  At 4 MHz the DWT counter wraps at
`2^32 / 4000 ~ 1 073 741 ms ~ 17.9 minutes`.

`embedded-nano-mesh`'s internal timer checks:
```rust
current_time > last_speak_time + listen_period
```
After the DWT wrap, `current_time` resets to 0 while `last_speak_time`
stays near 1 073 741.  The sum `last_speak_time + listen_period ~ 1 073 941`
is a valid u32, but `current_time` can never reach it - `is_time_to_speak`
returns `false` permanently and the transmit queue stops draining.  With
5 slots (`PACKET_QUEUE_SIZE = 5`) and a 10 s heartbeat, the queue fills in
~50 s and every subsequent `broadcast()` returns `SendingQueueIsFull`.

Fix: use `wrapping_sub` on successive DWT readings and accumulate elapsed
ms, so `millis()` returns a monotonically-increasing u32 that wraps only
at ~49 days.  This lives in `firmware/src/platform.rs`.

## Broadcast Collision Risk

Both nodes boot and start a 10 s TX timer simultaneously. Because LoRa is
half-duplex, simultaneous TX means both packets are lost. Mitigations:

- Stagger first TX by address: `next_tx = now + address * N seconds`
- Add random jitter to each subsequent interval (e.g. 0-3 s)
- `embedded-nano-mesh` has a built-in listen period before transmitting,
  which also helps break symmetry
