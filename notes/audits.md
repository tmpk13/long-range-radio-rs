# Bug sweeps ported from esp32c6-gps

## Radio fixes ported back from the esp32c6-gps derivative

That project started as a copy of this one and its radio was debugged much
further in the field. These are the bug fixes brought back; the mesh layer
was deliberately left alone.

- **RF antenna switch (PA4/PA5) was never driven.** The single biggest one.
  The LoRa-E5 puts the antenna switch inside the module and the radio die has
  no bonded DIO2, so `SetDio2AsRfSwitchCtrl` does not exist here - the MCU has
  to select the path itself. The pins sat in their reset state (analog in),
  the switch floated, and signal reached the antenna only through the
  switch's off-state isolation: tens of dB down, which works across a bench
  and nowhere else. `RfSwitch` in radio.rs now owns the pins and points them
  before every SetTx and SetRx, isolating the antenna in standby and while
  `init` reconfigures the PA. Both low = isolated, ctrl1 high = RX,
  ctrl2 high = TX (high-power PA).
- **`set_rx(Timeout::DISABLED)` armed single-shot RX, not continuous.** On the
  SX126x the SetRx timeout doubles as a mode select: 0x000000 is single mode
  (receiver drops to the fallback mode after one packet), 0xFFFFFF is
  continuous. `Timeout::DISABLED` is 0, so a node went deaf after its first
  received packet until its own next transmit re-armed RX. Now `RX_CONTINUOUS`.
- **TX left the receive payload ceiling at the last frame's length.** With an
  explicit header the packet-params payload length is the largest payload the
  receiver will *accept*; transmit has to narrow it to the frame being sent.
  Re-entering RX without restoring 255 caps the receiver at the size of this
  node's own last transmission. All RX arming now goes through `enter_rx`.
- **Bad-CRC packets were handed up as valid.** The chip raises RxDone
  alongside Err on a CRC failure with the corrupt payload still in the
  buffer, and nothing above the driver checksums. Err is now enabled in the
  IRQ mask and those packets are dropped.
- **No recalibration after the TCXO came up.** The automatic power-up
  calibration runs before the TCXO is enabled, so RC64k/RC13M/PLL/ADC/image
  were all derived from a clock that was not running - frequency error and
  lost sensitivity, with no failure reported. `calibrate(0x7F)` now runs
  after `set_tcxo_mode`.
- **SMPS clock detection.** Must be enabled *before* the SMPS is selected.
  The HAL's `set_smps_clock_det_en` writes the whole register and would clear
  the rest of the regulator config, so this is a read-modify-write through
  the raw `read_reg`/`write_reg` helpers.
- **The two SX126x transmit errata.** TX clamp (0x08D8 bits 4:1) for PA
  tolerance of antenna mismatch, and TX modulation (0x0889 bit 2) which must
  track the LoRa bandwidth. `stm32wlxx-hal` keeps its register table private,
  hence `subghz_xfer` driving SUBGHZSPI directly.
- **Image calibration is per-band.** Was hardcoded to 902-928; now derived
  from the configured frequency via `image_band`.
- **`print_diagnostics` reads GetError.** The status byte reports the mode
  the radio is in, not whether it got there intact - a TCXO that never
  started or a PLL that never locked still reports a healthy standby.
- **Bounded the NMEA drain** (`DRAIN_BUDGET` in gpslog.rs). With no GPS
  attached the floating USART1 RX pin can stream noise that never completes
  a sentence, and the unbounded drain loop could monopolize the main loop and
  starve the radio poll.

Deliberately **not** ported: the derivative moved to the private LoRa sync
word (0x1424). It is a real improvement - on the public word the receiver
locks onto every LoRaWAN preamble in earshot - but nodes on different sync
words cannot hear each other at all, so it is a flag day for the whole
fleet. Also skipped: the runtime radio config, tx-only/rx-only roles and the
SF12/BW500 defaults, all of which are behavior changes rather than fixes.
(RxBoost was on that list too; it has since been added deliberately - see
below.)

## Second sweep ported from esp32c6-gps: five of nine

The derivative did a whole-system bug sweep. Four of its nine findings have
no counterpart here - this repo has no RADIO.CFG (config is `option_env!`
constants), its GPS is RX-only and never sleeps, there is no ESP companion
holding sleep flags, and there is no CFG_END opcode. The rest applied, in
two cases worse than upstream.

### The watchdog was shorter than the radio's own TX deadline

`watchdog::start(&iwdg, 5_000)` against a `TX_POLL_TIMEOUT_MS` that scales
with the preset:

| Preset | poll deadline | vs 5 s watchdog |
|-|-|-|
| fast | 150 ms | safe |
| long | 1000 ms | safe |
| max | 4000 ms | inside LSI tolerance |
| extreme | 7500 ms | resets before the poll gives up |

`send()` blocks in that loop, so on `extreme` a transmit that never
completes resets the board rather than returning `Timeout`, and on `max` the
margin is inside the STM32WL's own LSI spread (at the 47 kHz end the real
timeout is ~4.7 s, not 5). Upstream hit the same thing against a 6 s
watchdog.

Rather than stretch the timeout, which would blunt it everywhere, the
bounded waits feed themselves through the new `watchdog::feed_now` - a raw
write of the reload key, safe from anywhere because the key register is
write-only and the reload is idempotent. It says "still waiting on something
that will time out", not "trust me". Three callers: the radio TX poll, the
SD `wait_not_busy` (500 ms per block, and a log flush writes several between
two of the main loop's feeds), and the ACMD41 loop in card init (1 s).

`wait_on_busy` in the radio driver is deliberately *not* fed - it has no
deadline of its own, and a radio that never releases BUSY is exactly what
the watchdog is for. `read_block`'s data-token wait is also left alone: at
200 ms it cannot overrun, and every feed is a place the watchdog is
suppressed.

### An abandoned OTA transfer took the node off the air for good

`handle_offer` answers `reason::BUSY` while a transfer is active, and only
an explicit `OTA_ABORT` cleared it - so a sender that crashed or walked out
of range mid-upload left the node refusing every future offer until someone
power-cycled it. `Receiving` now stamps `last_ms` per frame and
`OtaReceiver::expire`, called from the main loop, drops the transfer after
60 s of silence. The stamp happens before the duplicate and out-of-order
returns, so a sender that is retrying counts as present; only silence
expires. 60 s is far longer than any legitimate gap, where even a single
`extreme` packet is ~6.6 s on the air.

Not ported: upstream also made its END opcodes idempotent so a host retry
re-reports the real error. There is no separate END here - the final chunk
verifies the CRC and calls `request_swap` - so a lost `COMPLETE` reaches a
node that is already rebooting into the new firmware. Same caveat upstream
records for its FW_END half: making that unambiguous needs the sender to
re-query the version rather than retry, which is a larger protocol change
than the bug warrants.

### Smaller

- **First-beacon stagger scaled with the whole address**, so node 200 sat
  silent for ten minutes after boot with nothing on the console to say why
  (upstream's was 3 min 22 s). Folded into eight slots, `address % 8` times
  1 s: a second apart is already wide against a packet's air time, and the
  post-transmit jitter keeps nodes apart from there.
- **The bootloader read u32s through an unaligned pointer.** `flash_program`
  did `ptr::read(src as *const u32)`; `copy_page` passes an `align(8)`
  buffer, but `write_state` passes a bare `[u8; 8]` stack local with
  alignment 1. UB, and the compiler is free to answer it with an `ldrd`,
  which faults - in the code that writes the boot state, with no way back.
  Now assembled with `u32::from_le_bytes`, which costs nothing where LLVM
  can prove the alignment.
- **`platform::random` could return below `min`.** `(s as i32)
  .unsigned_abs()` is 2^31 for exactly one state value, which casts back
  negative, and `%` keeps the sign of its left operand. One call in 2^32,
  and the effect was only a repeat jitter that read as already due, but the
  cast was wrong. Reduced in unsigned space instead.
