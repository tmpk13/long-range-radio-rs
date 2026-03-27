# Development Notes

## IWDG SR Wait Hangs — Silent Reset Loop (No RTT Output)

After adding watchdog support, the firmware appeared to "do nothing" after
flashing — no RTT output, no display, no radio activity.

Root cause: `watchdog::start()` polled `while iwdg.sr.read().bits() != 0 {}`
which checks **all** SR bits. On STM32WL the IWDG_SR has a WVU (window
value update) bit 2 that can remain set if the WINR register was never
written. The loop hung indefinitely. Meanwhile the IWDG was already ticking
with its default ~512 ms timeout (prescaler /4, reload 0xFFF), so the MCU
reset before any `rprintln!` was reached. Result: infinite silent reboot loop.

Fix: only wait on PVU (bit 0) and RVU (bit 1):
```rust
while iwdg.sr.read().bits() & 0x03 != 0 {}
```

## Stack Overflow from OtaReceiver page_buf (HardFault in RTT write_str)

After adding OTA support, the firmware crashed immediately on boot with a
HardFault inside `rtt-target`'s `write_str`. probe-rs reported
"Firmware exited unexpectedly: Multiple" and the stack unwinder failed
(CFA = None), indicating a corrupted stack pointer.

`clippy::large_stack_frames` confirmed `init()` used **16,951 bytes** of stack.
The `OtaReceiver` struct contained a 2 KB `page_buf: [u8; 2048]` that was
constructed on the stack during `init()` before RTIC moved `Local` into static
storage. Combined with the Ssd1306 framebuffer (1 KB), MeshNode queues (~1.2 KB),
and SubGhz radio temporaries, the stack overflowed and corrupted the RTT control
block in RAM.

Fix: moved `page_buf` to a module-level `static` in `ota.rs`, accessed via
`page_buf()` / `page_buf_mut()` methods. This is safe because `OtaReceiver` is
only used from a single RTIC task on a single-core MCU. Reduced `init()` frame
to ~8,759 bytes and `Local` from 4,128 → 2,080 bytes.

Lesson: on embedded targets, always check `clippy::large_stack_frames` when
adding structs with large inline buffers to RTIC `Local` — the `init()` return
path constructs them on the stack before moving to statics.

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

## DWT millis() Wrap → SendingQueueIsFull After ~18 Minutes

The original `millis()` implementation divided the raw 32-bit DWT cycle
counter by `SYSCLK_HZ / 1000`.  At 4 MHz the DWT counter wraps at
`2^32 / 4000 ≈ 1 073 741 ms ≈ 17.9 minutes`.

`embedded-nano-mesh`'s internal timer checks:
```rust
current_time > last_speak_time + listen_period
```
After the DWT wrap, `current_time` resets to 0 while `last_speak_time`
stays near 1 073 741.  The sum `last_speak_time + listen_period ≈ 1 073 941`
is a valid u32, but `current_time` can never reach it — `is_time_to_speak`
returns `false` permanently and the transmit queue stops draining.  With
5 slots (`PACKET_QUEUE_SIZE = 5`) and a 10 s heartbeat, the queue fills in
~50 s and every subsequent `broadcast()` returns `SendingQueueIsFull`.

Fix: use `wrapping_sub` on successive DWT readings and accumulate elapsed
ms, so `millis()` returns a monotonically-increasing u32 that wraps only
at ~49 days.  This lives in `sx1262-mesh-rs/src/platform.rs`.

## I2C Display Resilience (Non-Blocking / Hot-Plug)

The stm32wlxx-hal I2C driver has **no software timeout** — its internal
`busy_wait!` macro spins indefinitely until a hardware flag fires. When
a device is simply absent (not ACKed), the peripheral's NACK detection
sets the NACKF flag and the driver returns `Error::Nack` quickly. If the
bus is electrically stuck (SDA held low), the spin is truly infinite and
only the 5 s IWDG watchdog can recover the MCU.

To keep the mesh running when the SSD1306 display is disconnected or
fails mid-operation:

- `display_ok: bool` tracks whether the display is reachable.
- On init, `display.init()` + `flush()` results are checked; if either
  fails, `display_ok = false` and the node boots without a display.
- Every `display.flush()` in the main loop is checked; on error the
  flag is cleared and a retry timer starts.
- Every 10 s, if `!display_ok`, the loop re-attempts `display.init()`
  + `flush()`. On success the flag is set and normal display updates
  resume.
- All draw/clear/flush calls are gated behind `if *display_ok { … }`,
  so a missing display adds zero I2C traffic to the bus.

This pattern generalises to multiple I2C devices: give each device its
own `_ok` flag and retry timer so one failing device doesn't block the
others.

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

## stm32wlxx-hal SubGhz API Reference (v0.6.1)

### Feature Flags for STM32WLE5JC

```toml
[dependencies]
stm32wlxx-hal = { version = "0.6.1", features = ["stm32wle5", "rt"] }
```

Available chip features: `stm32wl5x_cm0p`, `stm32wl5x_cm4`, `stm32wle5`.
Other useful features: `rt` (runtime), `defmt`, `chrono`, `embedded-time`.

### SubGhz Initialization

```rust
use stm32wlxx_hal::subghz::SubGhz;

// Without DMA (simplest)
let sg = SubGhz::new(dp.SPI3, &mut dp.RCC);

// With DMA
let sg = SubGhz::new_with_dma(dp.SPI3, miso_dma, mosi_dma, &mut dp.RCC);

// After sleep wakeup (unsafe, skips reset)
let sg = unsafe { SubGhz::new_no_reset(dp.SPI3, &mut dp.RCC) };

// Steal without singleton check (unsafe, for RTIC shared resources)
let sg = unsafe { SubGhz::steal() };
```

### Typical LoRa Configuration Sequence

```rust
use stm32wlxx_hal::subghz::*;

// 1. Standby
sg.set_standby(StandbyClk::Rc)?;

// 2. TCXO and calibration (if board has TCXO)
sg.set_tcxo_mode(&TcxoMode::new())?;
sg.calibrate_image(CalibrateImage::ISM_868)?; // or ISM_915

// 3. Regulator
sg.set_regulator_mode(RegMode::Smps)?; // or RegMode::Ldo

// 4. Buffer base addresses
sg.set_buffer_base_address(0, 128)?;

// 5. Packet type
sg.set_packet_type(PacketType::LoRa)?;

// 6. RF frequency
sg.set_rf_frequency(&RfFreq::F915)?; // Constants: F433, F868, F915
// Or custom: RfFreq::from_frequency(915_000_000)

// 7. PA config + TX params
sg.set_pa_config(&PaConfig::HP_22)?;
// HP_22, HP_20, HP_17, HP_14, LP_15, LP_14, LP_10
sg.set_tx_params(&TxParams::HP.set_ramp_time(RampTime::Micros200))?;

// 8. LoRa modulation params
let mod_params = LoRaModParams::new()
    .set_sf(SpreadingFactor::Sf7)    // Sf5..Sf12
    .set_bw(LoRaBandwidth::Bw125)   // Bw7..Bw500
    .set_cr(CodingRate::Cr45)        // Cr45, Cr46, Cr47, Cr48
    .set_ldro_en(false);
sg.set_lora_mod_params(&mod_params)?;

// 9. LoRa packet params
let pkt_params = LoRaPacketParams::new()
    .set_preamble_len(8)
    .set_header_type(HeaderType::Variable) // or Fixed
    .set_payload_len(255)
    .set_crc_en(true)
    .set_invert_iq(false);
sg.set_lora_packet_params(&pkt_params)?;

// 10. Sync word
sg.set_lora_sync_word(LoRaSyncWord::Public)?; // or Private

// 11. IRQ configuration
let irq_cfg = CfgIrq::new()
    .irq_enable_all(Irq::TxDone)
    .irq_enable_all(Irq::RxDone)
    .irq_enable_all(Irq::Timeout)
    .irq_enable_all(Irq::Err);
sg.set_irq_cfg(&irq_cfg)?;
```

### Transmitting

```rust
sg.write_buffer(0, &payload)?;
sg.set_lora_packet_params(
    &pkt_params.set_payload_len(payload.len() as u8)
)?;
sg.set_tx(Timeout::from_millis_sat(5000))?;
// Wait for TxDone IRQ (poll or hardware interrupt)...
let (status, irq_status) = sg.irq_status()?;
sg.clear_irq_status(irq_status)?;
```

### Receiving

```rust
sg.set_rx(Timeout::DISABLED)?; // continuous RX
// Or with timeout:
// sg.set_rx(Timeout::from_millis_sat(10_000))?;

// On RxDone IRQ:
let (status, irq_status) = sg.irq_status()?;
let (status, payload_len, rx_start) = sg.rx_buffer_status()?;
let mut buf = [0u8; 255];
sg.read_buffer(rx_start, &mut buf[..payload_len as usize])?;
sg.clear_irq_status(irq_status)?;

// RSSI / SNR from last packet:
let pkt_status = sg.lora_packet_status()?;
```

### IRQ API

**IRQ variants:** `TxDone`(1), `RxDone`(2), `PreambleDetected`(4),
`SyncDetected`(8), `HeaderValid`(16), `HeaderErr`(32), `Err`(64),
`CadDone`(128), `CadDetected`(256), `Timeout`(512).

**IrqLine variants:** `Global`, `Line1`, `Line2`, `Line3`.
All lines must be enabled for the internal NVIC interrupt to pend.

**CfgIrq builder:**
```rust
CfgIrq::new()
    .irq_enable(IrqLine::Global, Irq::TxDone)  // single line
    .irq_enable_all(Irq::RxDone)                // all lines
    .irq_disable_all(Irq::HeaderErr)            // disable on all
```

**NVIC helpers:**
- `subghz::unmask_irq()` — unmask SubGHz IRQ in NVIC (unsafe)
- `subghz::mask_irq()` — mask SubGHz IRQ in NVIC
- `subghz::rfbusys()` / `rfbusyms()` — check radio busy
- `subghz::wakeup()` — wake from sleep (unsafe)

### RTIC Integration Notes

The stm32wlxx-hal repo has **no RTIC examples**. The testsuite has
`subghz.rs` for on-target TX/RX tests (requires two nucleo boards).

For RTIC, bind the `SUBGHZ_RADIO` interrupt to a hardware task and use
`SubGhz::steal()` or pass via shared resources. The interrupt name in the
PAC is `SUBGHZ_RADIO`.

### Key Status Methods

- `sg.status()` — radio state (has documented HW bugs)
- `sg.irq_status()` -> `(Status, u16)` — IRQ flags
- `sg.rx_buffer_status()` -> `(Status, payload_len, buffer_ptr)`
- `sg.lora_packet_status()` -> `LoRaPacketStatus` (RSSI, SNR)
- `sg.rssi_inst()` -> instantaneous RSSI in dBm
- `sg.op_error()` -> operational error flags
- `sg.fsk_packet_status()`, `sg.fsk_stats()`, `sg.lora_stats()`
- `sg.reset_stats()` — clear cumulative stats

### Other Useful Methods

- `sg.set_sleep(SleepCfg)` — enter sleep (unsafe, 500us NSS hold-off)
- `sg.set_fs()` — frequency synthesis test mode
- `sg.set_rx_duty_cycle(rx_period, sleep_period)` — duty-cycled RX
- `sg.set_cad()` / `sg.set_cad_params(&CadParams)` — channel activity detection
- `sg.set_tx_rx_fallback_mode(FallbackMode)` — auto-mode after TX/RX
- `sg.set_pa_ocp(Ocp)` — over-current protection
- `sg.set_rx_gain(PMode)` — RX gain control
- `sg.free()` -> `(SPI3, MISO, MOSI)` — return peripherals



---

# OpenOCD
To unlock the `Seeed STM32WLE5 SX1262` using `openocd`
`openocd -f interface/cmsis-dap.cfg -f target/stm32wlx.cfg -c "init; reset halt; stm32l4x unlock 0; reset halt; exit"`

---

Using a RPI Pico ([DebugProbe](https://github.com/raspberrypi/debugprobe)) attached to the STM32WLE5 SWD  
| STM32 | Pico |
|---|---|
`PA13`  | `GP2`
`PA14`  | `GP3`
`NRST`  | `GND`

*NRST was held to GND, while the OpenOCD command was run.*
*As soon as the command was run, within a fraction of a second the GND was removed from NRST.*

Check probe-rs detects the chip:
`$ probe-rs info --chip STM32WLE5JCIx`

---

### Unlock
``` sh
$ openocd -f interface/cmsis-dap.cfg -f target/stm32wlx.cfg -c "init; reset halt; stm32l4x unlock 0; reset halt; exit"
Open On-Chip Debugger 0.12.0
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Info : auto-selecting first available session transport "swd". To override use 'transport select <transport>'.
none separate

Info : Using CMSIS-DAPv2 interface with VID:PID=0x2e8a:0x000c, serial=E6613852834C0C31
Info : CMSIS-DAP: SWD supported
Info : CMSIS-DAP: Atomic commands supported
Info : CMSIS-DAP: Test domain timer supported
Info : CMSIS-DAP: FW Version = 2.0.0
Info : CMSIS-DAP: Interface Initialised (SWD)
Info : SWCLK/TCK = 0 SWDIO/TMS = 0 TDI = 0 TDO = 0 nTRST = 0 nRESET = 1
Info : CMSIS-DAP: Interface ready
Info : clock speed 500 kHz
Info : SWD DPIDR 0x6ba02477
Info : [stm32wlx.cpu0] Cortex-M4 r0p1 processor detected
Info : [stm32wlx.cpu0] target has 6 breakpoints, 4 watchpoints
Info : starting gdb server for stm32wlx.cpu0 on 3333
Info : Listening on port 3333 for gdb connections
Info : [stm32wlx.cpu0] external reset detected
Error: [stm32wlx.cpu0] clearing lockup after double fault
Info : [stm32wlx.cpu0] external reset detected
[stm32wlx.cpu0] halted due to debug-request, current mode: Thread
xPSR: 0x01000000 pc: 0xfffffffe msp: 0xfffffffc
Info : device idcode = 0x10036497 (STM32WLE/WL5x - Rev 'unknown' : 0x1003)
Info : RDP level 1 (0x00)
Info : flash size = 256 KiB
Info : flash mode : single-bank
[stm32wlx.cpu0] halted due to debug-request, current mode: Thread
xPSR: 0x01000000 pc: 0xfffffffe msp: 0xfffffffc```

```

### Check probe-rs detects the chip  


``` sh
$ probe-rs info --chip STM32WLE5JCIx
Probing target via JTAG
-----------------------

Error while probing target: The protocol 'JTAG' could not be selected.

Caused by:
    The probe does not support the JTAG protocol.
Probing target via SWD
----------------------

ERROR probe_rs::architecture::arm::memory::romtable: 	Failed to read component information at 0xf0000000.
ARM Chip with debug port Default:

Debug Port: DPv2, Designer: STMicroelectronics, Part: 0x4970, Revision: 0x0, Instance: 0x00
├── V1(0) MemoryAP
│   └── 0 MemoryAP (AmbaAhb3)
│       ├── 0xe00ff000 ROM Table (Class 1), Designer: STMicroelectronics
│       ├── 0xe0001000 Generic
│       ├── 0xe0000000 Peripheral test block
│       ├── 0xe0040000 Generic
│       └── 0xe0043000 Coresight Component, Part: 0x0906, Devtype: 0x14, Archid: 0x0000, Designer: ARM Ltd
└── V1(1) MemoryAP
    └── 1 MemoryAP (AmbaAhb3)
```

---



