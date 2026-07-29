# Development Notes

## App at 0x08004000 Boots Silent — Missing `set-vtor` Feature

After moving the app from `0x0800_0000` to `0x0800_4000` (to make room for the
bootloader), the firmware appeared dead — no RTT output, no radio, no display.
The bootloader at `0x0800_0000` was correctly flashed and its `jump_to_app()`
sets VTOR before branching.  The app worked fine when linked at `0x0800_0000`.

Root cause: `probe-rs run` resets the chip and then sets the PC directly from
the ELF entry point, **bypassing the bootloader**.  Because `cortex-m-rt` does
**not** set VTOR by default, the vector table offset register stayed at the
reset default `0x0800_0000`.  The first SysTick interrupt (from
`Mono::start()`) read the vector table from the bootloader's flash instead of
the app's, jumping to a wrong handler and crashing before any `rprintln!`.

At `0x0800_0000` this was invisible because the default VTOR already matched.

Fix: enable the `set-vtor` feature so `cortex-m-rt`'s Reset handler writes
VTOR before calling `main()`:
```toml
cortex-m-rt = { version = "0.7", features = ["set-vtor"] }
```

Also added `cargo:rerun-if-changed=memory.x` to `build.rs` — without it,
changing the flash origin in `memory.x` doesn't trigger a relink and the
old binary is reused.

## Bootloader Linked at Wrong Address — Two-Stage Boot Silent

Symptom: with the normal layout (bootloader at 0x08000000, app at 0x08004000)
nothing runs — no RTT, no fault. The app runs fine when linked directly at
0x08000000. Reading the RTT control block symbol in RAM (`nm ... _SEGGER_RTT`,
then `probe-rs read <addr>`) showed the "SEGGER RTT" magic only in the direct
build, proving the app's startup never executes via the bootloader path.

Root cause: `cortex-m-rt`'s `link.x` does `INCLUDE memory.x`, and the linker
resolves that from its working directory first. Cargo runs the linker from the
workspace root regardless of `cd`, so the bare root `memory.x` (ORIGIN
0x08004000) shadowed every member — the bootloader linked at 0x08004000, so on
reset the CPU booted an empty 0x08000000 and the bootloader never ran. Verified
with `readelf -S <elf> | grep vector_table` (bootloader showed 0x08004000).

Fix: give each crate its own layout via `OUT_DIR` and remove the shadowing
bare file from the workspace root. Renamed the root `memory.x` to
`memory-app.x`; each `build.rs` now copies its crate's file into `OUT_DIR` as
`memory.x` and emits `cargo:rustc-link-search=<OUT_DIR>`. Confirmed:
bootloader=0x08000000, app/basestation=0x08004000; flashing both then reset
boots the app (SEGGER RTT magic present, TX heartbeats stream).

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

## probe-rs Timeout / Reset Loop When I2C Display Disconnected

With no display connected, the I2C bus has no external pull-up resistors
(they live on the display module). Floating SDA/SCL lines cause the HAL's
`busy_wait!` macro to spin indefinitely — the peripheral never sees NACK
or ARLO because the bus state is indeterminate. The 5 s IWDG fires, MCU
resets, bootloader jumps to app, I2C hangs again → perpetual reset loop.
probe-rs can't establish a stable SWD connection during this loop, so
`cargo run -r` mostly times out (works occasionally if the timing aligns).

Fix (three layers):

1. **Enable internal pull-ups** on the I2C GPIO pins (`I2c2::new(..., true, cs)`).
   The HAL's `pullup: bool` parameter configures `Pull::Up` on both SCL/SDA.
   With pull-ups, a missing device produces a fast NACK instead of a hang.
   The weak internal pull-ups (~40 kΩ) don't affect boards with external
   pull-ups (~4.7 kΩ) — they simply parallel.

2. **Probe power-sequencing**: plug the Pico DebugProbe into USB **before**
   powering the target board. If the target is powered first, probe-rs
   may fail with a `SwjSequence` command ID mismatch or intermittent
   timeouts. Note: `--connect-under-reset` does NOT work with this setup
   because nRST is not wired to the probe (GP14).

3. **Feed watchdog before I2C operations** (init and retry) so the full
   5 s budget is available for I2C to fail, rather than arriving with a
   partially-elapsed timer.

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

## Basestation Node — UART Pin Selection

The basestation uses a single UART (USART1) for all host communication via an
RS232 TTL 3.3V FTDI adapter. OTA and data relay commands are multiplexed on the
same link — the frame protocol has distinct command ranges (0x01–0x0F for OTA,
0x10–0x1F for data relay) so no second UART is needed.

- **USART1** (PB6 TX, PB7 RX) — pins 10/9 on the Wio-E5 module

HSI16 (16 MHz) clock source at 115200 baud. The HAL enables HSI16 automatically
when `uart::Clk::Hsi16` is specified — it runs independently of the MSI system
clock.

UART frame protocol: `[0xAA sync] [len_lo] [len_hi] [cmd] [payload] [crc8]`
CRC-8/ITU (poly 0x07) over cmd + payload bytes.

## OTA Chunk Null-Byte Truncation (Protocol Change)

`embedded-nano-mesh` pads packet data to full capacity with `0x00` bytes and
does not expose the real `data_length` field (it is private).  The `receive()`
wrapper truncated at the first null byte to recover the actual payload length.

This corrupts OTA firmware chunks — binary data legitimately contains `0x00`
bytes, so truncation silently drops payload data.  The OTA receiver writes
incomplete chunks to flash, producing a bad firmware image.

Fix (two parts):

1. **Skip null-truncation for OTA messages** (`node.rs`): messages with a
   first byte in `0xF0..=0xF6` are binary OTA protocol messages and are
   passed through without truncation.

2. **Serialize `data_len` explicitly in OTA_CHUNK** (`ota_protocol.rs`):
   the old format inferred chunk length from the serialized message size,
   which breaks when the mesh layer pads the buffer.  New wire format:
   `[0xF3 type] [index_lo] [index_hi] [data_len] [data...]`
   This adds one byte of overhead, reducing `CHUNK_DATA_SIZE` from 28 → 27.
   The `total_chunks` calculation in `OtaSender::new()` uses `div_ceil` so
   it adjusts automatically.

**This is a protocol-breaking change** — nodes with old firmware cannot
exchange OTA chunks with nodes running the new format.

## Crate version bumps: embedded-io and ssd1306 must stay pinned

Attempted to bump `embedded-io` 0.6 -> 0.7.1 and `ssd1306` 0.8 -> 0.10.0 in the
root crate. Both had to be reverted:

- `embedded-io` must stay at 0.6. `embedded-nano-mesh` 2.1.x depends on
  `embedded-io` 0.6; `Node::update` takes an `I: embedded_io::Read + Write +
  ReadReady` bound from that 0.6 crate. Bumping our dep to 0.7 pulled a second
  `embedded-io` into the graph, so `LoraIo`'s 0.7 trait impls no longer
  satisfied nano-mesh's 0.6 bound. (0.7 also adds a `core::error::Error`
  supertrait on `embedded_io::Error`.)
- `ssd1306` must stay at 0.8. 0.9+ moved to `embedded-hal` 1.0, but
  `stm32wlxx-hal` 0.6 only implements `embedded-hal` 0.2 for its I2C
  peripherals, so the ssd1306 0.10 driver's trait bounds are unsatisfiable
  until the HAL gains eh-1.0 support.

`rtt-target` 0.5 -> 0.6.2 and `embedded-nano-mesh` 2.1.9 -> 2.1.11 were fine.

## Charger board support (`board` feature)

Feature-gated support for the MPPT buck NiMH charger board
(`buck-converter-mppt-nimh` KiCad project). Net-to-pin mapping was traced
from `buck-converter-real.kicad_sch`:

- PB14 = ADC_IN1, VSENSE_VIN, 10k:1.5k divider (Vin = Vpin * 23/3)
- PB13 = ADC_IN0, VSENSE_VOUT, 10k:10k divider (also the BOOT strap pin,
  shared with the J1 header - reading it as ADC is fine after boot)
- PA10 = ADC_IN6, BAT_ISENSE, 50 mOhm low-side shunt in the battery
  return (1 mV = 20 mA of charge current)
- PA9 = TIM1_CH2 (AF1), PWM_HI to the LM5109B gate driver. LI is tied to
  GND on the board: non-synchronous buck, SS56 freewheel diode, so only
  the high-side PWM is driven. Duty is clamped to 95% because the
  bootstrap cap only recharges while the switch node is low.
- SPI1 (PB3/PB4/PB5) + CS on PA0 + card detect on PB9 (pull-up, low =
  inserted) for the DM3CS-SF microSD socket.

Design choices:

- The `board` build raises MSI from 4 to 16 MHz (`board::raise_sysclk`,
  called before the SysTick monotonic starts). Reason: TIM1 is clocked
  from sysclk and the SPICE-validated switching frequency is 100 kHz;
  4 MHz gives only 40 duty steps, 16 MHz gives 160. 16 MHz still needs
  zero flash wait states. `SYSCLK_HZ` is cfg-gated in `platform.rs`; the
  I2C/SubGHz drivers derive their timing from RCC at init so they follow
  automatically.
- stm32wlxx-hal 0.6 has no TIM1 support, so the PWM is set up through
  the PAC directly (PSC/ARR/CCMR1/CCER/BDTR.MOE). ADC uses the HAL
  (`Adc::pin`), with VREFINT factory calibration to correct for the
  actual 3V3 rail.
- SD card: minimal in-repo SPI-mode driver (CMD0/8/ACMD41/58, single
  block read/write) instead of embedded-sdmmc, because embedded-sdmmc
  0.7+ needs embedded-hal 1.0 (HAL only implements 0.2) and older
  versions want a `FullDuplex` impl the HAL also lacks. Init at 250 kHz
  (Div64), then 8 MHz (Div2) via a direct CR1.BR write (HAL has no baud
  setter).
- The RTIC `Local` struct uses `type BoardRes = Board` / `()` cfg alias
  so resource plumbing is identical with the feature off.
- With `board`, the 10 s heartbeat broadcasts `V=<mV> B=<mV> I=<mA>`
  instead of "hello" and logs the same over RTT.

Build with: `ADDRESS=n cargo run --release --features board`

## MPPT charge controller (board feature)

Perturb & observe MPPT in `board::Mppt`, stepped every 200 ms from the
main RTIC loop (`Board::mppt_step`). Design points:

- The shunt is on the battery side, so the controller hill-climbs on
  output power (vbat * ibat). Buck efficiency is flat enough across the
  range that the output maximum coincides with the panel MPP.
- States: Idle (vin < vbat + 1 V, PWM parked, 500 mV re-entry
  hysteresis so a marginal panel does not flap), Tracking (P&O, 13
  permille ~= 2 TIM1 counts per step, direction reverses only when
  power drops by more than 40 mW), Limiting (vbat > 4.2 V or ibat >
  1 A: back off 26 permille per step, resume tracking downhill).
- Noise: 1 ADC LSB across the 50 mOhm shunt is ~16 mA, so power noise
  would dwarf the P&O signal. Mitigated with 16x hardware oversampling
  (enabled before `Adc::enable`, result stays 12-bit) plus a 4-sweep
  software average per step; the 40 mW hysteresis absorbs the rest.
- Duty is tracked in the controller, not read back from TIM1: CCR
  quantization (~6 permille) makes register round-trips lossy.
- NiMH termination is CV float at 1.4 V/cell, not -dV detection: with
  solar input the current varies with irradiance anyway, so a -dV
  signature is unreliable. The 4.2 V ceiling matches the board's
  "3V6-4V2" battery rating.
- Heartbeat broadcast format is now `V=<mV> B=<mV> I=<mA> D=<permille>
  <i|t|l>` (state letter). Worst case 31 bytes, fits the 32-byte mesh
  payload.

## RS-422 link + I2C sensor suite (rs422 / sensor features)

Branch `peripherals`. Two new independent cargo features.

`rs422`: MAX3430-class transceiver on USART2, PA2 TX / PA3 RX (the
"UART 2" header nets on the charger board). Full-duplex RS-422
point-to-point assumed, driver enable strapped on - half-duplex RS-485
multi-drop would need a DE/RE GPIO around write_all. Includes a
minimal Modbus RTU master (function 0x03 + CRC16). Key discovery: the
DFRobot soil probe (product 2816 = SEN0600) is NOT I2C - it is RS-485
Modbus RTU, 9600 8N1, slave 0x01, moisture at holding reg 0x0000 and
temperature at 0x0001, both x10 scaled (temp two's complement). So
soil moisture rides the rs422 link, not the sensor bus. The `nb` crate
was added (optional dep) because the HAL UART only exposes nb-style
embedded-hal 0.2 serial traits; nb 1.x is compatible since eh 0.2's
nb 0.1.3 is a facade over nb 1.

`sensor`: probes one shared I2C bus (I2C2, PB15/PA15 - the module's
only exposed I2C) at startup and reads whatever answered every 30 s:
- BME680 (0x76/0x77): full in-repo driver - forced mode, datasheet
  float compensation for T/P/H, gas resistance with 320 C / 150 ms
  heater profile. IAQ index would need Bosch's BSEC blob; raw gas
  resistance is the honest proxy.
- SCD41 (0x62): periodic mode, CRC-8 checked reads, presence detected
  via get_serial_number (only ACKed while idle).
- ADXL345/343 (0x53/0x1D): same DEVID/driver for both, full-res +-2g.
- AS3935 (0x03..0x01): presence + INT_SRC/distance polling only; the
  IRQ pin is not wired (suggest GPIO-10/PB10 later).
- BMV080 (0x57/0x56): presence only - register protocol is only
  documented inside Bosch's closed vendor library.
Bus sharing with the SSD1306: no shared-bus crate; a 30-line
SharedI2c(&'static RefCell<I2c2>) wrapper (via cortex_m::singleton!)
is safe here because display + sensors live in the same priority-1
RTIC task. Display type is cfg-aliased accordingly.
No driver crates were pulled: bme680/scd4x/adxl343 crates have mixed
embedded-hal version support against this HAL (eh 0.2 only), and the
total in-repo code is ~450 lines.
Blocking sweep worst case ~0.5 s (BME heater + soil timeout), so the
sweep is skipped during OTA transfers.

## Radio activity LEDs (board feature)

LED1 (PC0) pulses on radio TX, LED2 (PC1) on RX; both active high
(pin -> LED -> 10k -> GND per schematic). Hooked at the Sx1262Driver
level (note_tx after set_tx, note_rx on packet read) rather than in
the app loop, so mesh-internal traffic and future packet forwarding
blink too. The driver sets lock-free atomic flags in board::activity;
Leds::update in the main loop starts/retires 30 ms pulses, avoiding
any GPIO coupling inside the radio driver.

## GPS + radio link logger (gps-radio-log feature)

GPS goes on USART1 (PB7 RX at 9600 8N1, AF7) - the only free UART:
USART2 is the RS-422 bus and LPUART1's PC0/PC1 pins are the activity
LEDs. RX-only; PB6 (USART1 TX) is left free until a module needs
configuring. The GPS is kept powered and active permanently (power
budget accepted for this feature), so no PMTK/UBX standby commands.
The feature implies `board` (SD card) and `nb` (UART reads).

NMEA handling: assemble lines, verify the XOR checksum, keep only
RMC/GGA (any talker prefix) and log the raw sentence - raw NMEA
already carries every metric (UTC, lat/lon, fix, sats, HDOP, alt,
speed, course), so the SD log stores sentences verbatim. GGA yields
fix-quality/sats for an RTT status line; GGA and RMC are also parsed
for position (see below). At 9600 baud (~1 char/ms) the 1 ms main
loop keeps up; the blocking sensor sweep (~0.5 s) causes UART
overruns, handled by clearing ICR flags and resyncing on the next '$'.

Display readout: in this build the SSD1306 is dedicated to a lat/long
readout (the TX/RX message banners are cfg'd out; OTA progress still
shows). GGA (quality>0) and RMC (status 'A') coordinates are parsed
to signed decimal degrees - NMEA ddmm.mmmm / dddmm.mmmm split as
deg = all-but-last-two integer digits, min = the rest, deg + min/60,
negated for S/W. f32 throughout (matches the sensor suite; core's
float Display handles the "{:.5}" format, no libm). A quality-0 GGA
or status-'V' RMC clears the fix so the display falls back to
"Acquiring NN". Refreshed at 1 Hz (the fix rate) to avoid flicker;
`Gps::has_pos`/`lat_deg`/`lon_deg` plus `fmt_status`/`fmt_lat`/
`fmt_lon` are the interface. Lines fit the 12-char FONT_10X20 width
(worst case "145.12267 W" = 11 chars).

Radio metrics: poll_recv already read LoRaPacketStatus for RSSI; the
driver now also queues SNR (snr_pkt() numer() = raw quarter-dB) and
signal_rssi_pkt (post-despread RSSI) per RX, TX length/result per TX,
into a small critical-section ring (gpslog::events) drained by the
main loop - same pattern as board::activity, but with payloads. A
STATS line (pkt_rx/pkt_crc/pkt_hdr_err from Get_Stats) logs every
60 s; that is everything the SX126x exposes.

Each drained event is also echoed to RTT (the SD-log line, minus the
trailing newline) so RSSI/SNR are visible live without pulling the
card. Independently, the main-loop receive path prints RSSI on its
always-on "RX #n ... rssi=<dBm>" line (io.last_rssi()) rather than
only under the `debug` feature, so RSSI shows in RTT in every build.

SD format: no filesystem. Header block at LBA 2048 (1 MiB in, clear
of any partition metadata) holds magic "GRL1" + next-LBA; data blocks
follow (1 GiB region, wraps). Blocks are zero-padded ASCII lines
("t=<ms> GPS/RX/TX/STATS ...") so recovery is dd+strings. Header is
checkpointed every 16 blocks (not every block) to cut wear; resume
skips 16+1 blocks past the stored pointer so unrecorded blocks and a
partial tail survive reboots. Partial blocks are synced in place
every 10 s, bounding loss on power failure. SD errors drop the logger
to not-ready and a 10 s retry path re-inits the card, so a card
inserted after boot (or a transient SPI error) recovers.

## SHT45 / MCP9808 / BMP280 added to the sensor suite

Same in-repo driver approach (eh 0.2 traits only, no crates):
- SHT45 (0x44): single-byte commands (0xFD measure hi-precision,
  0x89 serial for presence), 10 ms wait, 2x 16-bit words with the
  same Sensirion CRC-8 as the SCD41 (scd41_crc renamed
  sensirion_crc). RH clamped to 0..100 (raw formula goes to -6/+119).
- MCP9808 (0x18..0x1F scanned): identified via manufacturer id
  0x0054 (reg 0x06) + device id 0x04 (reg 0x07 high byte). No init -
  powers up converting. Temp reg 0x05 is 13-bit two's complement at
  0.0625 C/LSB (x100 = t * 25 / 4).
- BMP280 (0x76/0x77): shares the address pair with the BME680, so
  the probe dispatches on chip-id reg 0xD0 (0x61 = BME680, 0x58 =
  BMP280); one of each can coexist on opposite addresses. Forced
  mode osrs_t x2 / osrs_p x4, poll status bit 3, datasheet float
  compensation (same t_fine scheme as the BME680 but i16 T3/P3 and
  no p10 term).

## Sensor sweep broadcast + dashboard capture path

The 30 s sweep now broadcasts a compact ASCII line over the mesh
(alongside the existing rprintln output): `T=<cC> H=<c%RH> C=<ppm>
M=<permille moisture>`. Keys are disjoint from the heartbeat's
V/B/I/D so receivers parse both with one key table. T/H pick the
best available source (BME680 > SHT45 > SCD41 > MCP9808 > BMP280).
Pressure is not broadcast - it does not fit the 32-byte payload
(worst case with P would be 38 bytes). TelemetryLine in main.rs
drops any field that does not fit whole rather than truncating
mid-number.

Host path to the dashboard: basestation data UART -> data_relay.py
(JSON lines) -> forest-datad (../forest-data/daemon, captures to
CSV) -> dashboard LiveSource tails the CSV (FOREST_LIVE_LOG env
var). Details in ../forest-data/NOTES.md.

## LoRa range presets via LORA_PRESET build flag

SF/BW/CR are now build-time selectable via the LORA_PRESET env var
(fast/long/max/extreme), same option_env! pattern as ADDRESS. Selection
lives in config.rs (RadioPreset enum + RADIO_PRESET const); radio.rs
init() matches on it for set_lora_mod_params. TX power stays +22 dBm on
all presets - only modulation changes.

Key gotchas baked in:
- Airtime grows ~2^SF (SF7 ~0.1 s -> SF12 ~3.3 s for a 40 B packet), so
  TX_POLL/CHIP_TIMEOUT and MESH_LISTEN_PERIOD are derived per-preset in
  config.rs. Leaving them at the SF7 values would make every TX time out
  before TxDone fires.
- LDRO must be enabled for symbol time > 16.38 ms (SF11/SF12 @ BW125,
  SF12 @ BW62.5); the preset match sets ldro accordingly.
- Invalid LORA_PRESET values panic at compile time (const match).
- max/extreme exceed the FCC 400 ms dwell limit for 902-928; extreme
  relies on the Wio-E5 TCXO for the narrow 62.5 kHz bandwidth.

Confirmed +22 dBm is genuinely max: PaConfig(duty=0x04,hp_max=0x07,Hp) +
set_power(0x16) exactly match the HAL's PaConfig::HP_22 / TxParams::HP
presets; OCP at Max140m (140 mA) is required for the HP PA or output
would be clamped below +22.

## GPS-log status line: sats + last RX RSSI + time-since-RX

The OLED top line in gps-radio-log now reads e.g. "08 -95dBm 1m23s"
(sat count, last packet RSSI, elapsed since last RX), replacing the old
"Fix 08 sat" text. Choices:
- The RX timestamp is stamped at the physical layer in io.rs (note_rx,
  alongside last_rssi via platform::millis()), not at mesh.receive().
  This keeps "time since last RX" consistent with last_rssi - both refer
  to the same heard packet, including ones the mesh doesn't surface as
  app messages. Exposed as LoraIo::last_rx_ms() -> Option<u32> (None
  before any RX). Elapsed uses wrapping_sub to ride the DWT millis wrap.
- Three fields don't fit 12 chars of FONT_10X20, so the status line uses
  FONT_7X13 (~18 chars); lat/long below stay at FONT_10X20. Fix state is
  now implied by lat/long appearing rather than the "Fix"/"Acquiring"
  word.

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

## Build break: `trailing semicolon in macro used in expression position`

Current nightly rustc turned `semicolon_in_expressions_from_macros` into a
deny-by-default future-incompatibility error, and `rprintln!` expands to a
block ending in a semicolon. Every `rprintln!`/`debug_println!` used as the
value of a match arm or as the tail expression of a block became a hard
error, so the tree would not build at all. Fixed by adding the semicolon
inside the `debug_println!` macro and braces around the affected match arms.

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

## RxBoost via the RX_BOOST build flag

The SX126x RxGain register was never written, so the radio ran at its
power-up default of power-saving gain. Boosting it is worth ~2 dB of
sensitivity - about one step of `LORA_PRESET` - for no airtime at all,
which is the cheapest link budget available here.

Exposed as `RX_BOOST=0|1` alongside `LORA_PRESET` and `ADDRESS` rather than
as a preset field: it is orthogonal to the modulation, and unlike the
presets it does not have to match at both ends. Defaults to off to match the
chip, because the cost is receive current drawn continuously on a node that
listens most of the time - a real trade on solar, so it is opted into rather
than inherited. A bool rather than the HAL's four-level `PMode`: only power
saving and boosted have specified behavior.

Applied inside `init` on every call rather than once, because RxGain is not
covered by warm-start sleep retention. Moot today - the only path back is a
full re-init - but it keeps the setting correct if the radio is ever put
into a retaining sleep.
