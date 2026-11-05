# Boot and OTA

## App at 0x08004000 Boots Silent - Missing `set-vtor` Feature

After moving the app from `0x0800_0000` to `0x0800_4000` (to make room for the
bootloader), the firmware appeared dead - no RTT output, no radio, no display.
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

Also added `cargo:rerun-if-changed=memory.x` to `build.rs` - without it,
changing the flash origin in `memory.x` doesn't trigger a relink and the
old binary is reused.

## Bootloader Linked at Wrong Address - Two-Stage Boot Silent

Symptom: with the normal layout (bootloader at 0x08000000, app at 0x08004000)
nothing runs - no RTT, no fault. The app runs fine when linked directly at
0x08000000. Reading the RTT control block symbol in RAM (`nm ... _SEGGER_RTT`,
then `probe-rs read <addr>`) showed the "SEGGER RTT" magic only in the direct
build, proving the app's startup never executes via the bootloader path.

Root cause: `cortex-m-rt`'s `link.x` does `INCLUDE memory.x`, and the linker
resolves that from its working directory first. Cargo runs the linker from the
workspace root regardless of `cd`, so the bare root `memory.x` (ORIGIN
0x08004000) shadowed every member - the bootloader linked at 0x08004000, so on
reset the CPU booted an empty 0x08000000 and the bootloader never ran. Verified
with `readelf -S <elf> | grep vector_table` (bootloader showed 0x08004000).

Fix: give each crate its own layout via `OUT_DIR`, and keep every `memory.x`
out of the workspace root. Each `build.rs` copies its crate's `memory.x` into
`OUT_DIR` and emits `cargo:rustc-link-search=<OUT_DIR>`. The node firmware,
then the root package, kept its layout as `memory-app.x` until it moved into
`firmware/`; the root now holds no crate at all. Confirmed:
bootloader=0x08000000, app/basestation=0x08004000; flashing both then reset
boots the app (SEGGER RTT magic present, TX heartbeats stream).

## IWDG SR Wait Hangs - Silent Reset Loop (No RTT Output)

After adding watchdog support, the firmware appeared to "do nothing" after
flashing - no RTT output, no display, no radio activity.

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
to ~8,759 bytes and `Local` from 4,128 -> 2,080 bytes.

Lesson: on embedded targets, always check `clippy::large_stack_frames` when
adding structs with large inline buffers to RTIC `Local` - the `init()` return
path constructs them on the stack before moving to statics.

## Basestation Node - UART Pin Selection

The basestation uses a single UART (USART1) for all host communication via an
RS232 TTL 3.3V FTDI adapter. OTA and data relay commands are multiplexed on the
same link - the frame protocol has distinct command ranges (0x01-0x0F for OTA,
0x10-0x1F for data relay) so no second UART is needed.

- **USART1** (PB6 TX, PB7 RX) - pins 10/9 on the Wio-E5 module

HSI16 (16 MHz) clock source at 115200 baud. The HAL enables HSI16 automatically
when `uart::Clk::Hsi16` is specified - it runs independently of the MSI system
clock.

UART frame protocol: `[0xAA sync] [len_lo] [len_hi] [cmd] [payload] [crc8]`
CRC-8/ITU (poly 0x07) over cmd + payload bytes.

## OTA Chunk Null-Byte Truncation (Protocol Change)

`embedded-nano-mesh` pads packet data to full capacity with `0x00` bytes and
does not expose the real `data_length` field (it is private).  The `receive()`
wrapper truncated at the first null byte to recover the actual payload length.

This corrupts OTA firmware chunks - binary data legitimately contains `0x00`
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
   This adds one byte of overhead, reducing `CHUNK_DATA_SIZE` from 28 -> 27.
   The `total_chunks` calculation in `OtaSender::new()` uses `div_ceil` so
   it adjusts automatically.

**This is a protocol-breaking change** - nodes with old firmware cannot
exchange OTA chunks with nodes running the new format.
