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

A build script must also rerun when the layout changes, or the old binary
is reused with the old flash origin. The build scripts generate `memory.x`
from the `flash-layout` build-dependency, so a change there rebuilds and
reruns them.

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
out of the workspace root. Each `build.rs` writes its crate's `memory.x`,
generated from the `flash-layout` crate, into `OUT_DIR` and emits
`cargo:rustc-link-search=<OUT_DIR>`. The node firmware, then the root
package, kept its layout as `memory-app.x` until it moved into `firmware/`;
the root now holds no crate at all. Confirmed:
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

The basestation uses a single UART for all host communication. OTA and data
relay commands are multiplexed on the same link - the frame protocol
(`basestation/src/uart_proto.rs`) has distinct command ranges (0x01-0x0F for
OTA, 0x10-0x1F for data relay) so no second UART is needed.

- **USART1** (PB6 TX, PB7 RX, pads 10/9) through an RS232 TTL 3.3V FTDI
  adapter, by default.
- **USART2** (PA2 TX, PA3 RX, pads 19/18) with the `board` feature: on the
  Forest Sensor Platform USART1 belongs to the RS-485 transceiver, and USART2
  runs to the CH572D, which bridges it to USB-C.

HSI16 (16 MHz) clock source at 115200 baud. The HAL enables HSI16 automatically
when `uart::Clk::Hsi16` is specified - it runs independently of the MSI system
clock.

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

## The boot-state log and the swap

The first bootloader kept one state word and a page counter on the state
page, erasing the page for every update. A power cut was fatal in three
ways. An erased page read as "nothing pending", so a cut between the erase
and the write booted a half-swapped slot on every boot. A resume restarted
a page at its first copy, overwriting the only saved copy of the old page.
And a write the cut left undefined (RM0461 3.3.6) could carry a two-bit
ECC error, whose NMI (RM0461 3.3.3) hung every boot in cortex-m-rt's
default handler. The replacement, `flash_layout::boot`, is built around
those facts:

- The state is an append-only log, one 8-byte record per double-word with
  a 16-bit check. Only the app's swap request erases the page, while
  nothing is pending, so a cut there just loses the request. Every other
  change appends, and a torn record fails its check and is skipped.
- A swap is 168 steps, three page copies per page, each followed by its
  record. A resume runs exactly the next step, whose source no step has
  touched yet, so it never reads a page that may be half-written. Every
  step can run twice without harm.
- The bootloader and the app both install an NMI handler that turns an
  uncorrectable ECC error into an unreadable double-word. The scan skips
  it; the bootloader's CRC check refuses an image containing one.
- Before the first copy the bootloader checks the staged image's size,
  vector table and CRC-32 against the request. A bad image gets a
  `Refused` record and the running one stays.

`flash-layout/src/boot/tests.rs` cuts the power at every erase and program
of a swap, a revert, a request and a confirm. It cuts once and then twice,
leaving the interrupted double-word or page untouched, finished, half-done
or unreadable each time, in a three-page geometry. It also cuts at a
hundred points across a full-size swap. Both of the first bootloader's
resume bugs, put back, fail those tests.

Two windows remain. The log page holds 256 records and a swap plus a
revert uses 229. Only more than two dozen failed writes in one update
fill it, and the fallback then erases the page and keeps only the current
record, which reopens the erase window once. A cut after the swap's last
record but before the jump reverts an update that never ran; send it
again.

The format is a contract between the bootloader and the app. Every record
carries a mark (`0xB7` in the top byte) that none of the first
bootloader's words had, so a mix of the two never swaps anything. Flash
the new bootloader by SWD (`mise run bootloader`) before sending new apps
over the air. With the first bootloader, a new app keeps running until its
first reset; that bootloader then sees its own unconfirmed state and
reverts it. And send only images built from this code or later: an older
app's `confirm_boot` rewrites the page in the old format, and OTA activation
stops on that node until it is reflashed.

## Boot confirm, OTA reboot and the watchdog

- The bootloader starts the IWDG (5 s) before any swap work and again
  before every jump, so an image is covered from its first instruction;
  the app's `watchdog::start` reprograms the same timeout. An active slot
  without a vector table is not jumped to: the bootloader waits for a
  probe with the watchdog off.
- An image the bootloader swaps in runs on trial. The main loop appends
  `Confirmed` once the image has run 30 minutes with a transmission
  completed in the last 5 and a packet heard since boot, or 6 hours
  without one: a node with nobody in range cannot show it still receives
  (`supervisor::confirm_ready`). Any reset before that reverts it, a
  brown-out or SW1 included. That is the safe direction: the update can
  be sent again.
- After `OTA_COMPLETE` the node keeps answering retried chunks with
  `OTA_COMPLETE` and reboots about 10 s later (plus two TX deadlines, for
  the slow presets). The bootloader then swaps the image in.
- Offers are refused with `BOOT_PENDING` (0x07) while a swap waits for that
  reboot, and while the running image is on trial: its revert target in
  the DFU slot is what a new transfer would overwrite.
- The handoff word in TAMP_BKP0R tells the app what the bootloader did;
  an app that finds none was started by probe-rs, and confirms a trial log
  at once, so a reset after a debug flash cannot revert over it.

## OTA timing

Estimates, not measurements; docs/v1-board-test.md step 10 records a real
transfer time.

- Mesh transfer: stop-and-wait, one 27-byte chunk per round trip. An earlier
  estimate put a round trip at about 0.75 s at the default `fast` preset
  (SF7/BW125), so a 48 KB image (1,788 chunks) takes about 22 minutes and a
  full 112 KB slot (4,248 chunks) about 53. Airtime grows roughly as 2^SF, so
  the slower presets take several times longer.
- Upload to the basestation: 128-byte `FW_DATA` frames at 115200 baud, each
  acknowledged, about 10 s for 48 KB.
- Bootloader swap: a CRC-32 over the staged image (well under a second at
  4 MHz), then 56 pages each copied three times (active to scratch, DFU to
  active, scratch to DFU), every copy followed by one appended record: 168
  page erases of roughly 20-25 ms, 168 pages programmed and 169 records.
  DS13105 Table 60 gives 81.7 us per double-word, about 21 ms a page, and
  41.8 ms for a "2-Kbyte page", the two disagreeing: about 8 s by the first,
  12 by the second. The node runs nothing meanwhile; the watchdog is fed
  before each page.
- Reboot after `OTA_COMPLETE`: 10 s plus two TX deadlines (10.3 s at
  `fast`, 25 s at `extreme`).
- Trial: at least 30 minutes before the new image is kept.
