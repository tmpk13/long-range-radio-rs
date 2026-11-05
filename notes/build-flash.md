# Build, debug and flashing

## Workspace layout: three firmware crates side by side

The node firmware (`firmware/`), the bootloader and the basestation are
sibling crates, and `flash-layout/` holds the partition map all three link
against; the root holds only the workspace manifest and shared config.
Each part of that layout answers a failure:

- No `memory.x` may sit in the workspace root. The linker runs there, so
  cortex-m-rt's `INCLUDE memory.x` would find that file first for every
  member (boot-ota.md, "Bootloader Linked at Wrong Address"). For that
  reason the node firmware, while it was the root package, kept its layout
  as `memory-app.x`.
- The partition map lives only in `flash-layout`, and each build script
  generates its crate's `memory.x` from it. It used to be restated in three
  `memory.x` files and five source files, and a bootloader and an app that
  disagree about it break over-the-air updates without any build error.
- One package per cargo call. Building several members at once unifies
  cortex-m-rt's features, so the HAL's `device` feature reaches the
  bootloader, which has no `device.x`, and its link fails.
  `default-members = ["firmware"]` keeps a plain `cargo build` or
  `cargo run` at the root on the node firmware; the mise tasks pass `-p`.
- The root `.cargo/config.toml` sets `build.target = thumbv7em-none-eabi`
  for every crate below the root, and a nearer config cannot unset it
  (cargo appends arrays and has no unset for a scalar). Host-side Rust,
  such as `flash-layout`'s tests or a Rust port of `basestation/host/`,
  needs an explicit `--target`, as `mise run test` passes, or a home
  outside this repo's config. The crates used to carry identical copies of
  that config; only the root one is left.

Moving the crate into `firmware/` left eight of ten images (every feature
set, bootloader, basestation) byte-identical. The two `sensor` images
differed by 65 bytes: the package path feeds the symbol hashes, which
reordered two rodata constants, so their addresses changed in a few
instructions. Expect the same from any future move, and compare images
with `RUSTFLAGS=--remap-path-prefix=firmware/=` so panic-location paths
match.

## Debug Mode

A `debug` cargo feature gates verbose output via a `debug_println!` macro.
`#[cfg]` on expressions is not stable Rust, so the macro uses `cfg!()` inside
a block (evaluates at compile time but keeps the expression valid in all
positions including match arms).

Enable with:
```
cargo run --release --features debug
```

## probe-rs Timeout / Reset Loop When I2C Display Disconnected

With no display connected, the I2C bus has no external pull-up resistors
(they live on the display module). Floating SDA/SCL lines cause the HAL's
`busy_wait!` macro to spin indefinitely - the peripheral never sees NACK
or ARLO because the bus state is indeterminate. The 5 s IWDG fires, MCU
resets, bootloader jumps to app, I2C hangs again -> perpetual reset loop.
probe-rs can't establish a stable SWD connection during this loop, so
`cargo run -r` mostly times out (works occasionally if the timing aligns).

Fix (three layers):

1. **Enable internal pull-ups** on the I2C GPIO pins (`I2c2::new(..., true, cs)`).
   The HAL's `pullup: bool` parameter configures `Pull::Up` on both SCL/SDA.
   With pull-ups, a missing device produces a fast NACK instead of a hang.
   The weak internal pull-ups (~40 kOhm) don't affect boards with external
   pull-ups (~4.7 kOhm) - they simply parallel.

2. **Probe power-sequencing**: plug the Pico DebugProbe into USB **before**
   powering the target board. If the target is powered first, probe-rs
   may fail with a `SwjSequence` command ID mismatch or intermittent
   timeouts. Note: `--connect-under-reset` does NOT work with this setup
   because nRST is not wired to the probe (GP14).

3. **Feed watchdog before I2C operations** (init and retry) so the full
   5 s budget is available for I2C to fail, rather than arriving with a
   partially-elapsed timer.

## `mise run attach`: "Exception @ 0x08000400" is a reset, and a stale ELF is silent

`Firmware exited unexpectedly: Exception` with Frame 0 at
`<unknown function @ 0x08000400>` is not a crash in the app.
0x08000400 is the bootloader's `Reset` handler. probe-rs arms reset
vector catch, so a chip reset (watchdog, reset button, software reset)
halts the core on the bootloader's first instruction. Seen on the first
FSP board on 2026-11-05, after a reset-button press following
`ADDRESS=2 mise run flash --features sensor`. The same image later booted
cleanly after a power cycle and three SWD resets, so the build was not the
cause. Whether the button reset met a catch left armed by the flash
session's `probe-rs run`, or a real reset happened, was not determined.

probe-rs leaves the core halted with the catch still armed (DEMCR read
0x01000401). Only a power cycle clears DEMCR, so until then the reset
button halts the core again. Each later attach also prints the previous
boot's leftover RTT lines from RAM, so it looks like the same crash every
time. Power-cycle the board, or attach with `--no-catch-reset
--no-catch-hardfault` to watch without arming the catch.

`mise run attach` finds the RTT block at the `_SEGGER_RTT` address of the
ELF on disk, despite `--rtt-scan-memory`. After a build that never reached
the chip, attach prints nothing: a `board` rebuild had moved the block from
0x20000c44 to 0x20000c3c. To check that the chip runs the ELF on disk,
compare `probe-rs read --chip STM32WLE5JCIx b32 0x08004000 4` with the
first words of the ELF's binary image.

Read-only checks that work on a running node without halting it: DWT_PCSR
(0xE000101C) samples the PC. The app has no `#[idle]`, so a healthy node
samples at the `b .` that ends RTIC's `main`. RCC_CSR (0x58000094) bit 29
is the IWDG reset flag.

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
|-- V1(0) MemoryAP
|   `-- 0 MemoryAP (AmbaAhb3)
|       |-- 0xe00ff000 ROM Table (Class 1), Designer: STMicroelectronics
|       |-- 0xe0001000 Generic
|       |-- 0xe0000000 Peripheral test block
|       |-- 0xe0040000 Generic
|       `-- 0xe0043000 Coresight Component, Part: 0x0906, Devtype: 0x14, Archid: 0x0000, Designer: ARM Ltd
`-- V1(1) MemoryAP
    `-- 1 MemoryAP (AmbaAhb3)
```

The `0xf0000000` error is AP1, the second core's port on the dual-core
STM32WL55: its BASE reads 0xF0000003 but every access behind it FAULTs
on the WLE5. Harmless.

### Read protection

`probe-rs info` succeeds at RDP level 1 too (the DP, APs and ROM table
stay readable), so it does not show that a chip is unlocked. The RDP
byte is the low byte of FLASH_OPTR (RM0461):

``` sh
$ probe-rs read --chip STM32WLE5JCIx b32 0x58004020 1
3ffff0bb
```

0xAA is level 0, 0xCC level 2 (permanent), anything else level 1; a
factory Wio-E5 reads 0xBB. At level 1 with a debugger attached, flash
reads through the AP FAULT and the core cannot fetch its vectors, so it
sits in lockup (DHCSR 0x03090000, S_LOCKUP). The OpenOCD unlock above
(`mise run unlock`) sets level 0, which erases the flash.

The unlock only programs the option bytes: the stored copy at 0x1FFF7800
then reads 0x3FFFF0AA (complement 0xC0000F55 at 0x1FFF7804), while
FLASH_OPTR keeps 0xBB until the option bytes are loaded, at power-on or
OBL_LAUNCH. OpenOCD's closing `reset halt` is a system reset, which does
not load them, and `stm32l4x unlock` reports nothing either way. Power
cycle the board after the unlock; the regression erase happens at that
load. `mise run unlock-check` reads both words and names the state,
including an unlock still waiting for the power cycle. Through the CH572D bridge the unlock connected without holding
NRST: OpenOCD cleared the lockup itself.

---

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

## Build break: `trailing semicolon in macro used in expression position`

Current nightly rustc turned `semicolon_in_expressions_from_macros` into a
deny-by-default future-incompatibility error, and `rprintln!` expands to a
block ending in a semicolon. Every `rprintln!`/`debug_println!` used as the
value of a match arm or as the tail expression of a block became a hard
error, so the tree would not build at all. Fixed by adding the semicolon
inside the `debug_println!` macro and braces around the affected match arms.
