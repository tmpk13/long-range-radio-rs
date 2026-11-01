# Build, debug and flashing

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
