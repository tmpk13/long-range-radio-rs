# CH572D USB bridge firmware

The Forest Sensor Platform's CH572D (U9) sits between the USB-C port and
the Wio-E5: USB D-/D+ on PA0/PA1, UART TX/RX on PA8/PA9 to the Wio-E5's
USART2 (PA3/PA2), SWD on PA5 (SWCLK) and PA7 (SWDIO), WIO-RST on PA6.
`ch572/` makes it a composite USB device: a CDC-ACM serial port bridged to
that UART and a CMSIS-DAP v2 probe for the Wio-E5, so one cable gives
probe-rs flashing/RTT and the serial link. Pin table: `ch572/src/hw.rs`.
No BLE: that needs WCH's closed-source stack.

Sources: the datasheet translation in `~/ch572d/ch572d_en.md`, and WCH's
EVT (github.com/openwch/ch570: `CH572SFR.h`, `CH57x_sys.c` SetSysClock,
`USB/Device/COM`). `ch572/src/regs.rs` uses the SFR header's names; all
101 constants that have one were checked against it by script (0 mismatches);
SysTick comes from `core_riscv.h` and datasheet 3.4.4.

## Build layout

- Its own workspace and its own `.cargo/config.toml` (target
  riscv32imc: the core is a QingKe V3C, RV32IMBC, no atomics). A nearer
  config's `build.target` string overrides the parent's thumbv7em one, but
  nothing can unset it, so host tests need `--target`: `mise run
  ch572-test` derives the host triple. Its own `rust-toolchain.toml` too,
  because the parent's lists only the thumbv7em target.
- No qingke-rt or PAC: `ch572_pac` 0.2 pins qingke-rt 0.5 while current is
  0.8, and a polled firmware needs no interrupt machinery. `main.rs` has a
  40-line startup modelled on WCH's `startup_CH572.S` (gp, sp, copy
  `.highcode` and `.data`, zero `.bss`, corecfgr 0xBC0 = 0x25 for prefetch)
  but stays in machine mode with interrupts off. Every trap and panic does
  a software reset.
- Everything but `hw.rs` is hardware-independent and runs in host tests
  (`src/sim.rs`): a model of the USB controller as the datasheet and
  WCH's handler describe it, a host driving real enumeration through
  usb-device, an SWD target that enforces the protocol (line reset first,
  parity, turnaround contention), and a fake UART. Four deliberate driver
  bugs were each caught by a test. The model encodes a reading of the
  documentation, so silicon that differs from it would pass the tests.

## Clock

Fsys = PLL 600 MHz / 6 = 100 MHz, WCH's CLK_SOURCE_HSE_PLL_100MHz, also
what their USB COM example runs. The UART divides Fsys / 8 / baud, so the
clock decides baud accuracy: at 100 MHz every rate up to 460800 is within
0.5%. At 60 MHz, 230400 is 1.4% off; at 16 MHz (crystal / 2, no flash
retiming needed) 115200 is 2.1% off, beyond the CH572's 2% receive
tolerance. The PLL also gives USB its 48 MHz (600 / 25 x 2).

A PLL clock needs R8_FLASH_CFG = 0x01 and R8_FLASH_SCK bit 4 (WCH's
SetSysClock; neither register is described in the datasheet or SVD). WCH
runs SetSysClock from RAM (`__HIGH_CODE`) because it retimes the flash;
`hw::init_clocks` is linked into `.highcode` and was checked in the
disassembly to make no calls back into flash. Safe-access writes follow
WCH's SAFEOPERATE (`fence.i`, emitted as `.word 0x0000100f` because the
imc target has no Zifencei).

The crystal's load caps C26/C27 (18 pF) are fitted on V1 although the
CH572 has internal ones (board review H2): the clock runs some tens of ppm
low. USB does not care, so R8_XT32M_TUNE is left at reset. They also add to
the internal load (16 pF at reset) against Y2's 12 pF rating (Murata
XRCGB32M000F1S1DR0), which cuts the oscillator's start-up margin. That
matters more than usual: the chip has no fast internal oscillator and runs
from the crystal / 5 from power-on, so a crystal that does not start leaves
it silent, bootloader included.

## USB driver (`src/usb.rs`)

Polled, with RB_UC_INT_BUSY so the controller NAKs everything until
RB_UIF_TRANSFER is cleared. Handling follows WCH's CH572 COM example:

- Toggles are flipped by firmware per transaction, not RB_UEP_AUTO_TOG;
  OUT data with RB_U_TOG_OK clear is a retransmission and is dropped.
- An endpoint is set to NAK as its transaction completes. Otherwise the
  response bits still say ACK when the flag is cleared and the host gets
  the same IN buffer a second time.
- A SETUP behind a token transaction (RB_UIS_SETUP_ACT with a token
  pending) is handled after it, each with its own clear of
  RB_UIF_TRANSFER. The SETUP aborts any EP0 transfer, so stale EP0 events
  from the same poll are dropped.
- usb-device ignores an EP0 IN completion reported in the same poll as a
  SETUP. If that completion is SET_ADDRESS's status stage, usb-device never
  sets the address, so the driver snoops SET_ADDRESS and applies it itself
  when the status stage completes.
- SET_CONFIGURATION is snooped to reset data toggles; CLEAR_FEATURE(halt)
  resets them through `set_stalled(false)`, except on EP0: usb-device
  unstalls EP0 after every SETUP, which must not clear its DATA1 toggles.
  The DAP class resets its own on SET_INTERFACE by unstalling.
- `ep_out` and `ep_setup` are reported until read, `ep_in_complete` once
  (usb-device's contract).
- Endpoints 1-3 all have both directions and single buffers, which fixes
  each buffer as OUT data then IN data. EP4 is unused: it shares EP0's DMA
  area and has no auto-toggle. Allocation gives the serial port EP1 IN
  (notify), EP1 OUT and EP2 IN, and the DAP interface EP2 OUT and EP3 IN.

## Bridge and CMSIS-DAP

- The UART's receive FIFO is 8 bytes, full in 0.7 ms at 115200, and a DAP
  command can run longer. `SwdIo::service` runs before every SWD transfer
  and inside delays, moving the FIFOs into rings and feeding the watchdog
  (timeout ~335 ms at 100 MHz). There is no flow control towards the
  Wio-E5: a full ring drops bytes. Host to Wio-E5 is flow-controlled by
  NAK.
- The UART follows the host's line coding; queued bytes are dropped on a
  change since they were framed for the old one.
- dap-rs does the command handling. Its own v2 class adds an SWO endpoint
  the board cannot use (the Wio-E5's SWO pin is the SD clock), so
  `dapclass.rs` is a two-endpoint vendor interface. Hosts find it by the
  interface string "CMSIS-DAP v2 Interface"; the product string contains
  "CMSIS-DAP" too because OpenOCD checks it there. dap-rs supports only
  turnaround 1 with no forced data phase and ignores idle cycles, which is
  what probe-rs uses. No MS OS 2.0 descriptors: Windows would need WinUSB
  assigned by hand.
- SWD and nRESET float unless a session holds them: J2 shares the lines.
  They are released on DAP_Disconnect and also when USB drops out of the
  configured state, since a killed probe-rs never disconnects. nRESET is
  open drain by direction (R29 10k, C32 100 nF: ~1 ms rise). R32_PA_PIN is
  only valid for inputs, so pin readback of an output uses R32_PA_OUT.
- The SWD clock rate is approximate and errs slow: the delay loop assumes
  2 cycles per iteration and no overhead.

## IDs, permissions, flashing

- USB 1209:0001 is the pid.codes test ID, fine on the bench but not for
  boards that leave it. `BRIDGE_SERIAL=<name> mise run ch572-flash` sets
  the serial number to tell boards apart (default FSP-1).
- probe-rs's udev rules cover neither that ID nor the WCH ROM
  bootloader's (4348:55e0, 1a86:55e0). `mise run udev` installs
  `ch572/70-fsp-ch572.rules`, which covers all three.
- Flashing is over USB through the ROM bootloader with wchisp. CH570/CH572
  support is only on wchisp's main branch (added 2025-12, after 0.3.0), so
  mise.toml pins a commit. The board review says a blank chip enumerates
  on its own; the first board's did not, so use the PA1 pull-up below. wlink
  0.1.3 (WCH-LinkE) also supports the CH572 over its 2-wire debug pins,
  which are PA0/PA1, the USB lines, and which the firmware turns off when
  it attaches to USB.

## Getting back into the bootloader

The board has no BOOT button for the CH572D (review M3), so:

1. `mise run ch572-boot` (or any port open at 1200 baud then closed): the
   firmware detaches from USB and restarts through the datasheet's
   manual-BOOT path (RB_SOFTWARE_RESET with RB_WDOG_INT_EN set and
   R8_WDOG_COUNT in 128..191). That text names RB_ROM_CODE_WE = 1 and
   RB_ROM_DATA_WE = 0, bits from the CH58x; the CH572 merges them into the
   2-bit RB_ROM_CODE_WE, read here as bits 7:6 = 10b. No WCH example uses
   the path, so it is unverified: if the reading is wrong the chip only
   restarts the bridge, and `lsusb` shows 1209:0001 again instead of the
   WCH ISP device.
2. PA1 (D+) pulled up to VCC before power is applied (WCH's CH572
   evaluation board manual, section 4.3, in EVT/PUB of the repo above). On
   this board: 5.1k from R41's VDD33 end to D+, then plug in. Against the
   host's 15k pull-down D+ reads about 2.5 V; once the bootloader runs, its
   own 1.5k pull-up in parallel lifts it to about 3.0 V. Do not use +5V:
   PA1 is not 5 V tolerant (VIO33 + 0.4 V max).
3. A WCH-LinkE on D-/D+ with wlink, catching the chip before the firmware
   claims those pins for USB.
4. The ROM also takes serial downloads on PA0/PA1 with no pull-up (same
   manual, section 4.2), from a 3.3 V UART on D-/D+. The manual does not
   say which pin is TX.

## Board issues that touch the CH572D

From the Forest-Sensor-Platform BOARD-REVIEW.md (2026-08-22); the V1
netlist still has them, except B1 (the exposed-pad ground), which is
fixed (pad 21 is on GND):

- H1: VDD33 has no decoupling; C28 and C29 both sit on V5. If the CH572D
  resets or drops off USB under load, suspect this first.
- H3: the CH572D runs only from +5V (USB, or the boost under the Wio-E5's
  5V_EN). With it off, WIO-RST's pull-up and the LED resistors back-power
  it through its pins.
- M5: U7 can put the boost onto VBUS. The firmware holds USB_VOUT_EN (PA4)
  low and never sets it.

## First bring-up

Step 1 has passed on the first board (2026-11-03); step 3 is stuck there
(2026-11-04, see its notes); the rest has not run on hardware yet. In
order:

1. Power check before USB: 3.3 V into J4 (or J2 pin 3), nothing else
   attached, SD slot empty. Expect about 3-8 mA, 11 mA at datasheet
   maxima; much more points at a short or a misplaced part. The first
   board drew 200 mA with no short: D7 fitted backwards and put on the
   rail by R33 (Forest-Sensor-Platform BOARD-REVIEW.md, P1 and P2).
   - Wio-E5 awake with its radio off, 1-6 mA (DS13105 run current at
     4-48 MHz). Blank, it sits in ST's ROM bootloader (RM0461 empty
     check). The factory image goes to Seeed's upgrade mode instead of
     the AT firmware: PB13 is its boot pin and reads low through the solar
     divider without a panel.
   - U6 with its receiver on from the R36/R37 pulldowns: 1.0 mA, 2.2 max.
     U1: 0.26 mA max.
   - The unpowered CH572D leaks in through R29, R23/D10, R24/D11 and
     SWDIO: 0.6 mA at most, set by those resistors. D10/D11 may glow.
   - U10 is fed backwards and draws its ~30 uA ground current. Output
     above input is outside TI's absolute maximum (VOUT <= VIN + 0.3 V),
     harmless at that current, but use USB-C after this check.
   - WIO-RST (J2 pin 5) should read near 3.3 V. Much lower means the dead
     CH572D clamps it (H3) and holds the Wio-E5 in reset.
2. `mise install` (builds wchisp) and `mise run udev`.
3. USB-C in, and check VDD33 at R41's chip end: 3.2-3.4 V. Higher means
   the chip draws less than the ~1.13 mA that R41 feeds it, and the LDO
   cannot sink the rest. The first board's first CH572D read 4.5 V (above
   its 4.0 V absolute maximum) and was replaced. Then unplug, fit the PA1
   pull-up ("Getting back into the bootloader", method 2) and plug in:
   `lsusb` shows 4348:55e0 or 1a86:55e0. `mise run ch572-flash`, then
   remove the pull-up.
   - First board, replacement chip: VDD33 3.3 V, D+ 2.5 V with the
     pull-up, and the host logs a full-speed attach with -71 errors. The
     host sees only the external pull-up; the bootloader never turned on
     its own. Suspects: U9 pin 12 not soldered, or Y2 not oscillating
     (see Clock).
4. `lsusb` shows 1209:0001; dmesg shows a ttyACM and no descriptor errors.
   D10 lights once configured (blinks at 1 Hz until then).
5. `probe-rs list` shows the probe. The Wio-E5 ships read-protected
   (RDP 1); `mise run unlock` erases it (notes/build-flash.md; SW1 is the
   NRST button if reset needs holding). Then `probe-rs info --chip
   STM32WLE5JCIx` reads it (RM0461 gives DPIDR 0x5BA02477).
6. `mise run bootloader`, `ADDRESS=2 mise run flash --features board`,
   and RTT via `mise run attach`.
7. Serial: the basestation built with `board`, `mise run data-relay -p
   /dev/ttyACM0`.
8. `mise run ch572-boot`, then `lsusb` for the WCH ISP device and
   `wchisp info`.
