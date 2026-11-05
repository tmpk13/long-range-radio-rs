# Forest Sensor Platform V1 board test

Tests one Forest Sensor Platform V1 board, from an assembled board that
has never been powered to a node that charges its battery and talks on
the mesh. Sections 1-5 run in order, each needing the one before it;
once section 5 passes, the rest run in any order. Tick the boxes in a
copy of this file per board (for example `v1-board-test-board2.md`) and
fill the blanks with readings.

Board: ________ Date: ________ long-range-radio-rs: ________ ch572-bridge-rs: ________

## Before you start

### Equipment

- Bench supply with a current limit; two channels make section 7 easier
  (one stands in for the panel, the other for the battery)
- Multimeter, oscilloscope with a 10x probe, IR thermometer
- Linux host with mise, probe-rs and OpenOCD; this repo and
  [ch572-bridge-rs][bridge] checked out; a USB-C cable
- 3S NiMH pack on an XT30 plug, partly charged (3.6-4.0 V)
- A 10 ohm resistor rated 5 W or more (section 7.6)
- 915 MHz antenna for SMA1
- For sections 8-10, a second node: any Wio-E5 running this repo's
  firmware, such as a second V1 board or the prototype board with a
  USB-UART on USART1
- microSD card; I2C sensors from the `sensor` feature on JST SH 4-pin
  leads; an SSD1306 display; a DFRobot SEN0600 soil probe

### Rules

1. Fit the 915 MHz antenna on SMA1 before the first node flash: the
   firmware transmits every 4-7 s.
2. Battery first, panel last: connect the pack before the panel, and
   remove the panel before the pack. Nothing clamps Battery_Out
   ([review] H4). If the pack comes off while charging, Battery_Out
   passes the 6 V maximum of U8, U10 and U13 in well under a
   millisecond, faster than the firmware can react.
3. Whatever is on J6 must sink current while a panel is connected: a
   real pack, or a supply with a load resistor as in 7.6. A bench supply
   alone cannot sink, and the charger drives its output up.
4. Turn the panel off before flashing or halting the Wio-E5 in a
   debugger. TIM2 keeps switching at the last duty while the core is
   stopped, and nothing checks the limits.
5. Fit R34 and leave R33 off ([review] P2): with both fitted, +5V
   reaches the 3.3 V rail and the Wio-E5.

### Expected on every V1

Known design faults, listed in [BOARD-REVIEW.md][review]; do not chase
them:

- D2 and D6, the LM66200 status LEDs, never light (H5).
- D3 lights whenever Battery_Out is up and draws 2-3 mA (L2).
- On battery the CH572D is off unless the 5 V rail is on, and the
  Wio-E5's pins back-power it (H3), so D10 and D11 may glow faintly.
- The CH572D has no boot button (M3); re-entering its bootloader is in
  [ch572-bridge-rs NOTES.md][bridge-boot].
- J7's RS-485 lines have no TVS, termination or fail-safe bias (M4), and
  R35 (100 ohm) drops the sensor supply 1 V per 10 mA (M6).

## 1. Unpowered

Rework every V1 needs before power:

- [ ] R33 off, R34 fitted ([review] P2)
- [ ] D7 is the bidirectional SMAJ5.0CA, or a unidirectional part with
  its cathode on the sensor-supply side, away from the silkscreen mark
  ([review] P1)
- [ ] Y2 turned 90 degrees, so XI and XO are not shorted ([review] P3)
- [ ] C26 and C27 off ([review] H2); 2.2 uF and 100 nF added on VDD33 at
  R41's U9 end ([review] H1)
- [ ] U9 flat on a thin exposed-pad layer ([review] P4)

Then:

- [ ] Cathode bands of D1, D4, D5 and D8 on their silkscreen marks; Q2's
  pin 1 on its footprint's pin 1 mark
- [ ] The unpowered rows of the Checks table in
  [CH572D-BRINGUP.md][ch572-bringup] pass
- [ ] Nothing reads below 100 ohm to GND: +3.3V (J4 pin 1), +5V (C28),
  VUSB (C31), Battery_Out (C22), Solar In (TP5), Solar+ (J14 pin 3)

## 2. First power at J4

Step 1 of the [bridge's First bring-up][bridge-bringup] has the setup
and the expected current.

- [ ] 3.3 V into J4 with nothing else attached draws ____ mA, inside
  step 1's range
- [ ] WIO-RST (J2 pin 5) near 3.3 V: ____ V
- [ ] J4 supply off before section 3 (U10 is fed backwards while it is
  on)

## 3. USB-C and the CH572D bridge

Steps 2-4 of the [bridge's First bring-up][bridge-bringup], run from
ch572-bridge-rs:

- [ ] Step 2: udev rules installed on this host
- [ ] Step 3: VDD33 ____ V with USB-C in; the bridge flashed and
  verified
- [ ] Step 4: the bridge enumerates; D10 steady
- [ ] On USB alone: +5V (C28) ____ V, +3.3V (TP2) ____ V
- [ ] Optional, step 5: `mise run boot` reaches the ROM bootloader (not
  yet seen on hardware)

## 4. Wio-E5 through the bridge

- [ ] `probe-rs list` shows the bridge's CMSIS-DAP probe
- [ ] `probe-rs info --chip STM32WLE5JCIx` reads the debug port; its
  error about AP1 at 0xf0000000 is harmless
  ([notes/build-flash.md](../notes/build-flash.md))
- [ ] `mise run unlock-check` prints `RDP level 0: unlocked`. A new
  Wio-E5 reports level 1: `mise run unlock`, unplug USB-C to power-cycle
  the board, then check again
  ([Read protection](../notes/build-flash.md#read-protection))

## 5. Node firmware

- [ ] Antenna on SMA1 (rule 1)
- [ ] `mise run bootloader` flashes the bootloader (no RTT output;
  Ctrl-C once flashed)
- [ ] `ADDRESS=2 mise run flash --features board` flashes and attaches
  RTT, which shows `Starting... 1`, `Starting... 2`,
  `I2C display not detected, will retry`,
  `SD card init failed (no card?): Timeout` and `Mesh node 2 ready`,
  then every 4-7 s a `Board: vin=... duty=0 Idle` line and `TX #n`
- [ ] D12 flashes briefly on every TX
- [ ] With a microSD card in, SW1 restarts the node, and a fresh
  `mise run attach` shows `SD card ready: Sdhc` (`Sd2` or `Sd1` for
  small cards) and the TX count starting from 1

## 6. Power paths

The node firmware from section 5 runs; nothing on J14.

- [ ] Battery stand-in: bench supply at 3.9 V, 300 mA limit, on J6
  (XT30 pin 1 positive), USB-C out. The node runs: D12 flashes every
  4-7 s, and WIO-RST (J2 pin 5) reads ____ V. If the node stays dark
  with WIO-RST low, the unpowered CH572D is clamping it ([review] H3).
- [ ] Battery current with the node idle: ____ mA (D3 alone is 2-3 mA)
- [ ] USB-C back in: the battery current falls to a few mA, as U8 moves
  the 3.3 V rail onto VBUS
- [ ] Note the TX count in `mise run attach`, unplug USB-C for a minute,
  plug it back in and attach again: the count kept rising (a reset
  restarts it at 1)

## 7. Charger

Read rules 2-4 first. The node prints `Board: vin=<mV> vbat=<mV>
ibat=<mA> duty=<permille> <state>` every 4-7 s, the state being `Idle`,
`Tracking` or `Limiting`; flashing with `--features board,debug` adds
the controller's `MPPT:` line every 200 ms. Probe points: TP8 is
PWM_Buck, TP6 the Q1 bases, TP5 Solar In, D8's cathode the switch node,
and R56's Q2 end the Q2 gate. The panel supply goes on J14: pins 3 and 4
positive, 1 and 2 GND.

### 7.1 Gate drive with no firmware

No pack, no USB-C: nothing powers the Wio-E5, so R55 alone must hold the
switch off.

- [ ] Panel supply at 20 V with a 50 mA limit on J14 draws about 1 mA:
  ____ mA
- [ ] TP5 a few tenths of a volt below 20 V, TP6 equal to TP5, TP8 at
  0 V
- [ ] D1's anode about 12 V below TP5: ____ V
- [ ] Battery_Out (C22) near 0 V: ____ V
- [ ] Panel supply off

### 7.2 Senses at rest

Pack on J6, USB-C in, node running, no panel.

- [ ] vin under 100 mV, ibat 0, duty 0, `Idle`
- [ ] vbat within 2% of a meter across J6: ____ mV against ____ mV

### 7.3 First switching

As in 7.2, with the scope on TP8 and D8's cathode.

- [ ] Panel supply with a 100 mA limit, set to vbat + 1.2 V: stays
  `Idle` (it starts at vbat + 1.5 V)
- [ ] Panel supply to 12 V: `Tracking` within a second, and vin within
  3% of a meter on J14: ____ mV against ____ mV
- [ ] TP8 at 100 kHz within 3%: ____ kHz; its duty within one timer
  count (0.625%) of `duty`
- [ ] `duty` rises 13 per step (watch TP8, or the `debug` line) until
  the supply reaches its limit, then wanders around that point
- [ ] Switch node: a square wave from about -0.4 V (D8 conducting) up to
  TP5. Its high time is longer than TP8's (the gate turn-off delay, up
  to 2x at low duty in the simulation): TP8 ____ ns, switch node ____ ns,
  at duty ____

### 7.4 Tracking a current-limited source

- [ ] Panel supply at 20 V, 100 mA limit. After 30 s the supply sits at
  its limit, J14 stays well above vbat + 1 V, and ibat holds within 20%
  of 0.85 x 20 V x 0.1 A / vbat (about 440 mA at 3.9 V): ibat ____ mA,
  J14 ____ V
- [ ] Limit changed to 50 mA, then to 200 mA: ibat follows within 30 s
  each time: ____ mA, ____ mA

### 7.5 Current ceiling

- [ ] Panel supply at 24 V, 1.5 A limit. ibat rises to about 1 A and
  holds there, showing `Limiting` at times, and never goes above about
  1.2 A (one step past the ceiling): highest ibat ____ mA
- [ ] ibat is 20 x the millivolts across R1, within 5%: ____ mV,
  ____ mA
- [ ] Q2 gate swings between about TP5 (off) and TP5 - 12 V (on)
- [ ] `duty` at 1 A: ____ (the simulation predicts about 147)
- [ ] Efficiency, vbat x ibat over the panel supply's volts x amps:
  ____ % (the simulation gives about 87%)
- [ ] After 10 minutes at 1 A nothing reads above 85 C: D8 ____,
  L6 ____, Q2 ____, D5 ____, R1 ____

### 7.6 Voltage ceiling

This needs a battery stand-in that can sink current: the bench supply
with the 10 ohm resistor across J6 (rule 3).

- [ ] Panel supply off, pack off J6, stand-in on J6 at 4.30 V
- [ ] Panel supply at 20 V with a 50 mA limit, so the charger cannot
  push more than the resistor draws (430 mA): `Limiting` and `duty` 0
  within two steps, and it stays there
- [ ] Stand-in lowered to 4.00 V: back to `Tracking`
- [ ] Panel supply off, then the pack back on J6

### 7.7 Stopping

Charging as in 7.4 or 7.5.

- [ ] Panel supply off: `Idle`, `duty` 0 and TP8 flat within a second
- [ ] Panel supply on again, then lowered below vbat + 1 V: `Idle`
- [ ] SW1 while charging: TP8 stays low through the reset, and the
  charger starts again from `duty` 13
- [ ] USB-C out while charging: the node keeps running on Battery_Out and
  keeps charging (D12 still flashes; the TX count check from section 6)

### 7.8 Real panel (optional)

- [ ] 24 V panel in sun, pack connected first: `Tracking` or
  `Limiting`; readings at a few times of day: ________
- [ ] After dark: `Idle`, `duty` 0

## 8. Radio

This needs the second node, flashed with ADDRESS=3 and the same
LORA_PRESET as the board.

- [ ] The board's RTT shows `RX #n from=3 rssi=<n> dBm: ...` every 4-7 s,
  and the second node's shows `from=2` with the board's
  `V=... B=... I=... D=... i` heartbeat
- [ ] No `TX #n failed` line on either node for five minutes
- [ ] D12 flashes on each RX as well as each TX
- [ ] RSSI at 1 m: ____ dBm; optional range test: ____ dBm at ____ m

## 9. Basestation on the board

The board runs the basestation, with its host link through the CH572D's
serial port; the second node (ADDRESS=3) runs the node firmware.

- [ ] `ADDRESS=10 mise run basestation --features board` on the board
- [ ] `mise run data-relay -p /dev/ttyACM0` prints a JSON line for each
  of node 3's heartbeats: `src` 3, its `rssi`, and the message in base64
  under `data`
- [ ] With a panel connected, TP8 stays at 0 V: the basestation never
  drives the buck, so a board running it does not charge its pack

## 10. OTA update into the board

This needs the second node as the basestation: `ADDRESS=10 mise run
basestation` on it (add `--features board` if it is a V1 board), on
PORT (`/dev/ttyUSB0` through a USB-UART, `/dev/ttyACM0` through a
CH572D). The update scheme is in
[ARCHITECTURE.md](../ARCHITECTURE.md#over-the-air-update).

- [ ] The board runs `ADDRESS=2 mise run flash --features board`, which
  is version 1
- [ ] `ADDRESS=2 FW_VERSION=2 mise run ota-image --features board`
  writes `target/firmware.bin`
- [ ] `mise run ota-upload -p PORT -t 2 -v 2 target/firmware.bin` ends
  with `OTA transfer successful! Target node will reboot.`: took
  ____ min
- [ ] Press SW1 (any reset will do): the node swaps the image in at
  its next reset, not at the end of the transfer ([notes/boot-ota.md](../notes/boot-ota.md#known-gaps-an-early-boot-confirm-and-no-reboot-after-a-transfer))
- [ ] `mise run data-relay -p PORT` shows the board's heartbeats again
- [ ] The same upload again ends with
  `OTA transfer finished with result: 0x02`: the board refuses
  version 2 because it now runs it

## 11. I2C sensors and display

- [ ] `ADDRESS=2 mise run flash --features board,sensor` with sensors on
  J8 and J10 (both on I2C2, with 4.7k pull-ups on the board):
  `I2C sensors found:` lists every fitted part
- [ ] Every 30 s the readings print, and a `T=... H=...` line goes out
  over the mesh (keys in [notes/sensors.md](../notes/sensors.md))
- [ ] SSD1306 on J8 or J10: `I2C display detected`, and the display
  shows `Out: V=...` on each TX
- [ ] Display unplugged: `I2C display lost`; plugged back in:
  `I2C display reconnected` within 10 s; the node keeps running
  throughout

## 12. RS-485 soil probe and the 5 V rail

- [ ] SEN0600 on J7: A to pin 6, B to pin 5, supply to pin 1, GND to
  pin 2
- [ ] `ADDRESS=2 mise run flash --features rs422`: every 30 s RTT prints
  `Soil: <moisture> % <temperature> C`, and `M=...` goes out over the
  mesh
- [ ] On battery (USB-C out), J7 pin 1 comes up about 2 s before each
  sweep and drops after it: ____ V while up. D10 blinks while the rail
  is up, since the CH572D runs from it
- [ ] With USB-C in, J7 pin 1 stays up: U2 feeds +5V from VBUS whatever
  5V_EN does

## 13. GPS log (optional)

This needs an NMEA GPS talking RS-485 into J7, and a microSD card
([notes/gpslog.md](../notes/gpslog.md)).

- [ ] `ADDRESS=2 mise run flash --features gps-radio-log`: RTT shows
  `gps-radio-log: logging at block <n>` and `GPS $..RMC` and
  `GPS $..GGA` sentences
- [ ] With a display fitted, it shows the status line (satellites, last
  RX RSSI, time since that RX) and the position

## 14. Soak (optional)

- [ ] 48 hours outdoors on pack and panel, with a basestation logging
  `mise run data-relay` to a file: state `i` at night, `t` or `l` by
  day, B never above about 4250, heartbeats arriving throughout

## Not covered

- OTA rollback, where a new image never confirms its boot and the
  bootloader restores the old one
  ([ARCHITECTURE.md](../ARCHITECTURE.md#boot-state)): it needs a
  deliberately broken image, and only one that dies in the first lines
  of `init` is reverted today ([notes/boot-ota.md](../notes/boot-ota.md#known-gaps-an-early-boot-confirm-and-no-reboot-after-a-transfer)).
- The CH572D's 2.4 GHz radio on J9: the bridge firmware does not use it.
- Pulling the pack while charging (rule 2).

[review]: https://github.com/tmpk13/Forest-Sensor-Platform/blob/main/BOARD-REVIEW.md
[ch572-bringup]: https://github.com/tmpk13/Forest-Sensor-Platform/blob/main/CH572D-BRINGUP.md
[bridge]: https://github.com/tmpk13/ch572-bridge-rs
[bridge-bringup]: https://github.com/tmpk13/ch572-bridge-rs/blob/main/NOTES.md#first-bring-up
[bridge-boot]: https://github.com/tmpk13/ch572-bridge-rs/blob/main/NOTES.md#getting-back-into-the-bootloader
