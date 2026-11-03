# Forest Sensor Platform board

## Board support (`board` feature)

Feature-gated support for the Forest Sensor Platform V1 board
(`~/forest/Forest-Sensor-Platform`, KiCad 10). The pin map lives in the
`src/board.rs` module doc and in ARCHITECTURE.md; it was taken from a
kicad-cli netlist export of `Forest-Sensor-Platform.kicad_sch`, which
pulls in the buck and USB sheets. The board's predecessor, the
`buck-converter-mppt-nimh` charger board, is no longer supported; git
history before this note has its pin map.

What changed against the old charger board, and why it matters:

- Senses moved: PB13 = VSENSE_VIN (100k:10k, x11, ~36 V full scale),
  PA10 = VSENSE_OUT (10k:10k, x2), PB14 = BAT_ISENSE. All three have a
  1k / 100 nF RC filter at the pin.
- Battery current is now an INA181A1 (gain 20) across the 50 mOhm shunt
  R1, so the pin reads 1 V/A and 1 mV = 1 mA (~0.8 mA per LSB, against
  ~16 mA per LSB for the old bare shunt). REF is grounded, so discharge
  reads 0. The shunt sits between Battery_Out and the pack, downstream
  of every load, so the reading is net charge current.
- Buck PWM moved to PB10 = TIM2_CH3 (AF1, confirmed in the STM32WLE5
  datasheet AF table). TIM2 is on APB1, also at sysclk. PWM high turns
  the switch on: PB10 -> Q3 (BSS123) pulls the Q1 push-pull's bases
  down, Q1 pulls the Q2 PMOS gate to the D1 zener rail 12 V below the
  panel. R55 keeps the switch off until the timer owns the pin.
- No duty clamp below 100%: the old 95% limit protected the LM5109B
  bootstrap, and the PMOS drive has no bootstrap. A CCR above ARR keeps
  the output high for the whole period.
- Per the buck simulation (Q2-PMOS-BUCK-SIM.md in the board repo), 1 A
  needs only ~0.147 commanded duty from 24 V, and current is very
  sensitive to duty (0.01 -> ~0.07 A, 0.06 -> ~0.24 A). The 13 permille
  P&O step is ~2 TIM2 counts and moves the current ~0.2 A near 1 A.
  Commanded and delivered duty differ by up to 2x (gate turn-off
  delay), so duty is not comparable with the old board's.
- PC0 is now 5V_EN (boost enable), not an LED. Old firmware pulses it on
  every TX, which would toggle the 5 V rail: do not run pre-FSP builds
  on this board. PA9 is RS-485 DE now; old builds put the 100 kHz buck
  PWM on it.
- The SD socket's card-detect pad is not wired and PB9 is RS-485 /RE,
  so `SdCard` has no detect pin: a failed CMD0 is how a missing card
  shows up (R18 holds DAT0 high, so the first CMD0 times out in ~1 ms).

Design choices carried over:

- The `board` build raises MSI from 4 to 16 MHz (`board::raise_sysclk`,
  called before the SysTick monotonic starts): 160 duty steps at the
  100 kHz switching frequency, still zero flash wait states.
  `SYSCLK_HZ` is cfg-gated in `platform.rs`; the I2C/SubGHz drivers
  derive their timing from RCC at init so they follow automatically.
- stm32wlxx-hal 0.6 has no timer support, so the PWM is set up through
  the PAC directly. ADC uses the HAL (`Adc::pin`), with VREFINT factory
  calibration to correct for the actual 3V3 rail.
- SD card: minimal in-repo SPI-mode driver (CMD0/8/ACMD41/58, single
  block read/write) instead of embedded-sdmmc, because embedded-sdmmc
  0.7+ needs embedded-hal 1.0 (HAL only implements 0.2) and older
  versions want a `FullDuplex` impl the HAL also lacks. Init at 250 kHz
  (Div64), then 8 MHz (Div2) via a direct CR1.BR write (HAL has no baud
  setter).
- The RTIC `Local` struct uses `type BoardRes = Board` / `()` cfg alias
  so resource plumbing is identical with the feature off.
- With `board`, the heartbeat broadcasts `V=<mV> B=<mV> I=<mA>
  D=<permille> <i|t|l>` instead of "hello" (see the MPPT section).

Build with: `ADDRESS=n mise run flash --features board`

## MPPT charge controller (board feature)

Perturb & observe MPPT in `board::Mppt`, stepped every 200 ms from the
main RTIC loop (`Board::mppt_step`). Design points:

- The shunt is on the battery side, so the controller hill-climbs on
  output power (vbat * ibat). Buck efficiency is flat enough across the
  range that the output maximum coincides with the panel MPP.
- States: Idle (vin < vbat + 1 V, PWM parked, 500 mV re-entry
  hysteresis so a marginal panel does not flap), Tracking (P&O, 13
  permille ~= 2 TIM2 counts per step, direction reverses only when
  power drops by more than 40 mW), Limiting (vbat > 4.2 V or ibat >
  1 A: back off 26 permille per step, resume tracking downhill). 1 A is
  the buck's design point (L6 at 1.6x below saturation).
- Noise: 16x hardware oversampling (enabled before `Adc::enable`, result
  stays 12-bit) plus a 4-sweep software average per step; the 40 mW
  hysteresis absorbs the rest. With the INA181 the averaging mostly
  smooths switching ripple that gets past the RC filters.
- Duty is tracked in the controller, not read back from the timer: CCR
  quantization (~6 permille) makes register round-trips lossy.
- NiMH termination is CV float at 1.4 V/cell, not -dV detection: with
  solar input the current varies with irradiance anyway, so a -dV
  signature is unreliable. The 4.2 V ceiling suits the 3S NiMH pack.
- Heartbeat broadcast format: `V=<mV> B=<mV> I=<mA> D=<permille>
  <i|t|l>` (state letter). Worst case 31 bytes, fits the 32-byte mesh
  payload.

## Activity LED and switched 5 V rail (board feature)

D12 (PC1) is the only LED the Wio-E5 drives: cathode on the pin, anode
through 10k to +3.3V, so low = on. It pulses 30 ms on radio TX and RX
alike. Hooked at the Sx1262Driver level (note_tx after set_tx, note_rx
on packet read) rather than in the app loop, so mesh-internal traffic
and forwarding blink too; the driver sets lock-free flags in
board::activity and `ActivityLed::update` in the main loop starts and
retires the pulse.

PC0 (5V_EN) switches U13 (battery load switch) and so the U11 boost.
`Board::rail_5v` owns it. The rail feeds the J7 sensor supply (through
R34/R35; R33 is DNP) and the CH572D. With USB plugged in, U2 feeds +5V
from VBUS regardless, so the CH572D only goes dark on battery.

## RS-485 link (rs422 feature)

TPT482 (U6) on USART1: PB6 TX -> D, PB7 RX <- R, PA9 = DE (active high,
R36 1M pulldown), PB9 = /RE (active low, R37 1M pulldown). The schematic
net names for DE and /RE are swapped (`Receiver_Output_Enable` is DE);
the netlist pins are what count. R42/R43 strap A-Y and B-Z, making a
2-wire half-duplex bus, so `Rs422::write_all` turns the receiver off and
the driver on for the request, then reverses both.

- DE must stay up until ISR.TC. The HAL's serial `flush` polls ISR.BUSY,
  which is the receiver's busy flag, so it returns while the last byte
  is still shifting out; dropping DE on it truncates the request.
- `rs422` implies `board`: the probe is powered from the switched 5 V
  rail. The main loop turns the rail on `SOIL_WARMUP_MS` (2 s) before
  the 30 s sweep and off after the poll. 2 s is a guess with margin; the
  SEN0600's start-up time has not been measured.
- `rs422` and `gps-radio-log` both need USART1, so lib.rs refuses the
  combination with a compile_error.
