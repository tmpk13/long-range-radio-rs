# Charger board

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

## Radio activity LEDs (board feature)

LED1 (PC0) pulses on radio TX, LED2 (PC1) on RX; both active high
(pin -> LED -> 10k -> GND per schematic). Hooked at the Sx1262Driver
level (note_tx after set_tx, note_rx on packet read) rather than in
the app loop, so mesh-internal traffic and future packet forwarding
blink too. The driver sets lock-free atomic flags in board::activity;
Leds::update in the main loop starts/retires 30 ms pulses, avoiding
any GPIO coupling inside the radio driver.
