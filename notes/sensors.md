# I2C display, sensors and RS-422

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

## RS-422 link + I2C sensor suite (rs422 / sensor features)

Branch `peripherals`. Two new independent cargo features.

`rs422`: the Forest Sensor Platform's half-duplex RS-485 transceiver
on USART1 with DE/RE direction control; wiring and the TC pitfall are
in `notes/board.md` (RS-485 link). Includes a
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

## SEN0575 rain gauge on the sensor bus (`sensor` feature)

DFRobot's tipping-bucket gauge. The part is an I2C slave implemented in
firmware on a CS32L010, not a fixed-function sensor, which drives most
of the choices below. Register map (I2C, little-endian throughout):
0x00 PID+VID (4 B), 0x0A version, 0x0C windowed rainfall (4 B), 0x10
cumulative rainfall (4 B), 0x14 raw bucket tips (4 B), 0x18 uptime in
minutes (2 B), 0x26 window length in hours (write, 1 B), 0x28 mm per
tip x10000 (write, 2 B). Both rainfall registers count ten-thousandths
of a mm, so `mm_x100 = raw / 100` for the repo's centi-unit convention.

Presence check is the PID/VID pair rather than a chip-id byte: reading
4 B from 0x00 yields `C0 00 43 73`, unpacked as an 18-bit PID whose top
two bits ride in the spare bits of the VID's high byte (PID 0x100C0,
VID 0x3343).

Address conflict. The gauge is hard-wired to 0x1D, which is both the
ADXL345's alternate address and inside the MCP9808's strapped 0x18..0x1F
range - three drivers, one address, and the gauge cannot be moved. The
probe now runs the gauge first and the other two skip 0x1D once it is
found: a 32-bit PID/VID match is a much stronger claim on an address
than the ADXL's one-byte DEVID compare or the MCP's id-pair read, so
whichever part is actually fitted wins. An ADXL and a gauge still
cannot share a bus - nothing can fix that, it is the hardware.

Reads use a write-then-read with a STOP between, not a combined
`write_read` repeated start, matching what the vendor driver exercises.
Repeated start probably works, but a firmware I2C slave is exactly the
place where "probably" is not worth spending, and the two-transaction
form costs nothing here.

The 1 hour window register is written once at probe instead of before
every read (the vendor driver rewrites it each call). It is retained,
and every write to this device needs a ~100 ms save during which it
ignores the bus - so rewriting it per sweep would put 100 ms of blocking
delay into all of them for a value that never changes.

Telemetry key is `R` = windowed rainfall in hundredths of a mm, joining
T/H/C/M. Worst case the line is now over budget: `T=-4000 H=10000
C=40000 R=10000 M=1000` is 37 B against the 32 B payload, so a node
carrying every sensor at once would drop `M`. Left as is - `TelemetryLine`
already drops whole fields rather than truncating, and a node with a rain
gauge, a CO2 sensor and a soil probe on it is not a configuration that
exists. Worth revisiting if it ever does.
