//! I2C environmental sensor suite (`sensor` cargo feature).
//!
//! All sensors hang off the module's single exposed I2C bus (I2C2,
//! PB15 = SCL / PA15 = SDA, broken out on the charger board's "I2C"
//! header), shared with the SSD1306 display through [`SharedI2c`].
//!
//! | Sensor    | Address(es) | Measures                        | Support        |
//! |-----------|-------------|---------------------------------|----------------|
//! | BME680    | 0x76, 0x77  | temp, pressure, humidity, gas   | full           |
//! | SCD41     | 0x62        | CO2, temp, humidity             | full           |
//! | ADXL345/3 | 0x53, 0x1D  | acceleration                    | full           |
//! | AS3935    | 0x03..0x01  | lightning                       | poll, no IRQ   |
//! | BMV080    | 0x57, 0x56  | particulate matter              | presence only  |
//!
//! The BMV080's register protocol is only documented inside Bosch's
//! closed-source vendor library, so it is detected but not read.
//! The AS3935 interrupt line is not wired to the module; events are
//! polled from INT_SRC, which is best-effort (the datasheet wants the
//! IRQ serviced promptly).  Wire it to GPIO-10 (PB10) later for real
//! event capture.
//!
//! Everything on the bus is probed at startup; absent sensors are
//! skipped forever after.  All reads are blocking with millis-based
//! timeouts; the longest (BME680 with the gas heater) is ~250 ms.

use crate::platform::{millis, SYSCLK_HZ};
use core::cell::RefCell;
use stm32wlxx_hal::embedded_hal::blocking::i2c::{Read, Write, WriteRead};

/// Spin for approximately `ms` milliseconds.
fn delay_ms(ms: u32) {
    cortex_m::asm::delay(ms * (SYSCLK_HZ / 1000));
}

// ---------------------------------------------------------------------------
// Bus sharing

/// Zero-cost I2C bus splitter for bus members living in the same task.
///
/// Wraps the bus in a `RefCell` behind a `&'static` (make one with
/// `cortex_m::singleton!`), handing out any number of `Copy` handles.
/// Each transaction borrows the bus only for its own duration.  Not
/// interrupt-safe: all holders must run at the same priority.
pub struct SharedI2c<T: 'static> {
    bus: &'static RefCell<T>,
}

impl<T> SharedI2c<T> {
    pub fn new(bus: &'static RefCell<T>) -> Self {
        Self { bus }
    }
}

impl<T> Clone for SharedI2c<T> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<T> Copy for SharedI2c<T> {}

impl<T: Write> Write for SharedI2c<T> {
    type Error = T::Error;
    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.bus.borrow_mut().write(addr, bytes)
    }
}

impl<T: Read> Read for SharedI2c<T> {
    type Error = T::Error;
    fn read(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.borrow_mut().read(addr, buf)
    }
}

impl<T: WriteRead> WriteRead for SharedI2c<T> {
    type Error = T::Error;
    fn write_read(&mut self, addr: u8, bytes: &[u8], buf: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.borrow_mut().write_read(addr, bytes, buf)
    }
}

// ---------------------------------------------------------------------------
// Readings

#[derive(Debug, Clone, Copy, Default)]
pub struct BmeReading {
    /// Air temperature, centidegrees C.
    pub temp_c_x100: i32,
    /// Barometric pressure, Pa.
    pub press_pa: u32,
    /// Relative humidity, centipercent.
    pub hum_pct_x100: u32,
    /// Gas sensor resistance, Ohm (air quality proxy: lower = more VOCs).
    pub gas_ohm: u32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Scd41Reading {
    pub co2_ppm: u16,
    /// Centidegrees C.
    pub temp_c_x100: i32,
    /// Centipercent RH.
    pub hum_pct_x100: u32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct AccelReading {
    /// Milli-g per axis.
    pub x_mg: i32,
    pub y_mg: i32,
    pub z_mg: i32,
}

#[derive(Debug, Clone, Copy)]
pub struct LightningEvent {
    /// Estimated storm head distance in km; 63 = out of range.
    pub distance_km: u8,
    /// Raw INT_SRC bits (0x08 lightning, 0x04 disturber, 0x01 noise).
    pub int_src: u8,
}

/// One sweep of everything that was found; absent/failed sensors are None.
#[derive(Debug, Clone, Copy, Default)]
pub struct Readings {
    pub bme: Option<BmeReading>,
    pub co2: Option<Scd41Reading>,
    pub accel: Option<AccelReading>,
    pub lightning: Option<LightningEvent>,
}

/// Which devices answered the startup probe.
#[derive(Debug, Clone, Copy, Default)]
pub struct Found {
    pub bme680: Option<u8>,
    pub scd41: bool,
    pub adxl345: Option<u8>,
    pub as3935: Option<u8>,
    pub bmv080: Option<u8>,
}

// ---------------------------------------------------------------------------
// BME680 calibration

#[derive(Debug, Clone, Copy, Default)]
struct BmeCal {
    t1: u16,
    t2: i16,
    t3: i8,
    p1: u16,
    p2: i16,
    p3: i8,
    p4: i16,
    p5: i16,
    p6: i8,
    p7: i8,
    p8: i16,
    p9: i16,
    p10: u8,
    h1: u16,
    h2: u16,
    h3: i8,
    h4: i8,
    h5: i8,
    h6: u8,
    h7: i8,
    g1: i8,
    g2: i16,
    g3: i8,
    res_heat_range: u8,
    res_heat_val: i8,
    range_sw_err: i8,
}

// ---------------------------------------------------------------------------

pub struct Sensors<I2C> {
    i2c: I2C,
    pub found: Found,
    bme_cal: BmeCal,
}

impl<I2C, E> Sensors<I2C>
where
    I2C: Write<Error = E> + Read<Error = E> + WriteRead<Error = E>,
{
    /// Probe the bus and configure whatever answers.
    pub fn probe(i2c: I2C) -> Self {
        let mut s = Self {
            i2c,
            found: Found::default(),
            bme_cal: BmeCal::default(),
        };

        // BME680: chip id 0x61 at register 0xD0.
        for addr in [0x76u8, 0x77] {
            if matches!(s.reg_read(addr, 0xD0), Ok(0x61)) {
                s.found.bme680 = Some(addr);
                if s.bme_init(addr).is_err() {
                    s.found.bme680 = None;
                }
                break;
            }
        }

        // ADXL345/ADXL343: DEVID 0xE5 at register 0x00 (same for both).
        for addr in [0x53u8, 0x1D] {
            if matches!(s.reg_read(addr, 0x00), Ok(0xE5)) {
                s.found.adxl345 = Some(addr);
                // Full-resolution +-2g, then measurement mode.
                let _ = s.i2c.write(addr, &[0x31, 0x08]);
                let _ = s.i2c.write(addr, &[0x2D, 0x08]);
                break;
            }
        }

        // SCD41: no chip-id register; ask for the serial number (only
        // answered while idle) and start periodic measurement.
        if s.scd41_serial().is_ok() {
            s.found.scd41 = true;
            let _ = s.scd41_cmd(0x21B1); // start_periodic_measurement
        }

        // AS3935: address set by the breakout's A0/A1 straps.
        for addr in [0x03u8, 0x02, 0x01] {
            if s.reg_read(addr, 0x00).is_ok() {
                s.found.as3935 = Some(addr);
                // Direct commands: preset defaults, then recalibrate
                // the RC oscillators.
                let _ = s.i2c.write(addr, &[0x3C, 0x96]);
                let _ = s.i2c.write(addr, &[0x3D, 0x96]);
                delay_ms(3);
                break;
            }
        }

        // BMV080: protocol lives in Bosch's vendor blob; detect only.
        for addr in [0x57u8, 0x56] {
            let mut byte = [0u8];
            if s.i2c.read(addr, &mut byte).is_ok() {
                s.found.bmv080 = Some(addr);
                break;
            }
        }

        s
    }

    /// Read every present sensor.  `Readings::lightning` is only `Some`
    /// when an event was pending in the AS3935.
    pub fn read(&mut self) -> Readings {
        Readings {
            bme: self
                .found
                .bme680
                .and_then(|addr| self.bme_read(addr).ok()),
            co2: if self.found.scd41 {
                self.scd41_read().ok()
            } else {
                None
            },
            accel: self
                .found
                .adxl345
                .and_then(|addr| self.adxl_read(addr).ok()),
            lightning: self.found.as3935.and_then(|addr| self.as3935_poll(addr)),
        }
    }

    fn reg_read(&mut self, addr: u8, reg: u8) -> Result<u8, E> {
        let mut byte = [0u8];
        self.i2c.write_read(addr, &[reg], &mut byte)?;
        Ok(byte[0])
    }

    // -- ADXL345 ------------------------------------------------------------

    fn adxl_read(&mut self, addr: u8) -> Result<AccelReading, E> {
        let mut raw = [0u8; 6];
        self.i2c.write_read(addr, &[0x32], &mut raw)?;
        let axis = |lo: u8, hi: u8| i16::from_le_bytes([lo, hi]) as i32;
        // Full-resolution mode: 3.9 mg/LSB.
        Ok(AccelReading {
            x_mg: axis(raw[0], raw[1]) * 39 / 10,
            y_mg: axis(raw[2], raw[3]) * 39 / 10,
            z_mg: axis(raw[4], raw[5]) * 39 / 10,
        })
    }

    // -- SCD41 --------------------------------------------------------------

    fn scd41_cmd(&mut self, cmd: u16) -> Result<(), E> {
        self.i2c.write(0x62, &cmd.to_be_bytes())?;
        delay_ms(1);
        Ok(())
    }

    /// Read `words` 16-bit words, each followed by a CRC-8 byte.
    fn scd41_get(&mut self, cmd: u16, words: &mut [u16]) -> Result<(), ()> {
        self.scd41_cmd(cmd).map_err(|_| ())?;
        let mut raw = [0u8; 9];
        let len = words.len() * 3;
        self.i2c.read(0x62, &mut raw[..len]).map_err(|_| ())?;
        for (i, word) in words.iter_mut().enumerate() {
            let chunk = &raw[3 * i..3 * i + 3];
            if scd41_crc(&chunk[..2]) != chunk[2] {
                return Err(());
            }
            *word = u16::from_be_bytes([chunk[0], chunk[1]]);
        }
        Ok(())
    }

    fn scd41_serial(&mut self) -> Result<(), ()> {
        let mut words = [0u16; 3];
        self.scd41_get(0x3682, &mut words)
    }

    fn scd41_read(&mut self) -> Result<Scd41Reading, ()> {
        // Data ready? (lower 11 bits nonzero)
        let mut word = [0u16; 1];
        self.scd41_get(0xE4B8, &mut word)?;
        if word[0] & 0x07FF == 0 {
            return Err(()); // measurement not ready yet (5 s cadence)
        }
        let mut words = [0u16; 3];
        self.scd41_get(0xEC05, &mut words)?;
        Ok(Scd41Reading {
            co2_ppm: words[0],
            temp_c_x100: -4500 + 17500 * words[1] as i32 / 65535,
            hum_pct_x100: 10000 * words[2] as u32 / 65535,
        })
    }

    // -- AS3935 -------------------------------------------------------------

    fn as3935_poll(&mut self, addr: u8) -> Option<LightningEvent> {
        let int_src = self.reg_read(addr, 0x03).ok()? & 0x0F;
        if int_src == 0 {
            return None;
        }
        let distance_km = self.reg_read(addr, 0x07).unwrap_or(0x3F) & 0x3F;
        Some(LightningEvent {
            distance_km,
            int_src,
        })
    }

    // -- BME680 -------------------------------------------------------------

    fn bme_init(&mut self, addr: u8) -> Result<(), E> {
        // Soft reset, then pull both calibration blocks.
        self.i2c.write(addr, &[0xE0, 0xB6])?;
        delay_ms(5);

        let mut c1 = [0u8; 25];
        let mut c2 = [0u8; 16];
        self.i2c.write_read(addr, &[0x89], &mut c1)?;
        self.i2c.write_read(addr, &[0xE1], &mut c2)?;
        let u16le = |lo: u8, hi: u8| u16::from_le_bytes([lo, hi]);
        let i16le = |lo: u8, hi: u8| i16::from_le_bytes([lo, hi]);
        self.bme_cal = BmeCal {
            t1: u16le(c2[8], c2[9]),
            t2: i16le(c1[1], c1[2]),
            t3: c1[3] as i8,
            p1: u16le(c1[5], c1[6]),
            p2: i16le(c1[7], c1[8]),
            p3: c1[9] as i8,
            p4: i16le(c1[11], c1[12]),
            p5: i16le(c1[13], c1[14]),
            p6: c1[16] as i8,
            p7: c1[15] as i8,
            p8: i16le(c1[19], c1[20]),
            p9: i16le(c1[21], c1[22]),
            p10: c1[23],
            h1: ((c2[2] as u16) << 4) | (c2[1] as u16 & 0x0F),
            h2: ((c2[0] as u16) << 4) | (c2[1] as u16 >> 4),
            h3: c2[3] as i8,
            h4: c2[4] as i8,
            h5: c2[5] as i8,
            h6: c2[6],
            h7: c2[7] as i8,
            g1: c2[12] as i8,
            g2: i16le(c2[10], c2[11]),
            g3: c2[13] as i8,
            res_heat_range: (self.reg_read(addr, 0x02)? >> 4) & 0x03,
            res_heat_val: self.reg_read(addr, 0x00)? as i8,
            range_sw_err: (self.reg_read(addr, 0x04)? as i8) >> 4,
        };

        // Gas heater: 320 C for ~148 ms each measurement.
        let res_heat = self.bme_heater_code(320, 25);
        self.i2c.write(addr, &[0x5A, res_heat])?;
        self.i2c.write(addr, &[0x64, 0x65])?; // gas_wait_0: 37 * 4 ms
        self.i2c.write(addr, &[0x71, 0x10])?; // run_gas, profile 0
        Ok(())
    }

    /// Heater set-point register value (datasheet float formula).
    fn bme_heater_code(&self, target_c: i32, ambient_c: i32) -> u8 {
        let c = &self.bme_cal;
        let var1 = c.g1 as f32 / 16.0 + 49.0;
        let var2 = c.g2 as f32 / 32768.0 * 0.0005 + 0.00235;
        let var3 = c.g3 as f32 / 1024.0;
        let var4 = var1 * (1.0 + var2 * target_c as f32);
        let var5 = var4 + var3 * ambient_c as f32;
        let code = 3.4
            * (var5 * (4.0 / (4.0 + c.res_heat_range as f32))
                * (1.0 / (1.0 + c.res_heat_val as f32 * 0.002))
                - 25.0);
        code as u8
    }

    fn bme_read(&mut self, addr: u8) -> Result<BmeReading, ()> {
        // Forced mode: osrs_h x1, osrs_t x2, osrs_p x4.
        self.i2c.write(addr, &[0x72, 0x01]).map_err(|_| ())?;
        self.i2c.write(addr, &[0x74, 0x51]).map_err(|_| ())?;

        // TPH takes a few ms; the gas heater adds ~150 ms.
        let start = millis();
        loop {
            let status = self.reg_read(addr, 0x1D).map_err(|_| ())?;
            if status & 0x80 != 0 {
                break;
            }
            if millis().wrapping_sub(start) > 300 {
                return Err(());
            }
            delay_ms(5);
        }

        let mut f = [0u8; 15];
        self.i2c.write_read(addr, &[0x1D], &mut f).map_err(|_| ())?;
        let press_adc = ((f[2] as u32) << 12) | ((f[3] as u32) << 4) | (f[4] as u32 >> 4);
        let temp_adc = ((f[5] as u32) << 12) | ((f[6] as u32) << 4) | (f[7] as u32 >> 4);
        let hum_adc = u16::from_be_bytes([f[8], f[9]]) as u32;
        let gas_adc = ((f[13] as u32) << 2) | (f[14] as u32 >> 6);
        let gas_range = (f[14] & 0x0F) as usize;
        let gas_valid = f[14] & 0x20 != 0;

        let c = &self.bme_cal;

        // Temperature (datasheet float compensation).
        let var1 = (temp_adc as f32 / 16384.0 - c.t1 as f32 / 1024.0) * c.t2 as f32;
        let var2 = {
            let v = temp_adc as f32 / 131072.0 - c.t1 as f32 / 8192.0;
            v * v * c.t3 as f32 * 16.0
        };
        let t_fine = var1 + var2;
        let temp_c = t_fine / 5120.0;

        // Pressure.
        let mut var1 = t_fine / 2.0 - 64000.0;
        let mut var2 = var1 * var1 * (c.p6 as f32 / 131072.0);
        var2 += var1 * c.p5 as f32 * 2.0;
        var2 = var2 / 4.0 + c.p4 as f32 * 65536.0;
        var1 = (c.p3 as f32 * var1 * var1 / 16384.0 + c.p2 as f32 * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * c.p1 as f32;
        let press_pa = if var1 != 0.0 {
            let mut p = 1048576.0 - press_adc as f32;
            p = (p - var2 / 4096.0) * 6250.0 / var1;
            let v1 = c.p9 as f32 * p * p / 2147483648.0;
            let v2 = p * (c.p8 as f32 / 32768.0);
            let pq = p / 256.0;
            let v3 = pq * pq * pq * (c.p10 as f32 / 131072.0);
            p + (v1 + v2 + v3 + c.p7 as f32 * 128.0) / 16.0
        } else {
            0.0
        };

        // Humidity.
        let var1 = hum_adc as f32 - (c.h1 as f32 * 16.0 + c.h3 as f32 / 2.0 * temp_c);
        let var2 = var1
            * (c.h2 as f32 / 262144.0
                * (1.0
                    + c.h4 as f32 / 16384.0 * temp_c
                    + c.h5 as f32 / 1048576.0 * temp_c * temp_c));
        let var3 = c.h6 as f32 / 16384.0;
        let var4 = c.h7 as f32 / 2097152.0;
        let hum = (var2 + (var3 + var4 * temp_c) * var2 * var2).clamp(0.0, 100.0);

        // Gas resistance.
        let gas_ohm = if gas_valid {
            const K1: [f32; 16] = [
                0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -0.8, 0.0, 0.0, -0.2, -0.5, 0.0, -1.0, 0.0,
                0.0,
            ];
            const K2: [f32; 16] = [
                0.0, 0.0, 0.0, 0.0, 0.1, 0.7, 0.0, -0.8, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            ];
            let var1 = 1340.0 + 5.0 * c.range_sw_err as f32;
            let var2 = var1 * (1.0 + K1[gas_range] / 100.0);
            let var3 = 1.0 + K2[gas_range] / 100.0;
            let res = 1.0
                / (var3
                    * 0.000000125
                    * (1u32 << gas_range) as f32
                    * ((gas_adc as f32 - 512.0) / var2 + 1.0));
            res as u32
        } else {
            0
        };

        Ok(BmeReading {
            temp_c_x100: (temp_c * 100.0) as i32,
            press_pa: press_pa as u32,
            hum_pct_x100: (hum * 100.0) as u32,
            gas_ohm,
        })
    }
}

/// Sensirion CRC-8: poly 0x31, init 0xFF.
fn scd41_crc(data: &[u8]) -> u8 {
    let mut crc: u8 = 0xFF;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}
