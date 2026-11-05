//! RS-485 field bus link (`rs422` cargo feature).
//!
//! A TPT482 transceiver (U6) on the Forest Sensor Platform, wired to
//! USART1: PB6 = TX into the driver input D, PB7 = RX from the receiver
//! output R, PA9 = DE (driver enable, active high, R36 1M pulldown) and
//! PB9 = /RE (receiver enable, active low, R37 1M pulldown).  The
//! schematic's names for the two enable nets are swapped (DE sits on
//! `Receiver_Output_Enable`); the pins above are what the nets reach.
//!
//! R42/R43 strap the full-duplex part into two-wire half duplex (A to Y,
//! B to Z), so the driver is enabled only while a request is on the wire,
//! and the receiver is switched off meanwhile so the request does not
//! echo back into RX.  The pulldowns give the right idle state from
//! reset: driver off, receiver on.
//!
//! J7 carries the bus plus a sensor supply from the switched 5 V rail, so
//! a probe answers only while [`crate::board::Rail5v`] is on and has had
//! [`SOIL_WARMUP_MS`] to start.
//!
//! Includes a minimal Modbus RTU master, enough to poll the DFRobot
//! SEN0600 soil temperature/moisture probe (9600 8N1, slave 0x01,
//! moisture at holding register 0x0000 and temperature at 0x0001, both
//! scaled by 10).

use crate::platform::millis;
use stm32wlxx_hal::{
    embedded_hal::serial::{Read, Write},
    gpio::{pins, Output},
    pac,
    uart::{self, Uart1},
};

/// Line rate - the SEN0600 factory default.
pub const BAUD: u32 = 9_600;

/// Modbus slave address of the soil probe (factory default).
pub const SOIL_ADDR: u8 = 0x01;

/// How long the 5 V rail is on before the probe is polled.  A margin
/// picked without a measured start-up time for the SEN0600; shorten it
/// once a bench run shows how soon the probe answers.
pub const SOIL_WARMUP_MS: u32 = 2_000;

/// Longest Modbus response we accept (read of 16 registers).
const MAX_FRAME: usize = 5 + 2 * 16 + 2;

/// Longest wait for the last request byte to leave the shift register.
/// A full 8-byte FIFO at 9600 baud drains in ~8 ms.
const TX_DONE_TIMEOUT_MS: u32 = 50;

#[derive(Debug, Clone, Copy)]
pub enum Rs422Error {
    /// UART framing/noise/overrun/parity error.
    Uart,
    /// No (or truncated) response before the deadline.
    Timeout,
    /// Response failed the CRC check.
    Crc,
    /// Response was malformed (wrong address, length or function).
    Malformed,
    /// Modbus exception response; the exception code is included.
    Exception(u8),
}

/// Modbus CRC-16 (poly 0xA001, init 0xFFFF), transmitted low byte first.
fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= byte as u16;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

/// One reading from the SEN0600 soil probe.
#[derive(Debug, Clone, Copy)]
pub struct SoilReading {
    /// Volumetric moisture in tenths of a percent (486 = 48.6 %).
    pub moisture_pct_x10: u16,
    /// Soil temperature in tenths of a degree C (-97 = -9.7 C).
    pub temp_c_x10: i16,
}

/// RS-485 link on USART1 with a Modbus RTU master on top.
pub struct Rs422 {
    uart: Uart1<pins::B7, pins::B6>,
    de: Output<pins::A9>,
    re_n: Output<pins::B9>,
}

impl Rs422 {
    pub fn new(
        usart1: pac::USART1,
        b6: pins::B6,
        b7: pins::B7,
        a9: pins::A9,
        b9: pins::B9,
        rcc: &mut pac::RCC,
        cs: &cortex_m::interrupt::CriticalSection,
    ) -> Self {
        let uart = Uart1::new(usart1, BAUD, uart::Clk::PClk, rcc)
            .enable_rx(b7, cs)
            .enable_tx(b6, cs);
        // Low on both: driver off, receiver on, matching the pulldowns.
        Self {
            uart,
            de: Output::default(a9, cs),
            re_n: Output::default(b9, cs),
        }
    }

    /// The USART1 register block, for the flags the HAL does not expose.
    /// USART1 is owned by `self.uart`, so access through it is exclusive.
    fn regs(&self) -> &pac::usart1::RegisterBlock {
        unsafe { &*pac::USART1::PTR }
    }

    /// Clear sticky UART error flags (overrun keeps erroring until
    /// acknowledged in ICR).
    fn clear_errors(&mut self) {
        self.regs().icr.write(|w| {
            w.orecf()
                .set_bit()
                .fecf()
                .set_bit()
                .pecf()
                .set_bit()
                .ncf()
                .set_bit()
        });
    }

    /// Blocking write of the whole buffer, driving the bus only while it
    /// is on the wire.
    pub fn write_all(&mut self, buf: &[u8]) -> Result<(), Rs422Error> {
        // Receiver off before the driver comes on, so the request does
        // not loop back into RX.
        self.re_n.set_level_high();
        self.de.set_level_high();
        let result = self.send(buf);
        self.de.set_level_low();
        self.re_n.set_level_low();
        result
    }

    /// Queue the bytes, then wait until the last stop bit has left.
    ///
    /// The HAL's `flush` cannot be used for this: it waits on ISR.BUSY,
    /// which tracks the receiver.  Dropping DE on it would cut the last
    /// byte off the bus.  ISR.TC is the transmitter's own completion flag,
    /// and writing TDR clears it, so it cannot be left over from before.
    fn send(&mut self, buf: &[u8]) -> Result<(), Rs422Error> {
        for &byte in buf {
            nb::block!(self.uart.write(byte)).map_err(|_| Rs422Error::Uart)?;
        }
        let start = millis();
        while self.regs().isr.read().tc().bit_is_clear() {
            if millis().wrapping_sub(start) > TX_DONE_TIMEOUT_MS {
                return Err(Rs422Error::Timeout);
            }
        }
        Ok(())
    }

    /// Non-blocking single byte read; `None` when the FIFO is empty.
    pub fn read_byte(&mut self) -> Result<Option<u8>, Rs422Error> {
        match self.uart.read() {
            Ok(byte) => Ok(Some(byte)),
            Err(nb::Error::WouldBlock) => Ok(None),
            Err(nb::Error::Other(_)) => {
                self.clear_errors();
                Err(Rs422Error::Uart)
            }
        }
    }

    /// Discard anything left in the receiver.
    fn drain(&mut self) {
        while matches!(self.read_byte(), Ok(Some(_))) {}
    }

    /// Modbus function 0x03: read `out.len()` holding registers starting
    /// at `start_reg` from `slave`.
    pub fn modbus_read_holding(
        &mut self,
        slave: u8,
        start_reg: u16,
        out: &mut [u16],
        timeout_ms: u32,
    ) -> Result<(), Rs422Error> {
        let count = out.len() as u16;
        debug_assert!(out.len() <= 16);
        self.clear_errors();
        self.drain();

        let mut req = [0u8; 8];
        req[0] = slave;
        req[1] = 0x03;
        req[2..4].copy_from_slice(&start_reg.to_be_bytes());
        req[4..6].copy_from_slice(&count.to_be_bytes());
        let crc = crc16(&req[..6]);
        req[6..8].copy_from_slice(&crc.to_le_bytes());
        self.write_all(&req)?;

        // Collect the response: addr, func, byte count, data, CRC16.
        let expected = 5 + 2 * out.len();
        let mut resp = [0u8; MAX_FRAME];
        let mut len = 0usize;
        let start = millis();
        while len < expected {
            match self.read_byte()? {
                Some(byte) => {
                    resp[len] = byte;
                    len += 1;
                    // An exception response is only 5 bytes total.
                    if len == 5 && resp[1] == 0x83 {
                        break;
                    }
                }
                None => {
                    if millis().wrapping_sub(start) > timeout_ms {
                        return Err(Rs422Error::Timeout);
                    }
                }
            }
        }

        let crc_got = u16::from_le_bytes([resp[len - 2], resp[len - 1]]);
        if crc16(&resp[..len - 2]) != crc_got {
            return Err(Rs422Error::Crc);
        }
        if resp[0] != slave {
            return Err(Rs422Error::Malformed);
        }
        if resp[1] == 0x83 {
            return Err(Rs422Error::Exception(resp[2]));
        }
        if resp[1] != 0x03 || resp[2] as usize != 2 * out.len() {
            return Err(Rs422Error::Malformed);
        }
        for (i, word) in out.iter_mut().enumerate() {
            *word = u16::from_be_bytes([resp[3 + 2 * i], resp[4 + 2 * i]]);
        }
        Ok(())
    }

    /// Poll the SEN0600 soil probe.
    ///
    /// The probe answers a 9-byte response within a few character times;
    /// 200 ms is generous even with a sleepy sensor.
    pub fn read_soil(&mut self) -> Result<SoilReading, Rs422Error> {
        let mut regs = [0u16; 2];
        self.modbus_read_holding(SOIL_ADDR, 0x0000, &mut regs, 200)?;
        Ok(SoilReading {
            moisture_pct_x10: regs[0],
            temp_c_x10: regs[1] as i16,
        })
    }
}
