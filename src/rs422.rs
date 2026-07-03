//! RS-422/RS-485 field bus link (`rs422` cargo feature).
//!
//! Ground work for a MAX3430-class differential transceiver on USART2:
//! PA2 = TX and PA3 = RX (nets UART-2-TX / UART-2-RX, broken out on the
//! charger board's "UART 2" header).  Full-duplex RS-422 point-to-point
//! is assumed, with the transceiver's driver enable strapped on, so no
//! DE/RE GPIO is needed.  For half-duplex RS-485 multi-drop a direction
//! pin would have to be added around [`Rs422::write_all`].
//!
//! Includes a minimal Modbus RTU master, enough to poll the DFRobot
//! SEN0600 soil temperature/moisture probe (9600 8N1, slave 0x01,
//! moisture at holding register 0x0000 and temperature at 0x0001, both
//! scaled by 10).

use crate::platform::millis;
use stm32wlxx_hal::{
    embedded_hal::serial::{Read, Write},
    gpio::pins,
    pac,
    uart::{self, Uart2},
};

/// Line rate — the SEN0600 factory default.
pub const BAUD: u32 = 9_600;

/// Modbus slave address of the soil probe (factory default).
pub const SOIL_ADDR: u8 = 0x01;

/// Longest Modbus response we accept (read of 16 registers).
const MAX_FRAME: usize = 5 + 2 * 16 + 2;

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

/// RS-422 link on USART2 with a Modbus RTU master on top.
pub struct Rs422 {
    uart: Uart2<pins::A3, pins::A2>,
}

impl Rs422 {
    pub fn new(
        usart2: pac::USART2,
        a2: pins::A2,
        a3: pins::A3,
        rcc: &mut pac::RCC,
        cs: &cortex_m::interrupt::CriticalSection,
    ) -> Self {
        let uart = Uart2::new(usart2, BAUD, uart::Clk::PClk, rcc)
            .enable_rx(a3, cs)
            .enable_tx(a2, cs);
        Self { uart }
    }

    /// Clear sticky UART error flags (overrun keeps erroring until
    /// acknowledged in ICR).
    fn clear_errors(&mut self) {
        // The HAL keeps the register block private; USART2 is owned by
        // `self.uart` so this access is exclusive.
        unsafe {
            let usart2 = &*pac::USART2::PTR;
            usart2.icr.write(|w| {
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
    }

    /// Blocking write of the whole buffer.
    pub fn write_all(&mut self, buf: &[u8]) -> Result<(), Rs422Error> {
        for &byte in buf {
            nb::block!(self.uart.write(byte)).map_err(|_| Rs422Error::Uart)?;
        }
        nb::block!(self.uart.flush()).map_err(|_| Rs422Error::Uart)?;
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
