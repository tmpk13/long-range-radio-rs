//! I2C2 (PB15 SCL, PA15 SDA) with a deadline on every wait, and a bus clear.
//!
//! stm32wlxx-hal 0.6's I2C driver waits for each bus event in a loop with no
//! way out (`busy_wait!` in its `i2c.rs`): a bus held low, or a sensor that
//! stretches the clock forever, holds the main loop until the watchdog
//! resets the chip (notes/sensors.md).  [`BoundedI2c`] keeps the HAL's setup
//! (clock, pins, timing) and runs the same transfer sequences as the HAL's
//! `read`, `write` and `write_read`, giving each wait [`WAIT_MS`].  A
//! timeout resets the peripheral and returns `Error::Bus`, so a failing bus
//! costs the loop milliseconds, not the node.
//!
//! Resetting the peripheral does not free a sensor that was driving SDA low
//! when the transfer stopped: it keeps driving it until it has clocked the
//! rest of its byte out, and every later transfer then ends at once with a
//! lost arbitration or a bus left busy.  [`BoundedI2c::clear_bus`] gives it
//! those clocks; it runs at boot and after every transfer that failed on the
//! bus.

use stm32wlxx_hal::embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use stm32wlxx_hal::gpio::pins;
use stm32wlxx_hal::i2c::{Error, I2c2};
use stm32wlxx_hal::pac;

use crate::platform::{SYSCLK_HZ, millis};

/// The bus's pins.
pub type Pins = (pins::B15, pins::A15);

/// Longest wait for one bus event.  A byte takes 90 us at 100 kHz, and no
/// sensor on this bus stretches the clock for more than a few milliseconds.
pub const WAIT_MS: u32 = 25;

/// I2C2 whose waits give up after [`WAIT_MS`].
pub struct BoundedI2c {
    base: pac::I2C2,
    _pins: Pins,
}

impl BoundedI2c {
    /// Take over a configured and enabled HAL I2C2.
    pub fn new(i2c: I2c2<Pins>) -> Self {
        let (base, pins) = i2c.free();
        Self { base, _pins: pins }
    }

    /// Free a bus that a sensor holds, then hand the pins back to the I2C
    /// peripheral.  A sensor interrupted while it sends a byte keeps SDA low
    /// until it has clocked the rest out, and neither the MCU's reset nor
    /// the peripheral's clocks SCL.  With the peripheral off, up to nine SCL
    /// clocks with SDA released let it finish, then a STOP returns the bus
    /// to idle (the I2C specification's bus clear, NXP UM10204 3.1.16).
    pub fn clear_bus(&mut self) {
        const PIN: u32 = 15;
        const MASK: u32 = 0b11 << (2 * PIN);
        const INPUT: u32 = 0b00 << (2 * PIN);
        const OUTPUT: u32 = 0b01 << (2 * PIN);
        const ALTERNATE: u32 = 0b10 << (2 * PIN);
        // SAFETY: PB15 and PA15 belong to this driver (`_pins`).  Only their
        // mode bits and their BSRR bits are written, the mode bits inside a
        // critical section because the other pins share MODER.  I2c2::new
        // left both open-drain with pull-ups and selected their alternate
        // function, which mode ALTERNATE restores.
        let (gpiob, gpioa) = unsafe { (&*pac::GPIOB::PTR, &*pac::GPIOA::PTR) };
        let modes = |scl: u32, sda: u32| {
            cortex_m::interrupt::free(|_| {
                gpiob
                    .moder
                    .modify(|r, w| unsafe { w.bits((r.bits() & !MASK) | scl) });
                gpioa
                    .moder
                    .modify(|r, w| unsafe { w.bits((r.bits() & !MASK) | sda) });
            })
        };
        let level = |high: bool| if high { 1 << PIN } else { 1 << (PIN + 16) };
        let scl = |high: bool| gpiob.bsrr.write(|w| unsafe { w.bits(level(high)) });
        let sda = |high: bool| gpioa.bsrr.write(|w| unsafe { w.bits(level(high)) });
        let sda_high = || gpioa.idr.read().bits() & (1 << PIN) != 0;
        // Half a 100 kHz clock period.
        let half = || cortex_m::asm::delay(SYSCLK_HZ / 200_000);

        // With PE clear the peripheral releases both lines.
        self.base.cr1.modify(|_, w| w.pe().clear_bit());
        // Released (high, open drain) before the pins turn into outputs.
        scl(true);
        sda(true);
        modes(OUTPUT, INPUT);
        half();
        for _ in 0..9 {
            if sda_high() {
                break;
            }
            scl(false);
            half();
            scl(true);
            half();
        }
        // STOP: SDA rises while SCL is high.
        modes(OUTPUT, OUTPUT);
        scl(false);
        half();
        sda(false);
        half();
        scl(true);
        half();
        sda(true);
        half();
        modes(ALTERNATE, ALTERNATE);
        self.base.cr1.modify(|_, w| w.pe().set_bit());
    }

    /// Clear the bus after a transfer that failed on it: a timeout, a bus
    /// left busy, or a lost arbitration (a NACK is only an absent sensor).
    fn settle(&mut self, result: Result<(), Error>) -> Result<(), Error> {
        if matches!(
            result,
            Err(Error::Bus) | Err(Error::Busy) | Err(Error::Arbitration)
        ) {
            self.clear_bus();
        }
        result
    }

    /// Wait until `done` holds for ISR, a NACK, a lost arbitration, or the
    /// deadline.
    fn wait(&self, done: impl Fn(&pac::i2c1::isr::R) -> bool) -> Result<(), Error> {
        let start = millis();
        loop {
            let isr = self.base.isr.read();
            if isr.arlo().is_lost() {
                self.base.icr.write(|w| w.arlocf().clear());
                return Err(Error::Arbitration);
            }
            if isr.nackf().is_nack() {
                // The peripheral sends the STOP itself after a NACK.
                while self.base.isr.read().stopf().is_no_stop() {
                    if millis().wrapping_sub(start) > WAIT_MS {
                        self.recover();
                        return Err(Error::Nack);
                    }
                }
                self.base.icr.write(|w| w.nackcf().clear().stopcf().clear());
                return Err(Error::Nack);
            }
            if done(&isr) {
                return Ok(());
            }
            if millis().wrapping_sub(start) > WAIT_MS {
                self.recover();
                return Err(Error::Bus);
            }
        }
    }

    /// Software reset: clearing PE releases SCL and SDA and returns the state
    /// machine and the status flags to reset values; PE has to stay clear
    /// for three APB clocks (RM0461, I2C software reset).
    fn recover(&self) {
        self.base.cr1.modify(|_, w| w.pe().clear_bit());
        for _ in 0..3 {
            let _ = self.base.cr1.read();
        }
        self.base.cr1.modify(|_, w| w.pe().set_bit());
    }

    fn start(&self, addr: u8, read: bool, first: bool, len: usize, last: bool, autoend: bool) {
        self.base.cr2.modify(|_, w| {
            if first {
                w.add10().bit7();
                w.sadd().bits((addr << 1) as u16);
                if read {
                    w.rd_wrn().read();
                } else {
                    w.rd_wrn().write();
                }
                w.start().start();
            }
            w.nbytes().bits(len as u8);
            if !last {
                w.reload().not_completed()
            } else if autoend {
                w.reload().completed().autoend().automatic()
            } else {
                w.reload().completed().autoend().software()
            }
        });
    }

    fn receive(&self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        let chunks = buffer.len().div_ceil(0xFF);
        for (i, chunk) in buffer.chunks_mut(0xFF).enumerate() {
            self.start(addr, true, i == 0, chunk.len(), i + 1 == chunks, true);
            for byte in chunk.iter_mut() {
                self.wait(|isr| isr.rxne().is_not_empty())?;
                *byte = self.base.rxdr.read().rxdata().bits();
            }
            if i + 1 != chunks {
                self.wait(|isr| isr.tcr().is_complete())?;
            }
        }
        Ok(())
    }

    fn transmit(&self, addr: u8, bytes: &[u8], autoend: bool) -> Result<(), Error> {
        let chunks = bytes.len().div_ceil(0xFF);
        for (i, chunk) in bytes.chunks(0xFF).enumerate() {
            self.start(addr, false, i == 0, chunk.len(), i + 1 == chunks, autoend);
            for &byte in chunk {
                self.wait(|isr| isr.txis().is_empty())?;
                self.base.txdr.write(|w| w.txdata().bits(byte));
            }
            if i + 1 != chunks {
                self.wait(|isr| isr.tcr().is_complete())?;
            }
        }
        Ok(())
    }

    fn stop(&self) -> Result<(), Error> {
        self.wait(|isr| isr.stopf().is_stop())?;
        self.base.icr.write(|w| w.stopcf().clear());
        Ok(())
    }

    fn idle(&self) -> Result<(), Error> {
        if self.base.isr.read().busy().is_busy() {
            Err(Error::Busy)
        } else {
            Ok(())
        }
    }

    fn read_bytes(&self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        self.idle()?;
        self.receive(addr, buffer)?;
        self.stop()
    }

    fn write_bytes(&self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        self.idle()?;
        if bytes.is_empty() {
            // Address only.
            self.start(addr, false, true, 0, true, true);
        } else {
            self.transmit(addr, bytes, true)?;
        }
        self.stop()
    }

    fn write_read_bytes(&self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        self.idle()?;
        self.transmit(addr, bytes, false)?;
        self.wait(|isr| isr.tc().is_complete())?;
        // Repeated START for the read.
        self.receive(addr, buffer)?;
        self.stop()
    }
}

impl Read for BoundedI2c {
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        if buffer.is_empty() {
            return Ok(());
        }
        let result = self.read_bytes(addr, buffer);
        self.settle(result)
    }
}

impl Write for BoundedI2c {
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        let result = self.write_bytes(addr, bytes);
        self.settle(result)
    }
}

impl WriteRead for BoundedI2c {
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        if bytes.is_empty() {
            return self.read(addr, buffer);
        }
        if buffer.is_empty() {
            return self.write(addr, bytes);
        }
        let result = self.write_read_bytes(addr, bytes, buffer);
        self.settle(result)
    }
}
