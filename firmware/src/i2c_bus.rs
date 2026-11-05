//! I2C2 (PB15 SCL, PA15 SDA) with a deadline on every wait, and a bus clear
//! for boot.
//!
//! stm32wlxx-hal 0.6's I2C driver waits for each bus event in a loop with no
//! way out (`busy_wait!` in its `i2c.rs`): a bus held low, or a sensor that
//! stretches the clock forever, holds the main loop until the watchdog
//! resets the chip (notes/sensors.md).  [`BoundedI2c`] keeps the HAL's setup
//! (clock, pins, timing) and runs the same transfer sequences as the HAL's
//! `read`, `write` and `write_read`, giving each wait [`WAIT_MS`].  On a
//! timeout it resets the peripheral and returns `Error::Bus`, so a failing
//! bus costs the loop milliseconds, not the node.

use cortex_m::interrupt::CriticalSection;
use stm32wlxx_hal::embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use stm32wlxx_hal::gpio::{Input, Output, OutputArgs, OutputType, PinState, Pull, Speed, pins};
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
}

impl Read for BoundedI2c {
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        if buffer.is_empty() {
            return Ok(());
        }
        self.idle()?;
        self.receive(addr, buffer)?;
        self.stop()
    }
}

impl Write for BoundedI2c {
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        self.idle()?;
        if bytes.is_empty() {
            // Address only.
            self.start(addr, false, true, 0, true, true);
        } else {
            self.transmit(addr, bytes, true)?;
        }
        self.stop()
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
        self.idle()?;
        self.transmit(addr, bytes, false)?;
        self.wait(|isr| isr.tc().is_complete())?;
        // Repeated START for the read.
        self.receive(addr, buffer)?;
        self.stop()
    }
}

/// Free a bus that a sensor holds.  A sensor interrupted while it sends a
/// byte keeps driving SDA low until it has clocked the rest out, and the
/// MCU's own reset does not clock SCL, so the bus stays stuck across a
/// watchdog reset.  Up to nine SCL clocks with SDA released let it finish,
/// then a STOP returns the bus to idle (the I2C specification's bus clear,
/// NXP UM10204 3.1.16).  Call before `I2c2::new`, which takes the pins back
/// into the I2C function.
pub fn bus_clear(scl: pins::B15, sda: pins::A15, cs: &CriticalSection) -> (pins::B15, pins::A15) {
    let open_drain = |level| OutputArgs {
        speed: Speed::Low,
        level,
        ot: OutputType::OpenDrain,
        pull: Pull::Up,
    };
    // Half a 100 kHz clock period.
    let half = || cortex_m::asm::delay(SYSCLK_HZ / 200_000);

    let mut scl = Output::new(scl, &open_drain(PinState::High), cs);
    let sda_in = Input::new(sda, Pull::Up, cs);
    half();
    for _ in 0..9 {
        if sda_in.level() == PinState::High {
            break;
        }
        scl.set_level_low();
        half();
        scl.set_level_high();
        half();
    }
    // STOP: SDA rises while SCL is high.
    let mut sda = Output::new(sda_in.free(), &open_drain(PinState::High), cs);
    scl.set_level_low();
    half();
    sda.set_level_low();
    half();
    scl.set_level_high();
    half();
    sda.set_level_high();
    half();
    (scl.free(), sda.free())
}
