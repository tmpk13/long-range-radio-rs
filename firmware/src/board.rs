//! Support for the Forest Sensor Platform V1 board (`board` cargo feature).
//!
//! Pin assignments (from the `Forest-Sensor-Platform` KiCad project, V1):
//!
//! | Pin  | Peripheral | Net         | Function                                    |
//! |------|------------|-------------|---------------------------------------------|
//! | PB13 | ADC_IN0    | VSENSE_VIN  | Solar panel input, 100k:10k divider         |
//! | PA10 | ADC_IN6    | VSENSE_OUT  | Battery (Battery_Out), 10k:10k divider      |
//! | PB14 | ADC_IN1    | BAT_ISENSE  | INA181A1 across the 50 mOhm battery shunt   |
//! | PB10 | TIM2_CH3   | PWM_Buck    | Buck switch drive, high = switch on         |
//! | PB3  | SPI1_SCK   | SPI-SCK     | SD card CLK                                 |
//! | PB4  | SPI1_MISO  | SPI-CITO    | SD card DAT0                                |
//! | PB5  | SPI1_MOSI  | SPI-COTI    | SD card CMD                                 |
//! | PA0  | GPIO out   | SPI-CS      | SD card chip select (CD/DAT3)               |
//! | PC0  | GPIO out   | 5V_EN       | Switched 5 V rail enable                    |
//! | PC1  | GPIO out   | WIO_USR_LED | D12 radio activity LED (active low)         |
//!
//! Each analog input sits behind a 1k / 100 nF RC filter (R38-R40 with
//! C16, C19, C21).  The buck is non-synchronous: a high-side PMOS (Q2)
//! with an SK56A freewheel diode, so only the one PWM output is driven.
//!
//! The rest of the module's pins belong to other parts of the board:
//! USART1 plus PA9/PB9 drive the RS-485 transceiver (`rs422` feature),
//! USART2 (PA2/PA3) runs to the CH572D USB bridge, and SWD plus RST are
//! shared between the CH572D and the J2 debug header.

use crate::platform::SYSCLK_HZ;
use cortex_m::interrupt::CriticalSection;
use stm32wlxx_hal::{
    adc::{self, Adc, OversampleRatio, OversampleShift},
    embedded_hal::blocking::delay::DelayUs,
    gpio::{pins, Analog, Output, OutputArgs, PinState},
    pac,
    spi::{BaudRate, Spi, Transfer, Write, MODE_0},
};

/// Charger tuning constants.
pub mod charge {
    /// Battery voltage ceiling (mV): 3S NiMH at ~1.4 V/cell, matching the
    /// board's "3V6-4V2" output rating.  Above this the charger limits.
    pub const VBAT_MAX_MV: u32 = 4200;
    /// Charge current ceiling (mA): the buck's 1 A design point, where the
    /// 33 uH inductor still has 1.6x margin below saturation.
    pub const IBAT_MAX_MA: u32 = 1000;
    /// The non-synchronous buck needs the input above the battery by at
    /// least this much (dropout + SK56A diode) before charging is useful.
    pub const VIN_MARGIN_MV: u32 = 1000;
    /// Extra input headroom required before leaving [`ChargeState::Idle`],
    /// so a marginal panel does not flap between states.
    pub const VIN_HYST_MV: u32 = 500;
    /// Perturb & observe period (ms).
    pub const MPPT_PERIOD_MS: u32 = 200;
    /// P&O duty perturbation (permille): about two TIM2 counts.  The
    /// buck reaches 1 A near 150 permille from a 24 V panel, so one step
    /// moves the charge current by roughly 0.2 A at that point.
    pub const DUTY_STEP_PM: u32 = 13;
    /// Backoff step while a voltage/current limit is active (permille).
    pub const LIMIT_STEP_PM: u32 = 26;
    /// Power changes below this (mW) are treated as noise and do not
    /// reverse the perturb direction.
    pub const POWER_HYST_MW: u32 = 40;
    /// ADC sweeps averaged per MPPT step.  The current amplifier resolves
    /// ~0.8 mA per LSB, so this mostly averages out the switching ripple
    /// that gets past the 1k / 100 nF input filters.
    pub const AVG_SAMPLES: u32 = 4;
}

/// Busy-wait delay from CPU cycles (SysTick is owned by the RTIC monotonic).
struct CycleDelay;

impl DelayUs<u8> for CycleDelay {
    fn delay_us(&mut self, us: u8) {
        cortex_m::asm::delay(us as u32 * (SYSCLK_HZ / 1_000_000));
    }
}

/// Raise the MSI clock from the 4 MHz reset default to 16 MHz.
///
/// Must be called before the SysTick monotonic is started and before any
/// clock-derived peripheral setup (I2C timing, SubGHz SPI baud rate).
/// 16 MHz keeps zero flash wait states while giving TIM2 160 duty steps
/// at the 100 kHz buck switching frequency.
pub fn raise_sysclk(rcc: &mut pac::RCC) {
    rcc.cr
        .modify(|_, w| w.msirgsel().set_bit().msirange().range16m());
    while rcc.cr.read().msirdy().bit_is_clear() {}
}

/// One sensor sweep, in physical units.
#[derive(Debug, Clone, Copy, Default)]
pub struct Telemetry {
    /// Solar input voltage (mV).
    pub vin_mv: u32,
    /// Battery voltage (mV).
    pub vbat_mv: u32,
    /// Battery charge current (mA).  The amplifier's reference is
    /// grounded, so it reads 0 while the battery is discharging.
    pub ibat_ma: u32,
}

/// Format `args` into `buf`, returning the written prefix (truncation-safe).
fn fmt_into<'a>(buf: &'a mut [u8; 32], args: core::fmt::Arguments) -> &'a str {
    struct W<'b> {
        buf: &'b mut [u8],
        len: usize,
    }
    impl core::fmt::Write for W<'_> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let bytes = s.as_bytes();
            if self.len + bytes.len() > self.buf.len() {
                return Err(core::fmt::Error);
            }
            self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
            self.len += bytes.len();
            Ok(())
        }
    }
    let mut w = W { buf, len: 0 };
    let _ = core::fmt::write(&mut w, args);
    let len = w.len;
    core::str::from_utf8(&buf[..len]).unwrap_or("")
}

impl Telemetry {
    /// Format as `V=<vin> B=<vbat> I=<ibat>` into `buf`, returning the
    /// written prefix.  Fits a 32-byte mesh payload.
    pub fn format_into<'a>(&self, buf: &'a mut [u8; 32]) -> &'a str {
        fmt_into(
            buf,
            format_args!("V={} B={} I={}", self.vin_mv, self.vbat_mv, self.ibat_ma),
        )
    }

    /// Like [`format_into`](Self::format_into) with the charger status
    /// appended: `... D=<duty permille> <state letter>`.
    pub fn format_status<'a>(
        &self,
        duty_pm: u32,
        state: ChargeState,
        buf: &'a mut [u8; 32],
    ) -> &'a str {
        let s = match state {
            ChargeState::Idle => 'i',
            ChargeState::Tracking => 't',
            ChargeState::Limiting => 'l',
        };
        fmt_into(
            buf,
            format_args!(
                "V={} B={} I={} D={} {}",
                self.vin_mv, self.vbat_mv, self.ibat_ma, duty_pm, s
            ),
        )
    }
}

/// ADC readings of the panel divider, battery divider and current amplifier.
///
/// The battery current comes from U1, an INA181A1 (gain 20 V/V) across
/// the 50 mOhm shunt R1 between Battery_Out and the pack, so the pin sees
/// 1 V per amp of charge current.
pub struct Senses {
    adc: Adc,
    vin: Analog<pins::B13>,
    vbat: Analog<pins::A10>,
    ibat: Analog<pins::B14>,
}

impl Senses {
    pub fn new(
        adc: pac::ADC,
        b13: pins::B13,
        a10: pins::A10,
        b14: pins::B14,
        rcc: &mut pac::RCC,
        cs: &CriticalSection,
    ) -> Self {
        let mut adc = Adc::new(adc, adc::Clk::PClkDiv4, rcc);
        adc.calibrate(&mut CycleDelay);
        // Up to ~10k of divider impedance needs the longest sample time.
        adc.set_max_sample_time();
        // 16x hardware oversampling (result stays 12-bit) knocks a couple
        // of bits of noise off every channel.
        adc.enable_oversampling(OversampleRatio::Mul16, OversampleShift::Shift4);
        adc.enable();
        adc.enable_vref();
        Self {
            adc,
            vin: Analog::new(b13, cs),
            vbat: Analog::new(a10, cs),
            ibat: Analog::new(b14, cs),
        }
    }

    /// Average `n` sweeps of all channels.
    pub fn read_avg(&mut self, n: u32) -> Telemetry {
        let n = n.max(1);
        let mut acc = Telemetry::default();
        for _ in 0..n {
            let t = self.read();
            acc.vin_mv += t.vin_mv;
            acc.vbat_mv += t.vbat_mv;
            acc.ibat_ma += t.ibat_ma;
        }
        Telemetry {
            vin_mv: acc.vin_mv / n,
            vbat_mv: acc.vbat_mv / n,
            ibat_ma: acc.ibat_ma / n,
        }
    }

    /// Read all channels and convert to physical units.
    pub fn read(&mut self) -> Telemetry {
        // Correct for the actual 3V3 rail using the factory-calibrated
        // internal reference (VREFINT_CAL is taken at VDDA = 3.3 V).
        let vref = self.adc.vref().max(1) as u32;
        let vdda_mv = 3300 * adc::vref_cal() as u32 / vref;
        let mv = |raw: u16| raw as u32 * vdda_mv / 4095;

        let vin_pin = mv(self.adc.pin(&self.vin));
        let vbat_pin = mv(self.adc.pin(&self.vbat));
        let ibat_pin = mv(self.adc.pin(&self.ibat));

        Telemetry {
            // 100k over 10k: Vin = Vpin * 110k / 10k.  Full scale is ~36 V.
            vin_mv: vin_pin * 11,
            // 10k over 10k
            vbat_mv: vbat_pin * 2,
            // 50 mOhm x 20 V/V = 1 V/A: 1 mV at the pin is 1 mA
            ibat_ma: ibat_pin,
        }
    }
}

/// Buck converter PWM output: TIM2 channel 3 on PB10 (net PWM_Buck).
///
/// PWM_Buck drives the gate of Q3 (BSS123), which pulls the bases of the
/// Q1 push-pull low; Q1 then pulls the gate of the Q2 high-side PMOS down
/// to the D1 zener rail 12 V below the panel, turning the switch on.  So
/// a high output means switch on.  R55 (100k) holds Q3 off, and the
/// switch open, from reset until the timer takes the pin.
pub struct Buck {
    tim2: pac::TIM2,
    _pin: pins::B10,
}

impl Buck {
    /// Switching frequency (matches the SPICE-validated design point).
    pub const PWM_HZ: u32 = 100_000;
    const ARR: u32 = SYSCLK_HZ / Self::PWM_HZ - 1;
    /// Duty ceiling.  The PMOS gate drive needs no bootstrap capacitor, so
    /// the switch may stay on for whole periods.
    pub const MAX_DUTY_PERMILLE: u32 = 1000;

    pub fn new(tim2: pac::TIM2, b10: pins::B10, rcc: &mut pac::RCC, _cs: &CriticalSection) -> Self {
        rcc.apb1enr1.modify(|_, w| w.tim2en().set_bit());
        rcc.apb1enr1.read();
        rcc.apb1rstr1.modify(|_, w| w.tim2rst().set_bit());
        rcc.apb1rstr1.modify(|_, w| w.tim2rst().clear_bit());

        // PB10 to TIM2_CH3 (AF1), high speed.  The HAL has no TIM support,
        // so configure the pin registers directly; exclusivity is
        // guaranteed by owning the pin token and the critical section.
        unsafe {
            let gpiob = &*pac::GPIOB::PTR;
            gpiob.ospeedr.modify(|_, w| w.ospeedr10().very_high_speed());
            gpiob.afrh.modify(|_, w| w.afrh10().af1());
            gpiob.moder.modify(|_, w| w.moder10().alternate());
        }

        tim2.psc.write(|w| w.psc().bits(0));
        tim2.arr.write(|w| w.arr().bits(Self::ARR));
        // PWM mode 1 with preload: output high while CNT < CCR3.  A CCR3
        // above ARR keeps the output high for the whole period.
        tim2.ccmr2_output()
            .modify(|_, w| w.oc3m().bits(0b110).oc3pe().set_bit());
        tim2.ccr3.write(|w| w.ccr3().bits(0));
        tim2.ccer.modify(|_, w| w.cc3e().set_bit());
        tim2.egr.write(|w| w.ug().set_bit());
        tim2.cr1.modify(|_, w| w.arpe().set_bit().cen().set_bit());

        Self { tim2, _pin: b10 }
    }

    /// Set the duty cycle in permille (0..=1000).  0 turns the switch off.
    pub fn set_duty_permille(&mut self, permille: u32) {
        let pm = permille.min(Self::MAX_DUTY_PERMILLE);
        let ccr = (Self::ARR + 1) * pm / 1000;
        self.tim2.ccr3.write(|w| w.ccr3().bits(ccr));
    }

    /// Current duty cycle in permille.
    pub fn duty_permille(&self) -> u32 {
        self.tim2.ccr3.read().ccr3().bits() * 1000 / (Self::ARR + 1)
    }

    /// Force the switch off (duty 0).
    pub fn off(&mut self) {
        self.set_duty_permille(0);
    }
}

/// Radio activity flags, set by the radio driver from wherever a packet
/// is actually sent or received, consumed by [`ActivityLed::update`].
pub mod activity {
    use core::sync::atomic::{AtomicBool, Ordering};

    static TX: AtomicBool = AtomicBool::new(false);
    static RX: AtomicBool = AtomicBool::new(false);

    pub fn note_tx() {
        TX.store(true, Ordering::Relaxed);
    }
    pub fn note_rx() {
        RX.store(true, Ordering::Relaxed);
    }
    pub(super) fn take_tx() -> bool {
        TX.swap(false, Ordering::Relaxed)
    }
    pub(super) fn take_rx() -> bool {
        RX.swap(false, Ordering::Relaxed)
    }
}

/// LED pulse length.  Long enough to see, short enough that back-to-back
/// packets read as flicker rather than a solid light.
const LED_PULSE_MS: u32 = 30;

/// Radio activity LED: D12 on PC1 (net WIO_USR_LED), with its cathode on
/// the pin and its anode through R30 (10k) to +3.3V, so driving the pin
/// low lights it.  The board has one LED for the radio, so it pulses on
/// transmit and receive alike.
pub struct ActivityLed {
    pin: Output<pins::C1>,
    on_at: u32,
    lit: bool,
}

impl ActivityLed {
    pub fn new(c1: pins::C1, cs: &CriticalSection) -> Self {
        const OFF: OutputArgs = OutputArgs {
            level: PinState::High,
            ..OutputArgs::new()
        };
        Self {
            pin: Output::new(c1, &OFF, cs),
            on_at: 0,
            lit: false,
        }
    }

    /// Start a pending pulse and retire an expired one.  Call every main
    /// loop iteration; an event while lit extends the pulse.
    pub fn update(&mut self, now_ms: u32) {
        // Non-short-circuit `|` so a TX and an RX in the same iteration
        // are both consumed.
        if activity::take_tx() | activity::take_rx() {
            self.pin.set_level_low();
            self.on_at = now_ms;
            self.lit = true;
        } else if self.lit && now_ms.wrapping_sub(self.on_at) >= LED_PULSE_MS {
            self.pin.set_level_high();
            self.lit = false;
        }
    }
}

/// The switched 5 V rail.  PC0 (net 5V_EN) turns on U13, a load switch
/// from the battery, and with it the U11 boost converter.  The rail feeds
/// the J7 field-bus sensor supply and the CH572D USB bridge.  With USB
/// plugged in, U2 supplies +5V from VBUS whatever this pin does.  R44
/// (1M) holds the rail off from reset until this takes the pin.
pub struct Rail5v {
    en: Output<pins::C0>,
}

impl Rail5v {
    pub fn new(c0: pins::C0, cs: &CriticalSection) -> Self {
        Self {
            en: Output::default(c0, cs),
        }
    }

    pub fn set(&mut self, on: bool) {
        if on {
            self.en.set_level_high();
        } else {
            self.en.set_level_low();
        }
    }

    pub fn is_on(&self) -> bool {
        self.en.level() == PinState::High
    }
}

/// Charger state, one letter in the telemetry broadcast.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChargeState {
    /// Input too low to charge; PWM off, waiting for sun.
    Idle,
    /// Perturb & observe hill climbing on output power.
    Tracking,
    /// Battery voltage or current ceiling reached; duty backing off.
    Limiting,
}

/// Perturb & observe MPPT with output voltage/current limiting.
///
/// The board senses the battery side of the buck, so the controller
/// climbs on *output* power (`vbat * ibat`).  Converter efficiency is
/// flat across the operating range, so the output maximum coincides
/// with the panel's maximum power point.
pub struct Mppt {
    state: ChargeState,
    duty_pm: u32,
    dir_up: bool,
    last_power_mw: u32,
}

impl Default for Mppt {
    fn default() -> Self {
        Self::new()
    }
}

impl Mppt {
    pub const fn new() -> Self {
        Self {
            state: ChargeState::Idle,
            duty_pm: 0,
            dir_up: true,
            last_power_mw: 0,
        }
    }

    pub fn state(&self) -> ChargeState {
        self.state
    }

    /// Commanded duty (permille).  Tracked here because the timer
    /// quantizes to ~6 permille and would round-trip lossily.
    pub fn duty_permille(&self) -> u32 {
        self.duty_pm
    }

    fn apply(&mut self, buck: &mut Buck, duty_pm: u32) {
        self.duty_pm = duty_pm.min(Buck::MAX_DUTY_PERMILLE);
        buck.set_duty_permille(self.duty_pm);
    }

    /// Run one controller step.  Call every [`charge::MPPT_PERIOD_MS`]
    /// with a fresh (averaged) telemetry reading.
    pub fn step(&mut self, t: &Telemetry, buck: &mut Buck) {
        // Not enough input to buck into the battery: park.  Hysteresis on
        // the way out so a marginal panel does not flap.
        let vin_floor = t.vbat_mv + charge::VIN_MARGIN_MV;
        if t.vin_mv < vin_floor {
            self.state = ChargeState::Idle;
            self.dir_up = true;
            self.last_power_mw = 0;
            self.apply(buck, 0);
            return;
        }
        if self.state == ChargeState::Idle {
            if t.vin_mv < vin_floor + charge::VIN_HYST_MV {
                return;
            }
            // Sun is back: soft-start from the bottom.
            self.state = ChargeState::Tracking;
            self.dir_up = true;
            self.last_power_mw = 0;
            self.apply(buck, charge::DUTY_STEP_PM);
            return;
        }

        // Battery protection overrides tracking.
        if t.vbat_mv > charge::VBAT_MAX_MV || t.ibat_ma > charge::IBAT_MAX_MA {
            self.state = ChargeState::Limiting;
            // Resume tracking downhill so the limit is not hit again
            // on the very next perturbation.
            self.dir_up = false;
            self.last_power_mw = 0;
            let duty = self.duty_pm.saturating_sub(charge::LIMIT_STEP_PM);
            self.apply(buck, duty);
            return;
        }

        // Perturb & observe: reverse direction when power dropped.
        self.state = ChargeState::Tracking;
        let power_mw = t.vbat_mv * t.ibat_ma / 1000;
        if power_mw + charge::POWER_HYST_MW < self.last_power_mw {
            self.dir_up = !self.dir_up;
        }
        self.last_power_mw = power_mw;

        let duty = if self.dir_up {
            self.duty_pm + charge::DUTY_STEP_PM
        } else {
            self.duty_pm.saturating_sub(charge::DUTY_STEP_PM)
        };
        // Bounce off the rails instead of sticking to them.
        if duty >= Buck::MAX_DUTY_PERMILLE {
            self.dir_up = false;
        }
        if duty == 0 {
            self.dir_up = true;
        }
        self.apply(buck, duty);
    }
}

/// SD card errors.
#[derive(Debug, Clone, Copy)]
pub enum SdError {
    /// Card did not respond in time.
    Timeout,
    /// Unexpected R1/token response (value included).
    Response(u8),
    /// SPI bus error.
    Spi,
    /// Card not initialized (call [`SdCard::init`] first).
    NotInitialized,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SdKind {
    /// SDSC v1, byte addressing.
    Sd1,
    /// SDSC v2, byte addressing.
    Sd2,
    /// SDHC/SDXC, block addressing.
    Sdhc,
}

type SdSpi = Spi<pac::SPI1, pins::B3, pins::B4, pins::B5>;

/// Minimal SD card driver in SPI mode: init plus single-block read/write.
///
/// SPI1 on PB3/PB4/PB5, chip select on PA0.  The socket's card detect
/// switch is not wired on this board, so a missing card shows up as an
/// init failure: R18 pulls DAT0 high, and CMD0 never sees its reply.
pub struct SdCard {
    spi: SdSpi,
    cs: Output<pins::A0>,
    kind: Option<SdKind>,
}

const SD_BLOCK_LEN: usize = 512;

impl SdCard {
    pub fn new(
        spi1: pac::SPI1,
        sck: pins::B3,
        miso: pins::B4,
        mosi: pins::B5,
        cs_pin: pins::A0,
        rcc: &mut pac::RCC,
        cs: &CriticalSection,
    ) -> Self {
        // Cards must be initialized below 400 kHz; Div64 gives 250 kHz
        // at the 16 MHz core clock.  init() raises it afterwards.
        let spi = Spi::new_spi1_full_duplex(
            spi1,
            (sck, miso, mosi),
            MODE_0,
            BaudRate::Div64,
            rcc,
            cs,
        );
        const CS_ARGS: OutputArgs = OutputArgs {
            level: PinState::High,
            ..OutputArgs::new()
        };
        Self {
            spi,
            cs: Output::new(cs_pin, &CS_ARGS, cs),
            kind: None,
        }
    }

    /// Card kind detected by the last successful [`init`](Self::init).
    pub fn kind(&self) -> Option<SdKind> {
        self.kind
    }

    fn xfer(&mut self, byte: u8) -> Result<u8, SdError> {
        let mut buf = [byte];
        Transfer::transfer(&mut self.spi, &mut buf).map_err(|_| SdError::Spi)?;
        Ok(buf[0])
    }

    /// Send a command and return the R1 response.
    fn cmd(&mut self, cmd: u8, arg: u32) -> Result<u8, SdError> {
        // CRC is only checked for CMD0/CMD8 in SPI mode.
        let crc = match cmd {
            0 => 0x95,
            8 => 0x87,
            _ => 0x01,
        };
        let frame = [
            0x40 | cmd,
            (arg >> 24) as u8,
            (arg >> 16) as u8,
            (arg >> 8) as u8,
            arg as u8,
            crc,
        ];
        Write::write(&mut self.spi, &frame).map_err(|_| SdError::Spi)?;
        // R1 arrives within 8 bytes (bit 7 clear).
        for _ in 0..8 {
            let r = self.xfer(0xFF)?;
            if r & 0x80 == 0 {
                return Ok(r);
            }
        }
        Err(SdError::Timeout)
    }

    fn with_cs<T>(
        &mut self,
        f: impl FnOnce(&mut Self) -> Result<T, SdError>,
    ) -> Result<T, SdError> {
        self.cs.set_level_low();
        let result = f(self);
        self.cs.set_level_high();
        // One trailing clock byte releases the card's DO line.
        let _ = self.xfer(0xFF);
        result
    }

    /// Wait for the card to release its busy signal.
    ///
    /// Feeds the watchdog: a failing card spends the full timeout here on
    /// every block, and a log flush writes several blocks between two of the
    /// main loop's feeds. The deadline is what keeps this honest.
    fn wait_not_busy(&mut self, timeout_ms: u32) -> Result<(), SdError> {
        let start = crate::platform::millis();
        loop {
            crate::watchdog::feed_now();
            if self.xfer(0xFF)? == 0xFF {
                return Ok(());
            }
            if crate::platform::millis().wrapping_sub(start) > timeout_ms {
                return Err(SdError::Timeout);
            }
        }
    }

    fn set_baud(&mut self, baud: BaudRate) {
        // The HAL exposes no baud setter; we own the peripheral inside
        // `self.spi`, so a direct CR1 update is exclusive.
        unsafe {
            let spi1 = &*pac::SPI1::PTR;
            spi1.cr1.modify(|_, w| w.spe().clear_bit());
            spi1.cr1.modify(|_, w| w.br().bits(baud as u8));
            spi1.cr1.modify(|_, w| w.spe().set_bit());
        }
    }

    /// Initialize the card (CMD0 / CMD8 / ACMD41 / CMD58 sequence).
    pub fn init(&mut self) -> Result<SdKind, SdError> {
        self.kind = None;
        self.set_baud(BaudRate::Div64);

        // At least 74 clocks with CS high to enter native mode.
        self.cs.set_level_high();
        for _ in 0..10 {
            self.xfer(0xFF)?;
        }

        let kind = self.with_cs(|sd| {
            // Software reset into idle state.
            let mut r1 = 0xFF;
            for _ in 0..32 {
                r1 = sd.cmd(0, 0)?;
                if r1 == 0x01 {
                    break;
                }
            }
            if r1 != 0x01 {
                return Err(SdError::Response(r1));
            }

            // Voltage check distinguishes v2 from v1 cards.
            let v2 = match sd.cmd(8, 0x1AA)? {
                0x01 => {
                    let mut r7 = [0u8; 4];
                    for b in &mut r7 {
                        *b = sd.xfer(0xFF)?;
                    }
                    if r7[3] != 0xAA {
                        return Err(SdError::Response(r7[3]));
                    }
                    true
                }
                _ => false, // illegal command: v1 card
            };

            // ACMD41 until the card leaves idle (up to 1 s).
            let hcs = if v2 { 0x4000_0000 } else { 0 };
            let start = crate::platform::millis();
            loop {
                crate::watchdog::feed_now();
                sd.cmd(55, 0)?;
                if sd.cmd(41, hcs)? == 0x00 {
                    break;
                }
                if crate::platform::millis().wrapping_sub(start) > 1_000 {
                    return Err(SdError::Timeout);
                }
            }

            if v2 {
                // Read OCR: CCS bit selects block addressing.
                if sd.cmd(58, 0)? != 0x00 {
                    return Err(SdError::Spi);
                }
                let mut ocr = [0u8; 4];
                for b in &mut ocr {
                    *b = sd.xfer(0xFF)?;
                }
                if ocr[0] & 0x40 != 0 {
                    return Ok(SdKind::Sdhc);
                }
            }
            // Byte-addressed cards: fix the block length at 512.
            let r1 = sd.cmd(16, SD_BLOCK_LEN as u32)?;
            if r1 != 0x00 {
                return Err(SdError::Response(r1));
            }
            Ok(if v2 { SdKind::Sd2 } else { SdKind::Sd1 })
        })?;

        self.set_baud(BaudRate::Div2); // 8 MHz for data transfers
        self.kind = Some(kind);
        Ok(kind)
    }

    fn block_addr(&self, lba: u32) -> Result<u32, SdError> {
        match self.kind {
            Some(SdKind::Sdhc) => Ok(lba),
            Some(_) => Ok(lba * SD_BLOCK_LEN as u32),
            None => Err(SdError::NotInitialized),
        }
    }

    /// Read one 512-byte block.
    pub fn read_block(&mut self, lba: u32, buf: &mut [u8; SD_BLOCK_LEN]) -> Result<(), SdError> {
        let addr = self.block_addr(lba)?;
        self.with_cs(|sd| {
            let r1 = sd.cmd(17, addr)?;
            if r1 != 0x00 {
                return Err(SdError::Response(r1));
            }
            // Wait for the data start token.
            let start = crate::platform::millis();
            loop {
                let t = sd.xfer(0xFF)?;
                if t == 0xFE {
                    break;
                }
                if t != 0xFF {
                    return Err(SdError::Response(t));
                }
                if crate::platform::millis().wrapping_sub(start) > 200 {
                    return Err(SdError::Timeout);
                }
            }
            buf.fill(0xFF);
            Transfer::transfer(&mut sd.spi, buf).map_err(|_| SdError::Spi)?;
            // Discard the 16-bit CRC.
            sd.xfer(0xFF)?;
            sd.xfer(0xFF)?;
            Ok(())
        })
    }

    /// Write one 512-byte block.
    pub fn write_block(&mut self, lba: u32, buf: &[u8; SD_BLOCK_LEN]) -> Result<(), SdError> {
        let addr = self.block_addr(lba)?;
        self.with_cs(|sd| {
            let r1 = sd.cmd(24, addr)?;
            if r1 != 0x00 {
                return Err(SdError::Response(r1));
            }
            sd.xfer(0xFF)?; // gap before the data token
            sd.xfer(0xFE)?; // start token
            Write::write(&mut sd.spi, buf).map_err(|_| SdError::Spi)?;
            // Dummy CRC.
            sd.xfer(0xFF)?;
            sd.xfer(0xFF)?;
            let resp = sd.xfer(0xFF)? & 0x1F;
            if resp != 0x05 {
                return Err(SdError::Response(resp));
            }
            sd.wait_not_busy(500)
        })
    }
}

/// Everything the Forest Sensor Platform adds on top of the bare Wio-E5
/// module, apart from the RS-485 link (see [`crate::rs422`]).
pub struct Board {
    pub senses: Senses,
    pub buck: Buck,
    pub sd: SdCard,
    pub mppt: Mppt,
    pub led: ActivityLed,
    pub rail_5v: Rail5v,
}

impl Board {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        adc: pac::ADC,
        tim2: pac::TIM2,
        spi1: pac::SPI1,
        a0: pins::A0,
        a10: pins::A10,
        b3: pins::B3,
        b4: pins::B4,
        b5: pins::B5,
        b10: pins::B10,
        b13: pins::B13,
        b14: pins::B14,
        c0: pins::C0,
        c1: pins::C1,
        rcc: &mut pac::RCC,
        cs: &CriticalSection,
    ) -> Self {
        Self {
            senses: Senses::new(adc, b13, a10, b14, rcc, cs),
            buck: Buck::new(tim2, b10, rcc, cs),
            sd: SdCard::new(spi1, b3, b4, b5, a0, rcc, cs),
            mppt: Mppt::new(),
            led: ActivityLed::new(c1, cs),
            rail_5v: Rail5v::new(c0, cs),
        }
    }

    /// One charger control step: averaged sensor sweep, then MPPT/limit
    /// update of the buck duty.  Returns the reading for logging.
    pub fn mppt_step(&mut self) -> Telemetry {
        let t = self.senses.read_avg(charge::AVG_SAMPLES);
        self.mppt.step(&t, &mut self.buck);
        t
    }
}
