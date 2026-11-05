#![no_std]
#![no_main]
#![warn(clippy::large_stack_frames)]

// The panic handler is the library's: it records the panic for the next
// boot's reset report.
#[macro_use]
extern crate sx1262_mesh_rs;

use sx1262_mesh_rs::config::RF_FREQ;

/// Format "OTA XX%" into a buffer, returning the str slice.
fn format_pct(buf: &mut [u8; 16], pct: u32) -> &str {
    let prefix = b"OTA ";
    buf[..4].copy_from_slice(prefix);
    let mut n = pct;
    let mut digits = [0u8; 3];
    let mut i = 0;
    if n == 0 {
        digits[0] = b'0';
        i = 1;
    } else {
        while n > 0 && i < 3 {
            digits[i] = b'0' + (n % 10) as u8;
            n /= 10;
            i += 1;
        }
        digits[..i].reverse();
    }
    buf[4..4 + i].copy_from_slice(&digits[..i]);
    buf[4 + i] = b'%';
    core::str::from_utf8(&buf[..5 + i]).unwrap_or("OTA ?%")
}

/// Builds a `K=<int> K=<int> ...` telemetry line sized for the 32-byte
/// mesh payload.  A field that does not fit whole is dropped.
struct TelemetryLine {
    buf: [u8; 32],
    len: usize,
}

impl TelemetryLine {
    fn new() -> Self {
        Self {
            buf: [0; 32],
            len: 0,
        }
    }

    fn push(&mut self, key: char, value: i32) {
        use core::fmt::Write as _;
        let mark = self.len;
        let sep = if self.len == 0 { "" } else { " " };
        if write!(self, "{}{}={}", sep, key, value).is_err() {
            self.len = mark;
        }
    }

    fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

impl core::fmt::Write for TelemetryLine {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let b = s.as_bytes();
        if self.len + b.len() > self.buf.len() {
            return Err(core::fmt::Error);
        }
        self.buf[self.len..self.len + b.len()].copy_from_slice(b);
        self.len += b.len();
        Ok(())
    }
}

/// The boot report, sent in place of the first three heartbeats and every
/// 150th after (about every 14 minutes): why the chip reset (`W`, a
/// `reset_record::Cause` code), what the bootloader did (`B`: 0 not started
/// by it, 1 nothing, 2 swapped an image in on trial, 3 refused one, 4
/// reverted one), the firmware version (`V`), boots since power-on (`N`),
/// the loop phase the reset hit (`P`, a `reset_record::Phase` number) and a
/// panic's source line (`L`).
fn boot_line(record: &sx1262_mesh_rs::reset_record::Record) -> TelemetryLine {
    let mut line = TelemetryLine::new();
    line.push('W', record.cause.code() as i32);
    line.push('B', record.boot_code() as i32);
    line.push('V', sx1262_mesh_rs::config::FIRMWARE_VERSION as i32);
    line.push('N', record.boots.min(i32::MAX as u32) as i32);
    if let Some(phase) = record.phase {
        line.push('P', phase as i32);
    }
    if let Some(site) = record.panic {
        line.push('L', site.line as i32);
    }
    line
}

/*
    Seeed Wio-E5 (LoRa-E5) - STM32WLE5JC

    The SX1262 radio is integrated into the MCU.  There are no external
    SPI or GPIO connections for the radio - the SubGHz peripheral handles
    everything over an internal SPI3 bus.

    TCXO:  32 MHz on DIO3 (configured via SubGHz command)
    RF SW: controlled via DIO2 (set_dio2_as_rf_switch_ctrl)

    I2C2 display (SSD1306 128x64):
        SCL - PB15
        SDA - PA15

    Debug output via RTT (probe-rs / SWD).
*/

#[rtic::app(device = stm32wlxx_hal::pac, dispatchers = [SPI1])]
mod app {
    use rtic_monotonics::systick::prelude::*;
    systick_monotonic!(Mono, 1000);

    use embedded_graphics::{
        mono_font::{MonoTextStyleBuilder, iso_8859_13::FONT_10X20},
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };
    use rtt_target::{rprintln, rtt_init, set_print_channel};
    use ssd1306::{I2CDisplayInterface, Ssd1306, mode::BufferedGraphicsMode, prelude::*};
    use stm32wlxx_hal::{
        gpio::{PortA, PortB},
        i2c::I2c2,
        pac::{FLASH, IWDG},
        subghz::SubGhz,
    };
    use sx1262_mesh_rs::OtaReceiver;
    use sx1262_mesh_rs::boot_state::BootState;
    use sx1262_mesh_rs::config::{BROADCAST_LIFETIME, MESH_LISTEN_PERIOD_MS, THIS_ADDRESS};
    use sx1262_mesh_rs::i2c_bus::{self, BoundedI2c};
    use sx1262_mesh_rs::ota_protocol;
    use sx1262_mesh_rs::platform::{SYSCLK_HZ, millis};
    use sx1262_mesh_rs::radio::{RfSwitch, Sx1262Driver};
    use sx1262_mesh_rs::reset_record::{self, Phase, SoftReset};
    use sx1262_mesh_rs::supervisor::{self, RadioAction, RadioWatch};
    use sx1262_mesh_rs::watchdog;
    use sx1262_mesh_rs::{LoraIo, MeshNode};

    /// Charger board resources; unit type when the feature is off so the
    /// RTIC resource plumbing stays identical in both builds.
    #[cfg(feature = "board")]
    type BoardRes = sx1262_mesh_rs::board::Board;
    #[cfg(not(feature = "board"))]
    type BoardRes = ();

    /// The I2C2 peripheral driving the display (and sensor suite), with a
    /// deadline on every wait.
    type I2cPeriph = BoundedI2c;
    /// With `sensor`, the bus is split between display and sensors.
    #[cfg(feature = "sensor")]
    type I2cBus = sx1262_mesh_rs::sensors::SharedI2c<I2cPeriph>;
    #[cfg(not(feature = "sensor"))]
    type I2cBus = I2cPeriph;

    #[cfg(feature = "sensor")]
    type SensorsRes = LazySensors;
    #[cfg(not(feature = "sensor"))]
    type SensorsRes = ();

    /// The sensor suite, probed at the loop's first sweep instead of in
    /// `init`, so that a bus fault costs readings, not the radio.  While
    /// `held` (after watchdog resets that hit during I2C) every read comes
    /// back empty without touching the bus.
    #[cfg(feature = "sensor")]
    pub struct LazySensors {
        bus: &'static core::cell::RefCell<I2cPeriph>,
        inner: Option<sx1262_mesh_rs::sensors::Sensors<I2cBus>>,
        held: bool,
    }

    #[cfg(feature = "sensor")]
    impl LazySensors {
        fn read(&mut self) -> sx1262_mesh_rs::sensors::Readings {
            if self.held {
                return Default::default();
            }
            let bus = self.bus;
            let readings = self
                .inner
                .get_or_insert_with(|| {
                    let s = sx1262_mesh_rs::sensors::Sensors::probe(
                        sx1262_mesh_rs::sensors::SharedI2c::new(bus),
                    );
                    rprintln!("I2C sensors found: {:?}", s.found);
                    s
                })
                .read();
            // The sweep came back, so the bus did not hang.
            reset_record::clear_i2c_strikes();
            readings
        }
    }

    #[cfg(feature = "rs422")]
    type Rs422Res = sx1262_mesh_rs::rs422::Rs422;
    #[cfg(not(feature = "rs422"))]
    type Rs422Res = ();

    #[cfg(feature = "gps-radio-log")]
    type GpsLogRes = sx1262_mesh_rs::gpslog::GpsRadioLog;
    #[cfg(not(feature = "gps-radio-log"))]
    type GpsLogRes = ();

    type Radio = Sx1262Driver;
    type Display =
        Ssd1306<I2CInterface<I2cBus>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        io: LoraIo<Radio>,
        mesh: MeshNode,
        display: Display,
        display_ok: bool,
        flash: FLASH,
        ota: OtaReceiver,
        iwdg: IWDG,
        board: BoardRes,
        sensors: SensorsRes,
        rs422: Rs422Res,
        gpslog: GpsLogRes,
        boot: BootState,
        record: reset_record::Record,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        let channels = rtt_init! {
            up: {
                0: { size: 1024, name: "Terminal" }
            }
        };
        set_print_channel(channels.up.0);

        rprintln!("Starting... 1");

        let dp = cx.device;
        let mut rcc = dp.RCC;

        // The watchdog first.  The bootloader started it before the jump,
        // with the same 5 s timeout; this sets it again, and starts it when
        // probe-rs ran this image without the bootloader.
        let iwdg = dp.IWDG;
        watchdog::start(&iwdg, 5_000);

        // Why the chip reset, and what the bootloader did this boot.
        let record = reset_record::take(&mut rcc);
        rprintln!(
            "Reset: {:?}, boot {} since power-on, bootloader: {:?}",
            record.cause,
            record.boots,
            record.handoff
        );
        if let Some(phase) = record.phase {
            rprintln!("  the loop was in {:?}", phase);
        }
        if let Some(site) = record.panic {
            rprintln!(
                "  panic at {}...:{}",
                core::str::from_utf8(&site.file).unwrap_or("?"),
                site.line
            );
        }

        // Enable DWT cycle counter for millis()/random()
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        // The charger board runs the core at 16 MHz for buck PWM
        // resolution.  Must happen before the SysTick monotonic starts
        // and before any clock-derived peripheral setup.
        #[cfg(feature = "board")]
        sx1262_mesh_rs::board::raise_sysclk(&mut rcc);

        // Start SysTick monotonic (MSI 4 MHz, or 16 MHz with `board`)
        Mono::start(cx.core.SYST, SYSCLK_HZ);

        let mut flash_periph = dp.FLASH;

        rprintln!("Starting... 2");

        // The boot log.  An image the bootloader just swapped in runs on
        // trial: the loop keeps it (appends `Confirmed`) once it has run
        // healthy long enough (`supervisor::confirm_ready`), and a reset
        // before that makes the bootloader revert it.
        let boot = BootState::load(&mut flash_periph, record.handoff);
        if boot.on_trial() {
            rprintln!(
                "Boot: new image on trial, kept after {} min healthy",
                supervisor::CONFIRM_AFTER_MS / 60_000
            );
        }

        let gpioa = PortA::split(dp.GPIOA, &mut rcc);
        let gpiob = PortB::split(dp.GPIOB, &mut rcc);

        // ---- SubGHz radio (integrated SX1262) and its antenna switch ---------
        let sg = SubGhz::new(dp.SPI3, &mut rcc);
        let rf_switch = cortex_m::interrupt::free(|cs| RfSwitch::new(gpioa.a4, gpioa.a5, cs));
        let mut radio = Sx1262Driver::new(sg, rf_switch);
        radio.init(super::RF_FREQ);
        radio.print_diagnostics();

        // ---- I2C2 display (SSD1306 128x64) -----------------------------------
        // Clock out a sensor that a reset left holding the bus, then hand the
        // pins to the I2C driver.  Nothing on the bus is addressed until the
        // loop runs: a bus fault costs the display and the sensors there,
        // never the radio.
        let i2c = cortex_m::interrupt::free(|cs| {
            let pins = i2c_bus::bus_clear(gpiob.b15, gpioa.a15, cs);
            BoundedI2c::new(I2c2::new(dp.I2C2, pins, 100_000, &mut rcc, true, cs))
        });

        // With `sensor`, split the bus between the display and the
        // sensor suite; everything runs at the same task priority.
        #[cfg(feature = "sensor")]
        let (i2c, sensors) = {
            use core::cell::RefCell;
            let bus: &'static RefCell<I2cPeriph> =
                cortex_m::singleton!(: RefCell<I2cPeriph> = RefCell::new(i2c)).unwrap();
            let sensors = LazySensors {
                bus,
                inner: None,
                held: false,
            };
            (sx1262_mesh_rs::sensors::SharedI2c::new(bus), sensors)
        };
        #[cfg(not(feature = "sensor"))]
        let sensors = ();

        let display = Ssd1306::new(
            I2CDisplayInterface::new(i2c),
            DisplaySize128x64,
            DisplayRotation::Rotate0,
        )
        .into_buffered_graphics_mode();
        // The loop's first pass brings it up.
        let display_ok = false;

        // ---- Forest Sensor Platform: senses, buck PWM, SD card, LED, 5 V ----
        #[cfg(feature = "board")]
        let board = {
            use stm32wlxx_hal::gpio::PortC;
            let gpioc = PortC::split(dp.GPIOC, &mut rcc);
            let mut board = cortex_m::interrupt::free(|cs| {
                sx1262_mesh_rs::board::Board::new(
                    dp.ADC, dp.TIM2, dp.SPI1, gpioa.a0, gpioa.a10, gpiob.b3, gpiob.b4, gpiob.b5,
                    gpiob.b10, gpiob.b13, gpiob.b14, gpioc.c0, gpioc.c1, &mut rcc, cs,
                )
            });
            board.buck.off();
            watchdog::feed(&iwdg);
            // No card-detect switch on this board: a failed init is how a
            // missing card shows up.
            match board.sd.init() {
                Ok(kind) => {
                    rprintln!("SD card ready: {:?}", kind);
                }
                Err(e) => {
                    rprintln!("SD card init failed (no card?): {:?}", e);
                }
            }
            let t = board.senses.read();
            rprintln!(
                "Board: vin={} mV vbat={} mV ibat={} mA",
                t.vin_mv,
                t.vbat_mv,
                t.ibat_ma
            );
            board
        };
        #[cfg(not(feature = "board"))]
        let board = ();

        // ---- RS-485 field bus (TPT482 on USART1, DE PA9, /RE PB9) ------------
        #[cfg(feature = "rs422")]
        let rs422 = {
            let link = cortex_m::interrupt::free(|cs| {
                sx1262_mesh_rs::rs422::Rs422::new(
                    dp.USART1, gpiob.b6, gpiob.b7, gpioa.a9, gpiob.b9, &mut rcc, cs,
                )
            });
            rprintln!(
                "RS-485 link on USART1 (PB6 TX / PB7 RX) at {} baud",
                sx1262_mesh_rs::rs422::BAUD
            );
            link
        };
        #[cfg(not(feature = "rs422"))]
        let rs422 = ();

        // ---- GPS + radio link logger (NMEA on USART1) ------------------------
        // The GPS stays powered and active; SD logging is brought up
        // lazily inside poll() so a late-inserted card still works.
        #[cfg(feature = "gps-radio-log")]
        let gpslog = {
            let g = cortex_m::interrupt::free(|cs| {
                sx1262_mesh_rs::gpslog::GpsRadioLog::new(dp.USART1, gpiob.b7, &mut rcc, cs)
            });
            rprintln!(
                "GPS on USART1 (PB7 RX) at {} baud, logging radio + GPS to SD",
                sx1262_mesh_rs::gpslog::BAUD
            );
            g
        };
        #[cfg(not(feature = "gps-radio-log"))]
        let gpslog = ();

        // ---- Mesh networking -------------------------------------------------
        debug_println!(
            "Starting nano-mesh (address={}, freq={} Hz)...",
            THIS_ADDRESS,
            super::RF_FREQ
        );
        let io = LoraIo::new(radio);
        let mesh = MeshNode::new(THIS_ADDRESS, MESH_LISTEN_PERIOD_MS);
        let ota = OtaReceiver::new();

        rprintln!("Mesh node {} ready", THIS_ADDRESS);

        run::spawn().unwrap();

        (
            Shared {},
            Local {
                io,
                mesh,
                display,
                display_ok,
                flash: flash_periph,
                ota,
                iwdg,
                board,
                sensors,
                rs422,
                gpslog,
                boot,
                record,
            },
        )
    }

    #[task(local = [io, mesh, display, display_ok, flash, ota, iwdg, board, sensors, rs422, gpslog, boot, record], priority = 1)]
    async fn run(cx: run::Context) {
        let io = cx.local.io;
        let mesh = cx.local.mesh;
        let display = cx.local.display;
        let display_ok = cx.local.display_ok;
        let flash = cx.local.flash;
        let ota = cx.local.ota;
        let iwdg = cx.local.iwdg;
        let boot = cx.local.boot;
        let record = cx.local.record;
        #[cfg(feature = "board")]
        let board = cx.local.board;
        #[cfg(feature = "sensor")]
        let sensors = cx.local.sensors;
        #[cfg(feature = "rs422")]
        let rs422 = cx.local.rs422;
        #[cfg(feature = "gps-radio-log")]
        let gpslog = cx.local.gpslog;

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(BinaryColor::On)
            .build();

        // The combined GPS/radio status line packs three fields, so it
        // uses a narrower font than the lat/long readout below it.
        #[cfg(feature = "gps-radio-log")]
        let status_style = MonoTextStyleBuilder::new()
            .font(&embedded_graphics::mono_font::iso_8859_13::FONT_7X13)
            .text_color(BinaryColor::On)
            .build();

        // Stagger first TX by address so nodes don't collide on boot.
        //
        // Folded into eight slots rather than scaled by the whole address:
        // node 200 otherwise sat silent for ten minutes after boot with
        // nothing on the console to say why. A second apart is already wide
        // against a packet's air time, and the post-transmit jitter keeps
        // nodes apart from there.
        let tx_interval = 4_000_u32.millis();
        let mut next_tx = Mono::now() + ((THIS_ADDRESS as u32 % 8) * 1_000).millis();
        let mut tx_count: u32 = 0;
        let mut rx_count: u32 = 0;

        // Bring the display up on the first pass, then retry every 10 s
        // while it is missing.
        let display_retry_interval = 10_000_u32.millis();
        let mut next_display_retry = Mono::now();

        // MPPT charger control period
        #[cfg(feature = "board")]
        let mut next_mppt = Mono::now();

        // Radio stats snapshot period for the SD log
        #[cfg(feature = "gps-radio-log")]
        let stats_interval = 60_000_u32.millis();
        #[cfg(feature = "gps-radio-log")]
        let mut next_stats = Mono::now() + stats_interval;

        // Refresh the lat/long readout on the display at ~1 Hz (the GPS
        // fix rate); the position changes slowly so this avoids flicker.
        #[cfg(feature = "gps-radio-log")]
        let gps_display_interval = 1_000_u32.millis();
        #[cfg(feature = "gps-radio-log")]
        let mut next_gps_display = Mono::now();

        // Environmental sensor sweep every 30 s (first one shortly after
        // boot so the SCD41's 5 s first measurement is ready).
        #[cfg(any(feature = "sensor", feature = "rs422"))]
        let sensor_interval = 30_000_u32.millis();
        #[cfg(any(feature = "sensor", feature = "rs422"))]
        let mut next_sensor = Mono::now() + 6_000_u32.millis();

        // Health checks beyond the watchdog (see `supervisor`): the radio
        // has to keep transmitting, and an image on trial is kept once it
        // has run healthy long enough.
        let loop_start = millis();
        let mut radio_watch = RadioWatch::new(loop_start);
        let mut next_confirm = loop_start;

        // Watchdog resets in a row that hit during I2C keep the node off the
        // bus for a while, longer each time: the radio stays up even when a
        // sensor fault hangs the bus on every attempt.
        let hold_ms = supervisor::i2c_hold_ms(record.i2c_strikes);
        let mut i2c_hold_until = (hold_ms > 0).then(|| loop_start.wrapping_add(hold_ms));
        if hold_ms > 0 {
            rprintln!(
                "I2C held off for {} min after {} watchdog resets during I2C",
                hold_ms / 60_000,
                record.i2c_strikes
            );
        }

        loop {
            if let Some(until) = i2c_hold_until
                && millis().wrapping_sub(until) as i32 >= 0
            {
                i2c_hold_until = None;
            }
            let i2c_ok = i2c_hold_until.is_none();

            // Retry display connection if not detected.
            // Skip all I2C during OTA - a stuck bus could trigger a watchdog
            // reset and lose the in-progress transfer.
            //
            // Checked before that gate, so a sender that vanished mid-upload
            // releases the node instead of holding it off the air until it is
            // power-cycled.
            if ota.expire() {
                rprintln!("OTA transfer abandoned (sender went silent)");
            }
            let ota_active = ota.is_active();
            if !ota_active && i2c_ok && !*display_ok && Mono::now() >= next_display_retry {
                reset_record::set_phase(Phase::Display);
                if display.init().is_ok()
                    && display.clear(BinaryColor::Off).is_ok()
                    && display.flush().is_ok()
                {
                    rprintln!("I2C display ready");
                    *display_ok = true;
                    reset_record::clear_i2c_strikes();
                } else {
                    next_display_retry = Mono::now() + display_retry_interval;
                }
            }
            // Pulse the radio activity LED (D12 on PC1) on TX and RX.
            #[cfg(feature = "board")]
            board.led.update(sx1262_mesh_rs::platform::millis());

            // The soil probe on J7 is powered from the switched 5 V rail:
            // bring it up early enough for the probe to start before the
            // sweep polls it.  The sweep switches it off again.
            #[cfg(feature = "rs422")]
            if !ota_active
                && !board.rail_5v.is_on()
                && Mono::now() + sx1262_mesh_rs::rs422::SOIL_WARMUP_MS.millis() >= next_sensor
            {
                board.rail_5v.set(true);
            }

            // Drain the GPS and radio event queue into the SD log.
            #[cfg(feature = "gps-radio-log")]
            {
                reset_record::set_phase(Phase::GpsLog);
                gpslog.poll(&mut board.sd, sx1262_mesh_rs::platform::millis());
                if Mono::now() >= next_stats {
                    if let Ok(stats) = io.inner().lora_stats() {
                        gpslog.log_stats(&mut board.sd, sx1262_mesh_rs::platform::millis(), stats);
                    }
                    next_stats = Mono::now() + stats_interval;
                }

                // Show the current fix and lat/long on the display.
                if !ota_active && *display_ok && Mono::now() >= next_gps_display {
                    let gps = &gpslog.gps;
                    // Milliseconds since the last radio packet (wrapping_sub
                    // tracks the DWT millis rollover), or None if none yet.
                    let since_rx = io
                        .last_rx_ms()
                        .map(|t| sx1262_mesh_rs::platform::millis().wrapping_sub(t));
                    let mut buf = [0u8; 24];
                    display.clear(BinaryColor::Off).ok();
                    Text::with_baseline(
                        gps.fmt_status_line(&mut buf, io.last_rssi(), since_rx),
                        Point::new(2, 6),
                        status_style,
                        Baseline::Top,
                    )
                    .draw(display)
                    .ok();
                    if gps.has_pos {
                        let mut latbuf = [0u8; 16];
                        let mut lonbuf = [0u8; 16];
                        Text::with_baseline(
                            gps.fmt_lat(&mut latbuf),
                            Point::new(2, 24),
                            text_style,
                            Baseline::Top,
                        )
                        .draw(display)
                        .ok();
                        Text::with_baseline(
                            gps.fmt_lon(&mut lonbuf),
                            Point::new(2, 44),
                            text_style,
                            Baseline::Top,
                        )
                        .draw(display)
                        .ok();
                    }
                    if display.flush().is_err() {
                        rprintln!("I2C display lost");
                        *display_ok = false;
                        next_display_retry = Mono::now() + display_retry_interval;
                    }
                    next_gps_display = Mono::now() + gps_display_interval;
                }
            }

            // Charger control loop: sample, then perturb the buck duty.
            #[cfg(feature = "board")]
            if Mono::now() >= next_mppt {
                reset_record::set_phase(Phase::Charger);
                let t = board.mppt_step();
                debug_println!(
                    "MPPT: vin={} vbat={} ibat={} duty={} {:?}",
                    t.vin_mv,
                    t.vbat_mv,
                    t.ibat_ma,
                    board.mppt.duty_permille(),
                    board.mppt.state()
                );
                next_mppt = Mono::now() + sx1262_mesh_rs::board::charge::MPPT_PERIOD_MS.millis();
            }

            // Environmental sensor sweep.  Blocking (worst ~0.5 s), so
            // skipped while an OTA transfer is in flight.  The I2C sensors
            // read nothing while the bus is held off.
            #[cfg(feature = "sensor")]
            {
                sensors.held = !i2c_ok;
            }
            #[cfg(any(feature = "sensor", feature = "rs422"))]
            if !ota_active && Mono::now() >= next_sensor {
                reset_record::set_phase(if cfg!(feature = "sensor") {
                    Phase::Sensors
                } else {
                    Phase::Soil
                });
                // Compact `K=<int>` line broadcast after the sweep so the
                // basestation (and the dashboard behind it) sees the
                // readings: T/H centi-units, C ppm, M permille.
                let mut tele = super::TelemetryLine::new();
                #[cfg(feature = "sensor")]
                {
                    let r = sensors.read();
                    if let Some(b) = r.bme {
                        rprintln!(
                            "BME680: T={}.{:02} C P={} Pa RH={}.{:02} % gas={} ohm",
                            b.temp_c_x100 / 100,
                            (b.temp_c_x100 % 100).unsigned_abs(),
                            b.press_pa,
                            b.hum_pct_x100 / 100,
                            b.hum_pct_x100 % 100,
                            b.gas_ohm
                        );
                    }
                    if let Some(c) = r.co2 {
                        rprintln!(
                            "SCD41: CO2={} ppm T={}.{:02} C RH={}.{:02} %",
                            c.co2_ppm,
                            c.temp_c_x100 / 100,
                            (c.temp_c_x100 % 100).unsigned_abs(),
                            c.hum_pct_x100 / 100,
                            c.hum_pct_x100 % 100
                        );
                    }
                    if let Some(s) = r.sht {
                        rprintln!(
                            "SHT45: T={}.{:02} C RH={}.{:02} %",
                            s.temp_c_x100 / 100,
                            (s.temp_c_x100 % 100).unsigned_abs(),
                            s.hum_pct_x100 / 100,
                            s.hum_pct_x100 % 100
                        );
                    }
                    if let Some(m) = r.mcp {
                        rprintln!(
                            "MCP9808: T={}.{:02} C",
                            m.temp_c_x100 / 100,
                            (m.temp_c_x100 % 100).unsigned_abs()
                        );
                    }
                    if let Some(b) = r.baro {
                        rprintln!(
                            "BMP280: T={}.{:02} C P={} Pa",
                            b.temp_c_x100 / 100,
                            (b.temp_c_x100 % 100).unsigned_abs(),
                            b.press_pa
                        );
                    }
                    if let Some(a) = r.accel {
                        rprintln!("ADXL: x={} y={} z={} mg", a.x_mg, a.y_mg, a.z_mg);
                    }
                    if let Some(l) = r.lightning {
                        rprintln!(
                            "AS3935: event src={:#04x} distance={} km",
                            l.int_src,
                            l.distance_km
                        );
                    }

                    // Best available source per quantity.
                    let temp = r
                        .bme
                        .map(|b| b.temp_c_x100)
                        .or(r.sht.map(|s| s.temp_c_x100))
                        .or(r.co2.map(|c| c.temp_c_x100))
                        .or(r.mcp.map(|m| m.temp_c_x100))
                        .or(r.baro.map(|b| b.temp_c_x100));
                    let hum = r
                        .bme
                        .map(|b| b.hum_pct_x100)
                        .or(r.sht.map(|s| s.hum_pct_x100))
                        .or(r.co2.map(|c| c.hum_pct_x100));
                    if let Some(t) = temp {
                        tele.push('T', t);
                    }
                    if let Some(h) = hum {
                        tele.push('H', h as i32);
                    }
                    if let Some(c) = r.co2 {
                        tele.push('C', c.co2_ppm as i32);
                    }
                }
                #[cfg(feature = "rs422")]
                {
                    match rs422.read_soil() {
                        Ok(soil) => {
                            rprintln!(
                                "Soil: {}.{} % {}.{} C",
                                soil.moisture_pct_x10 / 10,
                                soil.moisture_pct_x10 % 10,
                                soil.temp_c_x10 / 10,
                                (soil.temp_c_x10 % 10).unsigned_abs()
                            );
                            tele.push('M', soil.moisture_pct_x10 as i32);
                        }
                        Err(e) => debug_println!("Soil probe: {:?}", e),
                    }
                    board.rail_5v.set(false);
                }
                if !tele.as_bytes().is_empty() {
                    if let Err(e) = mesh.broadcast(tele.as_bytes(), BROADCAST_LIFETIME) {
                        rprintln!("Sensor TX failed: {:?}", e);
                    }
                }
                next_sensor = Mono::now() + sensor_interval;
            }

            // Drive the mesh protocol (receive, forward, transmit)
            reset_record::set_phase(Phase::Mesh);
            mesh.update(io, sx1262_mesh_rs::platform::millis());

            // Check for incoming messages
            if let Some(msg) = mesh.receive() {
                if ota_protocol::is_ota_message(&msg.data) {
                    // Route to OTA handler
                    reset_record::set_phase(Phase::Ota);
                    if let Some(response) = ota.handle_message(&msg.data, flash, boot) {
                        mesh.send(
                            &response.data[..response.len],
                            msg.source,
                            BROADCAST_LIFETIME,
                        )
                        .ok();
                    }
                    // Show OTA progress on display if available
                    if *display_ok {
                        if let Some((done, total)) = ota.progress() {
                            let pct = (done as u32 * 100) / total as u32;
                            let mut line_buf = [0u8; 16];
                            let line = super::format_pct(&mut line_buf, pct);
                            display.clear(BinaryColor::Off).ok();
                            Text::with_baseline(
                                line,
                                Point::new(5, 64 / 2),
                                text_style,
                                Baseline::Middle,
                            )
                            .draw(display)
                            .ok();
                            if display.flush().is_err() {
                                rprintln!("I2C display lost");
                                *display_ok = false;
                                next_display_retry = Mono::now() + display_retry_interval;
                            }
                        }
                    }
                } else {
                    rx_count += 1;
                    let text = core::str::from_utf8(&msg.data).unwrap_or("<invalid utf8>");
                    rprintln!(
                        "RX #{} from={} rssi={} dBm: {}",
                        rx_count,
                        msg.source,
                        io.last_rssi(),
                        text
                    );
                    debug_println!("  len={} raw={:?}", msg.data.len(), &msg.data[..]);

                    // With the GPS logger the display is dedicated to the
                    // lat/long readout, so skip the received-message banner.
                    #[cfg(not(feature = "gps-radio-log"))]
                    if *display_ok {
                        const LEN: usize = 32;
                        let mut send_header: [u8; LEN] = *b"In:                             ";

                        let offset = 4;
                        let len = text.len().min(LEN - offset);

                        send_header[offset..offset + len].copy_from_slice(text.as_bytes());

                        display.clear(BinaryColor::Off).ok();
                        Text::with_baseline(
                            core::str::from_utf8(&send_header).unwrap_or("UTF8 Error Receiving"),
                            Point::new(5, 64 / 2),
                            text_style,
                            Baseline::Middle,
                        )
                        .draw(display)
                        .ok();
                        if display.flush().is_err() {
                            rprintln!("I2C display lost");
                            *display_ok = false;
                            next_display_retry = Mono::now() + display_retry_interval;
                        }
                    }
                }
            }

            // A verified image is staged and its OTA_COMPLETE has had time
            // to go out: reboot, and the bootloader swaps it in.
            if ota.reboot_due(millis()) {
                rprintln!("OTA: rebooting into the staged image");
                reset_record::software_reset(SoftReset::Ota);
            }

            // The radio has to keep transmitting: re-initialize it after ten
            // minutes without a completed transmission, and reset the chip
            // if that does not help (`supervisor::RadioWatch`).
            match radio_watch.poll(millis(), io.last_tx_ok_ms(), io.last_rx_ms()) {
                RadioAction::Nothing => {}
                RadioAction::Reinit => {
                    rprintln!("Radio: no traffic, re-initializing");
                    io.inner().init(super::RF_FREQ);
                }
                RadioAction::Reset => {
                    rprintln!("Radio: still not transmitting after a re-init, resetting");
                    reset_record::software_reset(SoftReset::RadioStall);
                }
            }

            // Keep an image on trial once it has run healthy long enough.
            let now = millis();
            if boot.on_trial()
                && now.wrapping_sub(next_confirm) as i32 >= 0
                && supervisor::confirm_ready(
                    now,
                    loop_start,
                    io.last_tx_ok_ms(),
                    io.last_rx_ms().is_some(),
                )
            {
                match boot.confirm(flash) {
                    Ok(()) => rprintln!("Boot: image confirmed"),
                    Err(e) => {
                        rprintln!("Boot: confirm failed ({:?}), retrying in a minute", e);
                        next_confirm = now.wrapping_add(60_000);
                    }
                }
            }

            // Send a heartbeat (broadcast).  With the `board` feature the
            // heartbeat carries the sensor readings instead of "hello".
            if Mono::now() >= next_tx {
                reset_record::set_phase(Phase::Heartbeat);
                #[cfg(feature = "board")]
                let mut msg_buf = [0u8; 32];
                #[cfg(feature = "board")]
                let message: &[u8] = {
                    let t = board.senses.read_avg(4);
                    let duty = board.mppt.duty_permille();
                    let state = board.mppt.state();
                    rprintln!(
                        "Board: vin={} mV vbat={} mV ibat={} mA duty={} {:?}",
                        t.vin_mv,
                        t.vbat_mv,
                        t.ibat_ma,
                        duty,
                        state
                    );
                    t.format_status(duty, state, &mut msg_buf).as_bytes()
                };
                #[cfg(not(feature = "board"))]
                let message: &[u8] = b"hello";

                tx_count += 1;
                // The boot report goes out instead of the first three
                // heartbeats and every 150th after.
                let report = super::boot_line(record);
                let message: &[u8] = if tx_count <= 3 || tx_count % 150 == 0 {
                    report.as_bytes()
                } else {
                    message
                };
                match mesh.broadcast(
                    core::str::from_utf8(message)
                        .unwrap_or("UTF8 Message Error")
                        .as_bytes(),
                    BROADCAST_LIFETIME,
                ) {
                    Ok(()) => {
                        rprintln!("TX #{}", tx_count);
                    }
                    Err(e) => {
                        rprintln!("TX #{} failed: {:?}", tx_count, e);
                    }
                }

                // With the GPS logger the display shows lat/long instead of
                // the transmitted-message banner.
                #[cfg(not(feature = "gps-radio-log"))]
                if *display_ok {
                    const LEN: usize = 32;
                    let mut send_header: [u8; LEN] = *b"Out:                            ";

                    let offset = 5;
                    let len = message.len().min(LEN - offset);

                    send_header[offset..offset + len].copy_from_slice(&message[..len]);

                    display.clear(BinaryColor::Off).ok();
                    Text::with_baseline(
                        core::str::from_utf8(&send_header).unwrap_or("Error decoding"),
                        Point::new(5, 64 / 2),
                        text_style,
                        Baseline::Middle,
                    )
                    .draw(display)
                    .ok();
                    if display.flush().is_err() {
                        rprintln!("I2C display lost");
                        *display_ok = false;
                        next_display_retry = Mono::now() + display_retry_interval;
                    }
                }

                // Schedule next TX with 0-3s jitter
                let jitter_ms = sx1262_mesh_rs::platform::random(0, 3000) as u32;
                next_tx = Mono::now() + tx_interval + jitter_ms.millis();
            }

            reset_record::set_phase(Phase::Idle);
            watchdog::feed(iwdg);
            Mono::delay(1_u32.millis()).await;
        }
    }
}
