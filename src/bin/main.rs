#![no_std]
#![no_main]
#![warn(clippy::large_stack_frames)]

use panic_halt as _;

#[macro_use]
extern crate sx1262_mesh_rs;

/// RF frequency in Hz (915 MHz ISM band).
const RF_FREQ: u32 = 915_000_000;

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
#[cfg(any(feature = "sensor", feature = "rs422"))]
struct TelemetryLine {
    buf: [u8; 32],
    len: usize,
}

#[cfg(any(feature = "sensor", feature = "rs422"))]
impl TelemetryLine {
    fn new() -> Self {
        Self { buf: [0; 32], len: 0 }
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

#[cfg(any(feature = "sensor", feature = "rs422"))]
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

/*
    Seeed Wio-E5 (LoRa-E5) — STM32WLE5JC

    The SX1262 radio is integrated into the MCU.  There are no external
    SPI or GPIO connections for the radio — the SubGHz peripheral handles
    everything over an internal SPI3 bus.

    TCXO:  32 MHz on DIO3 (configured via SubGHz command)
    RF SW: controlled via DIO2 (set_dio2_as_rf_switch_ctrl)

    I2C2 display (SSD1306 128x64):
        SCL — PB15
        SDA — PA15

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
    use sx1262_mesh_rs::{LoraIo, MeshNode};
    use rtt_target::{rprintln, rtt_init, set_print_channel};
    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};
    use stm32wlxx_hal::{
        gpio::{pins, PortA, PortB},
        i2c::I2c2,
        pac::{FLASH, IWDG},
        subghz::SubGhz,
    };
    use sx1262_mesh_rs::config::{BROADCAST_LIFETIME, MESH_LISTEN_PERIOD_MS, THIS_ADDRESS};
    use sx1262_mesh_rs::ota_protocol;
    use sx1262_mesh_rs::platform::SYSCLK_HZ;
    use sx1262_mesh_rs::radio::Sx1262Driver;
    use sx1262_mesh_rs::OtaReceiver;
    use sx1262_mesh_rs::watchdog;

    /// Charger board resources; unit type when the feature is off so the
    /// RTIC resource plumbing stays identical in both builds.
    #[cfg(feature = "board")]
    type BoardRes = sx1262_mesh_rs::board::Board;
    #[cfg(not(feature = "board"))]
    type BoardRes = ();

    /// The I2C2 peripheral driving the display (and sensor suite).
    type I2cPeriph = I2c2<(pins::B15, pins::A15)>;
    /// With `sensor`, the bus is split between display and sensors.
    #[cfg(feature = "sensor")]
    type I2cBus = sx1262_mesh_rs::sensors::SharedI2c<I2cPeriph>;
    #[cfg(not(feature = "sensor"))]
    type I2cBus = I2cPeriph;

    #[cfg(feature = "sensor")]
    type SensorsRes = sx1262_mesh_rs::sensors::Sensors<I2cBus>;
    #[cfg(not(feature = "sensor"))]
    type SensorsRes = ();

    #[cfg(feature = "rs422")]
    type Rs422Res = sx1262_mesh_rs::rs422::Rs422;
    #[cfg(not(feature = "rs422"))]
    type Rs422Res = ();

    #[cfg(feature = "gps-radio-log")]
    type GpsLogRes = sx1262_mesh_rs::gpslog::GpsRadioLog;
    #[cfg(not(feature = "gps-radio-log"))]
    type GpsLogRes = ();

    type Radio = Sx1262Driver;
    type Display = Ssd1306<
        I2CInterface<I2cBus>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >;

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


        // Enable DWT cycle counter for millis()/random()
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        let dp = cx.device;
        let mut rcc = dp.RCC;

        // The charger board runs the core at 16 MHz for buck PWM
        // resolution.  Must happen before the SysTick monotonic starts
        // and before any clock-derived peripheral setup.
        #[cfg(feature = "board")]
        sx1262_mesh_rs::board::raise_sysclk(&mut rcc);

        // Start SysTick monotonic (MSI 4 MHz, or 16 MHz with `board`)
        Mono::start(cx.core.SYST, SYSCLK_HZ);

        let mut flash_periph = dp.FLASH;

        rprintln!("Starting... 2");


        // Start watchdog (5 s timeout). If the app never reaches confirm_boot
        // or hangs during init, the MCU resets and the bootloader reverts.
        let iwdg = dp.IWDG;
        watchdog::start(&iwdg, 5_000);

        // Confirm boot to the bootloader (marks firmware as healthy).
        sx1262_mesh_rs::boot_state::confirm_boot(&mut flash_periph);

        // ---- SubGHz radio (integrated SX1262) --------------------------------
        let sg = SubGhz::new(dp.SPI3, &mut rcc);
        let mut radio = Sx1262Driver::new(sg);
        radio.init(super::RF_FREQ);
        radio.print_diagnostics();

        // ---- I2C2 display (SSD1306 128x64) -----------------------------------
        let gpioa = PortA::split(dp.GPIOA, &mut rcc);
        let gpiob = PortB::split(dp.GPIOB, &mut rcc);
        let i2c = cortex_m::interrupt::free(|cs| {
            I2c2::new(dp.I2C2, (gpiob.b15, gpioa.a15), 100_000, &mut rcc, true, cs)
        });

        // With `sensor`, split the bus between the display and the
        // sensor suite; everything runs at the same task priority.
        #[cfg(feature = "sensor")]
        let (i2c, sensors) = {
            use core::cell::RefCell;
            let bus: &'static RefCell<I2cPeriph> =
                cortex_m::singleton!(: RefCell<I2cPeriph> = RefCell::new(i2c)).unwrap();
            let sensors = sx1262_mesh_rs::sensors::Sensors::probe(
                sx1262_mesh_rs::sensors::SharedI2c::new(bus),
            );
            rprintln!("I2C sensors found: {:?}", sensors.found);
            (sx1262_mesh_rs::sensors::SharedI2c::new(bus), sensors)
        };
        #[cfg(not(feature = "sensor"))]
        let sensors = ();

        let mut display = Ssd1306::new(
            I2CDisplayInterface::new(i2c),
            DisplaySize128x64,
            DisplayRotation::Rotate0,
        )
        .into_buffered_graphics_mode();
        watchdog::feed(&iwdg);
        let display_ok = if display.init().is_ok()
            && display.clear(BinaryColor::Off).is_ok()
            && display.flush().is_ok()
        {
            rprintln!("I2C display detected");
            true
        } else {
            rprintln!("I2C display not detected, will retry");
            false
        };

        // ---- Charger board: senses, buck PWM, SD card, LEDs ------------------
        #[cfg(feature = "board")]
        let board = {
            use stm32wlxx_hal::gpio::PortC;
            let gpioc = PortC::split(dp.GPIOC, &mut rcc);
            let mut board = cortex_m::interrupt::free(|cs| {
                sx1262_mesh_rs::board::Board::new(
                    dp.ADC,
                    dp.TIM1,
                    dp.SPI1,
                    gpioa.a0,
                    gpioa.a9,
                    gpioa.a10,
                    gpiob.b3,
                    gpiob.b4,
                    gpiob.b5,
                    gpiob.b9,
                    gpiob.b13,
                    gpiob.b14,
                    gpioc.c0,
                    gpioc.c1,
                    &mut rcc,
                    cs,
                )
            });
            board.buck.off();
            watchdog::feed(&iwdg);
            if board.sd.card_present() {
                match board.sd.init() {
                    Ok(kind) => rprintln!("SD card ready: {:?}", kind),
                    Err(e) => rprintln!("SD card init failed: {:?}", e),
                }
            } else {
                rprintln!("No SD card inserted");
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

        // ---- RS-422 field bus (MAX3430 on USART2) ----------------------------
        #[cfg(feature = "rs422")]
        let rs422 = {
            let link = cortex_m::interrupt::free(|cs| {
                sx1262_mesh_rs::rs422::Rs422::new(dp.USART2, gpioa.a2, gpioa.a3, &mut rcc, cs)
            });
            rprintln!(
                "RS-422 link on USART2 (PA2 TX / PA3 RX) at {} baud",
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

        (Shared {}, Local { io, mesh, display, display_ok, flash: flash_periph, ota, iwdg, board, sensors, rs422, gpslog })
    }

    #[task(local = [io, mesh, display, display_ok, flash, ota, iwdg, board, sensors, rs422, gpslog], priority = 1)]
    async fn run(cx: run::Context) {
        let io = cx.local.io;
        let mesh = cx.local.mesh;
        let display = cx.local.display;
        let display_ok = cx.local.display_ok;
        let flash = cx.local.flash;
        let ota = cx.local.ota;
        let iwdg = cx.local.iwdg;
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

        // Stagger first TX by address so nodes don't collide on boot
        let tx_interval = 10_000_u32.millis();
        let mut next_tx = Mono::now() + (THIS_ADDRESS as u32 * 3_000).millis();
        let mut tx_count: u32 = 0;
        let mut rx_count: u32 = 0;

        // Retry display init every 10 s if not connected
        let display_retry_interval = 10_000_u32.millis();
        let mut next_display_retry = Mono::now() + display_retry_interval;

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

        loop {
            // Retry display connection if not detected.
            // Skip all I2C during OTA — a stuck bus could trigger a watchdog
            // reset and lose the in-progress transfer.
            let ota_active = ota.is_active();
            if !ota_active && !*display_ok && Mono::now() >= next_display_retry {
                watchdog::feed(iwdg);
                if display.init().is_ok()
                    && display.clear(BinaryColor::Off).is_ok()
                    && display.flush().is_ok()
                {
                    rprintln!("I2C display reconnected");
                    *display_ok = true;
                } else {
                    next_display_retry = Mono::now() + display_retry_interval;
                }
            }
            // Pulse the radio activity LEDs (TX on PC0, RX on PC1).
            #[cfg(feature = "board")]
            board.leds.update(sx1262_mesh_rs::platform::millis());

            // Drain the GPS and radio event queue into the SD log.
            #[cfg(feature = "gps-radio-log")]
            {
                gpslog.poll(&mut board.sd, sx1262_mesh_rs::platform::millis());
                if Mono::now() >= next_stats {
                    if let Ok(stats) = io.inner().lora_stats() {
                        gpslog.log_stats(
                            &mut board.sd,
                            sx1262_mesh_rs::platform::millis(),
                            stats,
                        );
                    }
                    next_stats = Mono::now() + stats_interval;
                }

                // Show the current fix and lat/long on the display.
                if !ota_active && *display_ok && Mono::now() >= next_gps_display {
                    let gps = &gpslog.gps;
                    let mut buf = [0u8; 16];
                    display.clear(BinaryColor::Off).ok();
                    Text::with_baseline(
                        gps.fmt_status(&mut buf),
                        Point::new(2, 2),
                        text_style,
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
                let t = board.mppt_step();
                debug_println!(
                    "MPPT: vin={} vbat={} ibat={} duty={} {:?}",
                    t.vin_mv,
                    t.vbat_mv,
                    t.ibat_ma,
                    board.mppt.duty_permille(),
                    board.mppt.state()
                );
                next_mppt = Mono::now()
                    + sx1262_mesh_rs::board::charge::MPPT_PERIOD_MS.millis();
            }

            // Environmental sensor sweep.  Blocking (worst ~0.5 s), so
            // skipped while an OTA transfer is in flight.
            #[cfg(any(feature = "sensor", feature = "rs422"))]
            if !ota_active && Mono::now() >= next_sensor {
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
                if !tele.as_bytes().is_empty() {
                    if let Err(e) = mesh.broadcast(tele.as_bytes(), BROADCAST_LIFETIME) {
                        rprintln!("Sensor TX failed: {:?}", e);
                    }
                }
                watchdog::feed(iwdg);
                next_sensor = Mono::now() + sensor_interval;
            }

            // Drive the mesh protocol (receive, forward, transmit)
            mesh.update(io, sx1262_mesh_rs::platform::millis());

            // Check for incoming messages
            if let Some(msg) = mesh.receive() {
                if ota_protocol::is_ota_message(&msg.data) {
                    // Route to OTA handler
                    if let Some(response) = ota.handle_message(&msg.data, flash) {
                        mesh.send(&response.data[..response.len], msg.source, BROADCAST_LIFETIME).ok();
                    }
                    // Show OTA progress on display if available
                    if *display_ok {
                        if let Some((done, total)) = ota.progress() {
                            let pct = (done as u32 * 100) / total as u32;
                            let mut line_buf = [0u8; 16];
                            let line = super::format_pct(&mut line_buf, pct);
                            display.clear(BinaryColor::Off).ok();
                            Text::with_baseline(line, Point::new(5, 64/2), text_style, Baseline::Middle)
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
                    rprintln!("RX #{} from={}: {}", rx_count, msg.source, text);
                    debug_println!(
                        "  len={} rssi={} raw={:?}",
                        msg.data.len(),
                        io.last_rssi(),
                        &msg.data[..],
                    );

                    // With the GPS logger the display is dedicated to the
                    // lat/long readout, so skip the received-message banner.
                    #[cfg(not(feature = "gps-radio-log"))]
                    if *display_ok {
                        const LEN: usize = 32;
                        let mut send_header: [u8; LEN] = *b"In:                             ";

                        let offset = 4;
                        let len = text.len().min(LEN-offset);

                        send_header[offset..offset+len].copy_from_slice(text.as_bytes());

                        display.clear(BinaryColor::Off).ok();
                        Text::with_baseline(core::str::from_utf8(&send_header).unwrap_or("UTF8 Error Receiving"), Point::new(5, 64/2), text_style, Baseline::Middle)
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

            // Send a heartbeat (broadcast).  With the `board` feature the
            // heartbeat carries the sensor readings instead of "hello".
            if Mono::now() >= next_tx {
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
                match mesh.broadcast(core::str::from_utf8(message).unwrap_or("UTF8 Message Error").as_bytes(), BROADCAST_LIFETIME) {
                    Ok(()) => rprintln!("TX #{}", tx_count),
                    Err(e) => rprintln!("TX #{} failed: {:?}", tx_count, e),
                }

                // With the GPS logger the display shows lat/long instead of
                // the transmitted-message banner.
                #[cfg(not(feature = "gps-radio-log"))]
                if *display_ok {
                    const LEN: usize = 32;
                    let mut send_header: [u8; LEN] = *b"Out:                            ";

                    let offset = 5;
                    let len = message.len().min(LEN-offset);

                    send_header[offset..offset+len].copy_from_slice(&message[..len]);

                    display.clear(BinaryColor::Off).ok();
                    Text::with_baseline(core::str::from_utf8(&send_header).unwrap_or("Error decoding"), Point::new(5, 64/2), text_style, Baseline::Middle)
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

            watchdog::feed(iwdg);
            Mono::delay(1_u32.millis()).await;
        }
    }
}
