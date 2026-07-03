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

        (Shared {}, Local { io, mesh, display, display_ok, flash: flash_periph, ota, iwdg, board, sensors, rs422 })
    }

    #[task(local = [io, mesh, display, display_ok, flash, ota, iwdg, board, sensors, rs422], priority = 1)]
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
                }
                #[cfg(feature = "rs422")]
                match rs422.read_soil() {
                    Ok(soil) => rprintln!(
                        "Soil: {}.{} % {}.{} C",
                        soil.moisture_pct_x10 / 10,
                        soil.moisture_pct_x10 % 10,
                        soil.temp_c_x10 / 10,
                        (soil.temp_c_x10 % 10).unsigned_abs()
                    ),
                    Err(e) => debug_println!("Soil probe: {:?}", e),
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

                    const LEN: usize = 32;
                    let mut send_header: [u8; LEN] = *b"In:                             ";

                    let offset = 4;
                    let len = text.len().min(LEN-offset);

                    send_header[offset..offset+len].copy_from_slice(text.as_bytes());

                    if *display_ok {
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
                const LEN: usize = 32;
                let mut send_header: [u8; LEN] = *b"Out:                            ";
                
                let offset = 5;
                let len = message.len().min(LEN-offset);

                send_header[offset..offset+len].copy_from_slice(&message[..len]);

                tx_count += 1;
                match mesh.broadcast(core::str::from_utf8(message).unwrap_or("UTF8 Message Error").as_bytes(), BROADCAST_LIFETIME) {
                    Ok(()) => rprintln!("TX #{}", tx_count),
                    Err(e) => rprintln!("TX #{} failed: {:?}", tx_count, e),
                }

                if *display_ok {
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
