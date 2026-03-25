#![no_std]
#![no_main]
#![warn(clippy::large_stack_frames)] // warn, not deny — RTIC macro generates large closures we can't annotate

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

    type Radio = Sx1262Driver;
    type Display = Ssd1306<
        I2CInterface<I2c2<(pins::B15, pins::A15)>>,
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
        flash: FLASH,
        ota: OtaReceiver,
        iwdg: IWDG,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        let channels = rtt_init! {
            up: {
                0: { size: 1024, name: "Terminal" }
            }
        };
        set_print_channel(channels.up.0);

        // Enable DWT cycle counter for millis()/random()
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        // Start SysTick monotonic at default MSI 4 MHz
        Mono::start(cx.core.SYST, SYSCLK_HZ);

        let dp = cx.device;
        let mut flash_periph = dp.FLASH;

        // Start watchdog (5 s timeout). If the app never reaches confirm_boot
        // or hangs during init, the MCU resets and the bootloader reverts.
        let iwdg = dp.IWDG;
        watchdog::start(&iwdg, 5_000);

        // Confirm boot to the bootloader (marks firmware as healthy).
        sx1262_mesh_rs::boot_state::confirm_boot(&mut flash_periph);

        // ---- SubGHz radio (integrated SX1262) --------------------------------
        let mut rcc = dp.RCC;
        let sg = SubGhz::new(dp.SPI3, &mut rcc);
        let mut radio = Sx1262Driver::new(sg);
        radio.init(super::RF_FREQ);
        radio.print_diagnostics();

        // ---- I2C2 display (SSD1306 128x64) -----------------------------------
        let gpioa = PortA::split(dp.GPIOA, &mut rcc);
        let gpiob = PortB::split(dp.GPIOB, &mut rcc);
        let i2c = cortex_m::interrupt::free(|cs| {
            I2c2::new(dp.I2C2, (gpiob.b15, gpioa.a15), 100_000, &mut rcc, false, cs)
        });
        let mut display = Ssd1306::new(
            I2CDisplayInterface::new(i2c),
            DisplaySize128x64,
            DisplayRotation::Rotate0,
        )
        .into_buffered_graphics_mode();
        display.init().ok();
        display.clear(BinaryColor::Off).ok();
        display.flush().ok();

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

        (Shared {}, Local { io, mesh, display, flash: flash_periph, ota, iwdg })
    }

    #[task(local = [io, mesh, display, flash, ota, iwdg], priority = 1)]
    async fn run(cx: run::Context) {
        let io = cx.local.io;
        let mesh = cx.local.mesh;
        let display = cx.local.display;
        let flash = cx.local.flash;
        let ota = cx.local.ota;
        let iwdg = cx.local.iwdg;

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(BinaryColor::On)
            .build();

        // Stagger first TX by address so nodes don't collide on boot
        let tx_interval = 10_000_u32.millis();
        let mut next_tx = Mono::now() + (THIS_ADDRESS as u32 * 3_000).millis();
        let mut tx_count: u32 = 0;
        let mut rx_count: u32 = 0;

        loop {
            // Drive the mesh protocol (receive, forward, transmit)
            mesh.update(io, sx1262_mesh_rs::platform::millis());

            // Check for incoming messages
            if let Some(msg) = mesh.receive() {
                if ota_protocol::is_ota_message(&msg.data) {
                    // Route to OTA handler
                    if let Some(response) = ota.handle_message(&msg.data, flash) {
                        mesh.send(&response.data[..response.len], msg.source, BROADCAST_LIFETIME).ok();
                    }
                    // Show OTA progress on display if active
                    if let Some((done, total)) = ota.progress() {
                        let pct = (done as u32 * 100) / total as u32;
                        // Format "OTA XX%" into a fixed buffer
                        let mut line_buf = [0u8; 16];
                        let line = super::format_pct(&mut line_buf, pct);
                        display.clear(BinaryColor::Off).ok();
                        Text::with_baseline(line, Point::new(5, 64/2), text_style, Baseline::Middle)
                            .draw(display)
                            .ok();
                        display.flush().ok();
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

                    display.clear(BinaryColor::Off).ok();
                    Text::with_baseline(core::str::from_utf8(&send_header).unwrap_or("UTF8 Error Receiving"), Point::new(5, 64/2), text_style, Baseline::Middle)
                        .draw(display)
                        .ok();
                    display.flush().ok();
                }
            }

            // Send a heartbeat (broadcast)
            if Mono::now() >= next_tx {
                let message = b"hello";
                const LEN: usize = 32;
                let mut send_header: [u8; LEN] = *b"Out:                            ";
                
                let offset = 5;
                let len = message.len().min(LEN-offset);

                send_header[offset..offset+len].copy_from_slice(message);

                tx_count += 1;
                match mesh.broadcast(core::str::from_utf8(message).unwrap_or("UTF8 Message Error").as_bytes(), BROADCAST_LIFETIME) {
                    Ok(()) => rprintln!("TX #{}", tx_count),
                    Err(e) => rprintln!("TX #{} failed: {:?}", tx_count, e),
                }

                display.clear(BinaryColor::Off).ok();
                Text::with_baseline(core::str::from_utf8(&send_header).unwrap_or("Error decoding"), Point::new(5, 64/2), text_style, Baseline::Middle)
                    .draw(display)
                    .ok();
                display.flush().ok();

                // Schedule next TX with 0-3s jitter
                let jitter_ms = sx1262_mesh_rs::platform::random(0, 3000) as u32;
                next_tx = Mono::now() + tx_interval + jitter_ms.millis();
            }

            watchdog::feed(iwdg);
            Mono::delay(1_u32.millis()).await;
        }
    }
}
