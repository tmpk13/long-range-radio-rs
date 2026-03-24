#![no_std]
#![no_main]
#![deny(clippy::large_stack_frames)]

use panic_halt as _;

#[macro_use]
extern crate sx1262_mesh_rs;

/// RF frequency in Hz (915 MHz ISM band).
const RF_FREQ: u32 = 915_000_000;

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
    use core::str;

    use rtic_monotonics::systick::prelude::*;
    systick_monotonic!(Mono, 1000);

    use embedded_graphics::{
        mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10, iso_8859_13::FONT_10X20},
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };
    use sx1262_mesh_rs::{LoraIo, MeshMessage, MeshNode};
    use rtt_target::{rprintln, rtt_init, set_print_channel, DownChannel};
    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};
    use stm32wlxx_hal::{
        gpio::{pins, PortA, PortB},
        i2c::I2c2,
        subghz::SubGhz,
    };
    use sx1262_mesh_rs::config::{BROADCAST_LIFETIME, MESH_LISTEN_PERIOD_MS, THIS_ADDRESS};
    use sx1262_mesh_rs::platform::SYSCLK_HZ;
    use sx1262_mesh_rs::radio::Sx1262Driver;

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
        term_in: DownChannel,
        input_buf: [u8; 64],
        input_len: usize,
    }

    #[init]
    #[allow(clippy::large_stack_frames)]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        let channels = rtt_init! {
            up: {
                0: { size: 1024, name: "Terminal" }
            }
            down: {
                0: { size: 256, name: "Terminal" }
            }
        };
        set_print_channel(channels.up.0);
        let term_in = channels.down.0;

        // Enable DWT cycle counter for millis()/random()
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        // Start SysTick monotonic at default MSI 4 MHz
        Mono::start(cx.core.SYST, SYSCLK_HZ);

        let dp = cx.device;

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

        rprintln!("Mesh node {} ready", THIS_ADDRESS);

        run::spawn().unwrap();

        (Shared {}, Local { io, mesh, display, term_in, input_buf: [0u8; 64], input_len: 0 })
    }

    #[task(local = [io, mesh, display, term_in, input_buf, input_len], priority = 1)]
    async fn run(cx: run::Context) {
        let io = cx.local.io;
        let mesh = cx.local.mesh;
        let display = cx.local.display;

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
            // Poll RTT terminal input and broadcast on newline
            #[cfg(feature = "terminal")]
            {
                let mut rbuf = [0u8; 16];
                let n = cx.local.term_in.read(&mut rbuf);
                for &b in &rbuf[..n] {
                    if b == b'\n' || b == b'\r' {
                        if *cx.local.input_len > 0 {
                            let text = core::str::from_utf8(&cx.local.input_buf[..*cx.local.input_len])
                                .unwrap_or("<utf8 error>");
                            match mesh.broadcast(text.as_bytes(), BROADCAST_LIFETIME) {
                                Ok(()) => rprintln!("TX terminal: {}", text),
                                Err(e) => rprintln!("TX terminal failed: {:?}", e),
                            }
                            *cx.local.input_len = 0;
                        }
                    } else if *cx.local.input_len < cx.local.input_buf.len() {
                        cx.local.input_buf[*cx.local.input_len] = b;
                        *cx.local.input_len += 1;
                    }
                }
            }

            // Drive the mesh protocol (receive, forward, transmit)
            mesh.update(io, sx1262_mesh_rs::platform::millis());

            // Check for incoming messages
            if let Some(msg) = mesh.receive() {
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

            Mono::delay(1_u32.millis()).await;
        }
    }
}
