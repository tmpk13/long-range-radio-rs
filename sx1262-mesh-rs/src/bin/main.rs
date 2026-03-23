#![no_std]
#![no_main]
#![deny(clippy::large_stack_frames)]

use panic_halt as _;

#[macro_use]
extern crate sx1262_mesh_rs;

/// This node's mesh address.
/// Set at compile time via the `ADDRESS` environment variable, e.g.:
///   ADDRESS=2 cargo run --release
/// Defaults to 1 if not specified.
const THIS_ADDRESS: u8 = {
    match option_env!("ADDRESS") {
        Some(s) => {
            let bytes = s.as_bytes();
            assert!(bytes.len() > 0, "ADDRESS must not be empty");
            let mut i = 0;
            let mut n: u8 = 0;
            while i < bytes.len() {
                let d = bytes[i];
                assert!(d >= b'0' && d <= b'9', "ADDRESS must be a number 0-255");
                let next = n as u16 * 10 + (d - b'0') as u16;
                assert!(next <= 255, "ADDRESS must be 0-255");
                n = next as u8;
                i += 1;
            }
            n
        }
        None => 1,
    }
};

/// RF frequency in Hz (915 MHz ISM band).
const RF_FREQ: u32 = 915_000_000;

/*
    Seeed Wio-E5 (LoRa-E5) — STM32WLE5JC

    The SX1262 radio is integrated into the MCU.  There are no external
    SPI or GPIO connections for the radio — the SubGHz peripheral handles
    everything over an internal SPI3 bus.

    TCXO:  32 MHz on DIO3 (configured via SubGHz command)
    RF SW: controlled via DIO2 (set_dio2_as_rf_switch_ctrl)

    Debug output via RTT (probe-rs / SWD).
*/

#[rtic::app(device = stm32wlxx_hal::pac, dispatchers = [SPI1])]
mod app {
    use rtic_monotonics::systick::prelude::*;
    systick_monotonic!(Mono, 1000);

    use nano_mesh::{LoraIo, MeshNode};
    use rtt_target::{rprintln, rtt_init_print};
    use stm32wlxx_hal::subghz::SubGhz;
    use sx1262_mesh_rs::config::{BROADCAST_LIFETIME, MESH_LISTEN_PERIOD_MS};
    use sx1262_mesh_rs::platform::SYSCLK_HZ;
    use sx1262_mesh_rs::radio::Sx1262Driver;

    type Radio = Sx1262Driver;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        io: LoraIo<Radio>,
        mesh: MeshNode,
    }

    #[init]
    #[allow(clippy::large_stack_frames)]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        rtt_init_print!();

        // Enable DWT cycle counter for millis()/random()
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        // Start SysTick monotonic at default MSI 4 MHz
        Mono::start(cx.core.SYST, SYSCLK_HZ);

        let dp = cx.device;

        // ---- SubGHz radio (integrated SX1262) --------------------------------
        debug_println!("Initialising SubGHz radio...");
        let mut rcc = dp.RCC;
        let sg = SubGhz::new(dp.SPI3, &mut rcc);
        let mut radio = Sx1262Driver::new(sg);
        radio.init(super::RF_FREQ);
        debug_println!("SubGHz init complete, checking hardware:");
        if !radio.print_diagnostics() {
            rprintln!("WARNING: Radio not responding!");
        }

        // ---- Mesh networking -------------------------------------------------
        debug_println!(
            "Starting nano-mesh (address={}, freq={} Hz)...",
            super::THIS_ADDRESS,
            super::RF_FREQ
        );
        let io = LoraIo::new(radio);
        let mesh = MeshNode::new(super::THIS_ADDRESS, MESH_LISTEN_PERIOD_MS);

        rprintln!("Mesh node {} ready", super::THIS_ADDRESS);

        run::spawn().unwrap();

        (Shared {}, Local { io, mesh })
    }

    #[task(local = [io, mesh], priority = 1)]
    async fn run(cx: run::Context) {
        let io = cx.local.io;
        let mesh = cx.local.mesh;

        // Stagger first TX by address so nodes don't collide on boot
        let tx_interval = 10_000_u32.millis();
        let mut next_tx = Mono::now() + (super::THIS_ADDRESS as u32 * 3_000).millis();
        let mut tx_count: u32 = 0;
        let mut rx_count: u32 = 0;

        loop {
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
            }

            // Send a heartbeat (broadcast)
            if Mono::now() >= next_tx {
                tx_count += 1;
                match mesh.broadcast(b"hello", BROADCAST_LIFETIME) {
                    Ok(()) => rprintln!("TX #{}", tx_count),
                    Err(e) => rprintln!("TX #{} failed: {:?}", tx_count, e),
                }
                // Schedule next TX with 0-3s jitter
                let jitter_ms = sx1262_mesh_rs::platform::random(0, 3000) as u32;
                next_tx = Mono::now() + tx_interval + jitter_ms.millis();
            }

            Mono::delay(1_u32.millis()).await;
        }
    }
}
