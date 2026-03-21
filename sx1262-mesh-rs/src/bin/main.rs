#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::main;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode;
use esp_hal::delay::Delay;
use esp_hal::time::{Duration, Instant, Rate};

use embedded_hal_bus::spi::ExclusiveDevice;
#[macro_use]
extern crate sx1262_mesh_rs;
use sx1262_mesh_rs::config::MESH_LISTEN_PERIOD_MS;
use sx1262_mesh_rs::radio::Sx1262Driver;

use nano_mesh::{LoraIo, MeshNode};

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

/// This node's mesh address.
/// Set at compile time via the `ADDRESS` environment variable, e.g.:
///   ADDRESS=2 cargo run --release
/// Defaults to 1 if not specified.
const THIS_ADDRESS: u8 = {
    // option_env! reads the variable at compile time; it cannot fail at runtime.
    match option_env!("ADDRESS") {
        Some(s) => {
            let bytes = s.as_bytes();
            assert!(bytes.len() > 0, "ADDRESS must not be empty");
            let mut i = 0;
            let mut n: u8 = 0;
            while i < bytes.len() {
                let d = bytes[i];
                assert!(d >= b'0' && d <= b'9', "ADDRESS must be a number 0-255");
                // Manual overflow check for const context
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

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]

/*
    xiao ESP32c3 - sx1262

    XL       SX              XR         SX
    =========================================
    GPIO 2   D0      |       5V
    GPIO 3   DIO1    |       GND
    GPIO 4   RST     |       3V3
    GPIO 5   BUSY    |       GPIO 10    MOSI
    GPIO 6   NSS     |       GPIO 9     MISO
    GPIO 7   RF_SW   |       GPIO 8     SCK
    GPIO 21  D6      |       GPIO 20    D7
    =========================================

*/
#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // ---- SPI + GPIO pin assignments ------------------------------------
    // Adjust these to match your board's wiring.
    let sclk = peripherals.GPIO8;
    let miso = peripherals.GPIO9;
    let mosi = peripherals.GPIO10;
    let cs = Output::new(peripherals.GPIO6, Level::High, OutputConfig::default());
    let nrst = Output::new(peripherals.GPIO4, Level::High, OutputConfig::default());
    let busy = Input::new(peripherals.GPIO5, InputConfig::default());
    let ant = Output::new(peripherals.GPIO7, Level::High, OutputConfig::default());
    let dio1 = Input::new(peripherals.GPIO3, InputConfig::default().with_pull(Pull::Down));

    // ---- SPI bus -------------------------------------------------------
    let spi_bus = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(8))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_miso(miso)
    .with_mosi(mosi);

    let spi_device = ExclusiveDevice::new(spi_bus, cs, Delay::new()).unwrap();

    // ---- SX1262 radio --------------------------------------------------
    debug_println!("Initialising SX1262 radio...");
    let mut radio = Sx1262Driver::new(spi_device, nrst, busy, ant, dio1);
    radio.init(RF_FREQ);
    debug_println!("SX1262 init complete, checking hardware:");
    if !radio.print_diagnostics() {
        esp_println::println!("WARNING: Radio not responding! Check wiring.");
    }

    // ---- Mesh networking -----------------------------------------------
    debug_println!("Starting nano-mesh (address={}, freq={} Hz)...", THIS_ADDRESS, RF_FREQ);
    let mut io = LoraIo::new(radio);
    let mut mesh = MeshNode::new(THIS_ADDRESS, MESH_LISTEN_PERIOD_MS);

    esp_println::println!("Mesh node {} ready", THIS_ADDRESS);

    // ---- Main loop -----------------------------------------------------
    // Stagger first TX by address so nodes don't collide on boot
    let tx_interval = Duration::from_secs(10);
    let mut next_tx = Instant::now() + Duration::from_secs(THIS_ADDRESS as u64 * 3);
    let mut tx_count: u32 = 0;
    let mut rx_count: u32 = 0;

    loop {
        // Drive the mesh protocol (receive, forward, transmit)
        mesh.update(&mut io, sx1262_mesh_rs::platform::millis());

        // Check for incoming messages
        if let Some(msg) = mesh.receive() {
            rx_count += 1;
            let text = core::str::from_utf8(&msg.data).unwrap_or("<invalid utf8>");
            esp_println::println!(
                "RX #{} from={}: {}",
                rx_count,
                msg.source,
                text,
            );
            debug_println!(
                "  len={} rssi={} raw={:?}",
                msg.data.len(),
                io.last_rssi(),
                &msg.data[..],
            );
        }

        // Send a heartbeat (broadcast)
        if Instant::now() > next_tx {
            tx_count += 1;
            match mesh.broadcast(b"hello", 3) {
                Ok(()) => esp_println::println!("TX #{}", tx_count),
                Err(e) => esp_println::println!("TX #{} failed: {:?}", tx_count, e),
            }
            // Schedule next TX from now, with 0-3s jitter to avoid repeated collisions
            let jitter_ms = sx1262_mesh_rs::platform::random(0, 3000) as u64;
            next_tx = Instant::now() + tx_interval + Duration::from_millis(jitter_ms);
        }
    }
}
