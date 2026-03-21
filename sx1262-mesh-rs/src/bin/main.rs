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
// Ensure platform functions (rh_millis, rh_delay, rh_random) are linked.
extern crate sx1262_mesh_rs;
use sx1262_mesh_rs::radio::Sx1262Driver;

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

/// This node's mesh address (change per device).
const THIS_ADDRESS: u8 = 1;
/// RF frequency in Hz (915 MHz ISM band).
const RF_FREQ: u32 = 915_000_000;

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]

/* 
xiao ESP32c3 - sx1262

L             R
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
    let sclk = peripherals.GPIO6;
    let miso = peripherals.GPIO5;
    let mosi = peripherals.GPIO7;
    let cs = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());
    let nrst = Output::new(peripherals.GPIO10, Level::High, OutputConfig::default());
    let busy = Input::new(peripherals.GPIO3, InputConfig::default());
    let ant = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());
    let dio1 = Input::new(peripherals.GPIO4, InputConfig::default().with_pull(Pull::Down));

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
    let mut radio = Sx1262Driver::new(spi_device, nrst, busy, ant, dio1);
    radio.init(RF_FREQ);

    // ---- RadioHead mesh ------------------------------------------------
    let mut mesh = unsafe { radiohead::RhMesh::new(&mut radio, THIS_ADDRESS) };
    mesh.init();

    // Optional: tweak retransmit behaviour
    mesh.set_timeout(200);
    mesh.set_retries(3);

    esp_println::println!("Mesh node {} ready", THIS_ADDRESS);

    // ---- Main loop -----------------------------------------------------
    let mut rx_buf = [0u8; 64];
    let mut tick = Instant::now();

    loop {
        // Poll for incoming messages (also handles routing for other nodes)
        if let Some(msg) = mesh.recv(&mut rx_buf) {
            let payload = &rx_buf[..msg.len as usize];
            esp_println::println!(
                "RX from={} dest={} len={}: {:?}",
                msg.source,
                msg.dest,
                msg.len,
                payload,
            );
        }

        // Send a heartbeat every 10 seconds (to broadcast address 0xFF)
        if tick.elapsed() > Duration::from_secs(10) {
            tick = Instant::now();
            let msg = b"hello";
            match mesh.send(msg, 0xFF) {
                Ok(()) => esp_println::println!("TX ok"),
                Err(e) => esp_println::println!("TX err: {:?}", e),
            }
        }
    }
}
