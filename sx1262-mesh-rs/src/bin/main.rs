#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[unsafe(link_section = ".boot2")]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

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

#[rtic::app(device = rp2040_hal::pac)]
mod app {
    use embedded_hal_bus::spi::ExclusiveDevice;
    use rp2040_hal::clocks::init_clocks_and_plls;
    use rp2040_hal::gpio::bank0::*;
    use rp2040_hal::gpio::{FunctionSio, FunctionSpi, Pin, PullDown, PullNone, SioInput, SioOutput};
    use rp2040_hal::spi::Spi;
    use rp2040_hal::fugit::RateExtU32;
    use rp2040_hal::{Clock, Sio, Watchdog};

    use nano_mesh::{LoraIo, MeshNode};
    use sx1262_mesh_rs::config::{BROADCAST_LIFETIME, MESH_LISTEN_PERIOD_MS};
    use sx1262_mesh_rs::radio::Sx1262Driver;

    use super::{RF_FREQ, THIS_ADDRESS, XOSC_CRYSTAL_FREQ};

    /*
        RP2040 Pico - SX1262

        GP2   SCK   (SPI0)
        GP3   MOSI  (SPI0)
        GP4   MISO  (SPI0)
        GP5   NSS
        GP6   RST
        GP7   BUSY
        GP8   RF_SW / ANT
        GP9   DIO1
    */

    type SpiPins = (
        Pin<Gpio3, FunctionSpi, PullDown>,
        Pin<Gpio4, FunctionSpi, PullDown>,
        Pin<Gpio2, FunctionSpi, PullDown>,
    );
    type Spi0 = Spi<rp2040_hal::spi::Enabled, rp2040_hal::pac::SPI0, SpiPins, 8>;
    type CsPin = Pin<Gpio5, FunctionSio<SioOutput>, PullDown>;
    type SpiDev = ExclusiveDevice<Spi0, CsPin, embedded_hal_bus::spi::NoDelay>;
    type Radio = Sx1262Driver<
        SpiDev,
        Pin<Gpio6, FunctionSio<SioOutput>, PullDown>,
        Pin<Gpio7, FunctionSio<SioInput>, PullNone>,
        Pin<Gpio8, FunctionSio<SioOutput>, PullDown>,
        Pin<Gpio9, FunctionSio<SioInput>, PullDown>,
    >;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        mesh: MeshNode,
        io: LoraIo<Radio>,
        next_tx_ms: u32,
        tx_count: u32,
        rx_count: u32,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut pac = cx.device;

        // ---- Clocks ----------------------------------------------------------
        let mut watchdog = Watchdog::new(pac.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // ---- GPIO ------------------------------------------------------------
        let sio = Sio::new(pac.SIO);
        let pins = rp2040_hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        // ---- SPI + GPIO pin assignments --------------------------------------
        let sclk = pins.gpio2.into_function::<FunctionSpi>();
        let mosi = pins.gpio3.into_function::<FunctionSpi>();
        let miso = pins.gpio4.into_function::<FunctionSpi>();
        let cs = pins.gpio5.into_push_pull_output_in_state(rp2040_hal::gpio::PinState::High);
        let nrst = pins.gpio6.into_push_pull_output_in_state(rp2040_hal::gpio::PinState::High);
        let busy = pins.gpio7.into_floating_input();
        let ant = pins.gpio8.into_push_pull_output_in_state(rp2040_hal::gpio::PinState::High);
        let dio1 = pins.gpio9.into_pull_down_input();

        // ---- SPI bus ---------------------------------------------------------
        let spi = Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sclk));
        let spi = spi.init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
            8_000_000u32.Hz(),
            embedded_hal::spi::MODE_0,
        );
        let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

        // ---- SX1262 radio ----------------------------------------------------
        defmt::debug!("Initialising SX1262 radio...");
        let mut radio = Sx1262Driver::new(spi_device, nrst, busy, ant, dio1);
        radio.init(RF_FREQ);
        defmt::debug!("SX1262 init complete, checking hardware:");
        if !radio.print_diagnostics() {
            defmt::warn!("Radio not responding! Check wiring.");
        }

        // ---- Mesh networking -------------------------------------------------
        defmt::debug!("Starting nano-mesh (address={}, freq={} Hz)...", THIS_ADDRESS, RF_FREQ);
        let io = LoraIo::new(radio);
        let mesh = MeshNode::new(THIS_ADDRESS, MESH_LISTEN_PERIOD_MS);

        defmt::info!("Mesh node {} ready", THIS_ADDRESS);

        // Stagger first TX by address so nodes don't collide on boot
        let now = sx1262_mesh_rs::platform::millis();
        let next_tx_ms = now.wrapping_add(THIS_ADDRESS as u32 * 3000);

        (
            Shared {},
            Local {
                mesh,
                io,
                next_tx_ms,
                tx_count: 0,
                rx_count: 0,
            },
        )
    }

    #[idle(local = [mesh, io, next_tx_ms, tx_count, rx_count])]
    fn idle(cx: idle::Context) -> ! {
        let mesh = cx.local.mesh;
        let io = cx.local.io;
        let next_tx_ms = cx.local.next_tx_ms;
        let tx_count = cx.local.tx_count;
        let rx_count = cx.local.rx_count;

        loop {
            // Drive the mesh protocol (receive, forward, transmit)
            mesh.update(io, sx1262_mesh_rs::platform::millis());

            // Check for incoming messages
            if let Some(msg) = mesh.receive() {
                *rx_count += 1;
                let text = core::str::from_utf8(&msg.data).unwrap_or("<invalid utf8>");
                defmt::info!(
                    "RX #{} from={}: {}",
                    *rx_count,
                    msg.source,
                    text,
                );
            }

            // Send a heartbeat (broadcast)
            let now = sx1262_mesh_rs::platform::millis();
            if now.wrapping_sub(*next_tx_ms) < 0x8000_0000 {
                *tx_count += 1;
                match mesh.broadcast(b"hello", BROADCAST_LIFETIME) {
                    Ok(()) => defmt::info!("TX #{}", *tx_count),
                    Err(e) => defmt::info!("TX #{} failed: {}", *tx_count, defmt::Debug2Format(&e)),
                }
                // Schedule next TX with 0-3s jitter
                let jitter_ms = sx1262_mesh_rs::platform::random(0, 3000) as u32;
                *next_tx_ms = now.wrapping_add(10_000 + jitter_ms);
            }
        }
    }
}
