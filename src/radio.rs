//! SX1262 radio driver implementing [`PacketRadio`] via the STM32WLE5 SubGHz peripheral.

use crate::config::{TX_CHIP_TIMEOUT_MS, TX_POLL_TIMEOUT_MS};
use crate::platform;

/// A packet-oriented radio interface.
///
/// Implement this for your radio hardware to use it with [`crate::io::LoraIo`]
/// and the mesh layer.
pub trait PacketRadio {
    /// Error type for radio operations.
    type Error: core::fmt::Debug;

    /// Poll for a received packet (non-blocking).
    ///
    /// If a packet is available, write it into `buf` and return
    /// `Ok(Some((bytes_written, rssi_dbm)))`.
    /// If nothing is available, return `Ok(None)`.
    fn poll_recv(&mut self, buf: &mut [u8]) -> Result<Option<(usize, i16)>, Self::Error>;

    /// Transmit a raw packet. Blocks until transmission completes.
    fn send(&mut self, data: &[u8]) -> Result<(), Self::Error>;

    /// Maximum packet size in bytes.
    fn max_packet_len(&self) -> usize;
}
use stm32wlxx_hal::subghz::{
    CalibrateImage, CfgIrq, CodingRate, FallbackMode, HeaderType, Irq, LoRaBandwidth,
    LoRaModParams, LoRaPacketParams, LoRaSyncWord, Ocp, PaConfig, PaSel, PacketType, RampTime,
    RegMode, RfFreq, SpreadingFactor, StandbyClk, SubGhz, TcxoMode, TcxoTrim, Timeout, TxParams,
};
use stm32wlxx_hal::spi::{SgMiso, SgMosi};

/// Errors from the SubGHz radio.
#[derive(Debug)]
pub enum Sx1262Error {
    Radio,
    Timeout,
}

/// SubGHz radio driver that implements [`PacketRadio`].
///
/// On the STM32WLE5 the SX1262 is integrated — no external SPI bus or GPIO
/// pins are needed.  The [`SubGhz`] peripheral handles the internal SPI3
/// interface, BUSY signal, and DIO lines.
pub struct Sx1262Driver {
    radio: SubGhz<SgMiso, SgMosi>,
    rx_active: bool,
}

impl Sx1262Driver {
    /// Create a new SubGHz radio driver. Call [`init`](Self::init) before use.
    pub fn new(radio: SubGhz<SgMiso, SgMosi>) -> Self {
        Self {
            radio,
            rx_active: false,
        }
    }

    /// Initialise the radio with LoRa settings.
    ///
    /// `rf_frequency` is in Hz, e.g. `915_000_000` for 915 MHz.
    ///
    /// # Panics
    ///
    /// Panics if the radio fails to initialise.
    pub fn init(&mut self, rf_frequency: u32) {
        debug_println!("Initialising SubGHz radio...");
        // Reset the radio and enter standby
        self.radio
            .set_standby(StandbyClk::Rc)
            .expect("set_standby");

        // Use DCDC regulator for better efficiency
        self.radio.set_regulator_mode(RegMode::Smps).ok();

        // Configure TCXO: Wio-E5 has a 32 MHz TCXO on DIO3
        self.radio
            .set_tcxo_mode(
                &TcxoMode::new()
                    .set_txco_trim(TcxoTrim::Volts1pt8)
                    .set_timeout(Timeout::from_millis_sat(10)),
            )
            .expect("set_tcxo_mode");

        // Calibrate image for 915 MHz band (902–928 MHz)
        self.radio
            .calibrate_image(CalibrateImage::ISM_902_928)
            .expect("calibrate_image");

        // Set packet type to LoRa
        self.radio
            .set_packet_type(PacketType::LoRa)
            .expect("set_packet_type");

        // Set RF frequency
        self.radio
            .set_rf_frequency(&RfFreq::from_frequency(rf_frequency))
            .expect("set_rf_frequency");

        // PA config: +22 dBm high-power PA
        self.radio
            .set_pa_config(
                &PaConfig::new()
                    .set_pa_duty_cycle(0x04)
                    .set_hp_max(0x07)
                    .set_pa(PaSel::Hp),
            )
            .expect("set_pa_config");

        // TX params: 22 dBm, 200 µs ramp
        self.radio
            .set_tx_params(
                &TxParams::new()
                    .set_power(0x16)
                    .set_ramp_time(RampTime::Micros200),
            )
            .expect("set_tx_params");

        // LoRa modulation: SF7, BW125, CR4/5
        self.radio
            .set_lora_mod_params(
                &LoRaModParams::new()
                    .set_sf(SpreadingFactor::Sf7)
                    .set_bw(LoRaBandwidth::Bw125)
                    .set_cr(CodingRate::Cr45)
                    .set_ldro_en(false),
            )
            .expect("set_lora_mod_params");

        // LoRa packet params: 8-sym preamble, variable header, 255-byte max
        self.radio
            .set_lora_packet_params(
                &LoRaPacketParams::new()
                    .set_preamble_len(8)
                    .set_header_type(HeaderType::Variable)
                    .set_payload_len(255)
                    .set_crc_en(true)
                    .set_invert_iq(false),
            )
            .expect("set_lora_packet_params");

        // LoRa sync word: public network (0x3444)
        self.radio
            .set_lora_sync_word(LoRaSyncWord::Public)
            .expect("set_lora_sync_word");

        // Buffer base addresses: TX at 0x00, RX at 0x00
        self.radio
            .set_buffer_base_address(0x00, 0x00)
            .expect("set_buffer_base_address");

        // IRQ: route RxDone, TxDone, Timeout to all lines
        self.radio
            .set_irq_cfg(
                &CfgIrq::new()
                    .irq_enable_all(Irq::RxDone)
                    .irq_enable_all(Irq::TxDone)
                    .irq_enable_all(Irq::Timeout),
            )
            .expect("set_irq_cfg");

        // Set fallback mode to standby after TX/RX
        self.radio
            .set_tx_rx_fallback_mode(FallbackMode::Standby)
            .ok();

        // Over-current protection
        self.radio.set_pa_ocp(Ocp::Max140m).ok();
        debug_println!("SubGHz init complete.");
    }

    /// Print radio diagnostics. Returns `true` if the radio responds.
    pub fn print_diagnostics(&mut self) -> bool {
        debug_println!("Checking radio hardware:");
        match self.radio.status() {
            Ok(s) => {
                debug_println!("  Status: {:?}", s);
                true
            }
            Err(_) => {
                rtt_target::rprintln!("WARNING: Radio not responding!");
                false
            }
        }
    }

    /// Poll the RFBUSYS bit to wait for the radio to be ready.
    fn wait_on_busy(&self) {
        // On STM32WLE5 the BUSY signal is exposed as RFBUSYS in PWR->SR2.
        // The HAL's SPI transactions poll this internally, but we call it
        // explicitly after set_tx/set_rx for safety.
        while unsafe {
            (*stm32wlxx_hal::pac::PWR::ptr())
                .sr2
                .read()
                .rfbusys()
                .bit_is_set()
        } {}
    }
}

impl PacketRadio for Sx1262Driver {
    type Error = Sx1262Error;

    fn poll_recv(&mut self, buf: &mut [u8]) -> Result<Option<(usize, i16)>, Self::Error> {
        // Enter continuous RX if not already listening
        if !self.rx_active {
            self.radio
                .set_rx(Timeout::DISABLED)
                .map_err(|_| Sx1262Error::Radio)?;
            self.wait_on_busy();
            self.rx_active = true;
        }

        // Poll IRQ status
        let (_, irq) = self.radio.irq_status().map_err(|_| Sx1262Error::Radio)?;

        if irq & Irq::RxDone.mask() == 0 {
            return Ok(None);
        }

        // Clear all pending IRQs
        let _ = self.radio.clear_irq_status(0xFFFF);

        // rx_buffer_status returns (Status, payload_len, rx_start_ptr)
        let (_, len_u8, offset) = self
            .radio
            .rx_buffer_status()
            .map_err(|_| Sx1262Error::Radio)?;
        let len = len_u8 as usize;

        if len > buf.len() {
            self.rx_active = false;
            return Ok(None);
        }

        self.radio
            .read_buffer(offset, &mut buf[..len])
            .map_err(|_| Sx1262Error::Radio)?;

        let pkt_status = self
            .radio
            .lora_packet_status()
            .map_err(|_| Sx1262Error::Radio)?;
        // rssi_pkt() returns Ratio<i16>; .to_integer() gives dBm
        let rssi = pkt_status.rssi_pkt().to_integer();

        // Stay in RX — continuous mode persists
        Ok(Some((len, rssi)))
    }

    fn send(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.rx_active = false;

        // Standby before TX
        self.radio
            .set_standby(StandbyClk::Rc)
            .map_err(|_| Sx1262Error::Radio)?;
        self.wait_on_busy();

        // Clear any pending IRQs
        let _ = self.radio.clear_irq_status(0xFFFF);

        // Write data to buffer
        self.radio
            .write_buffer(0x00, data)
            .map_err(|_| Sx1262Error::Radio)?;

        // Set packet params with actual payload length
        self.radio
            .set_lora_packet_params(
                &LoRaPacketParams::new()
                    .set_preamble_len(8)
                    .set_header_type(HeaderType::Variable)
                    .set_payload_len(data.len() as u8)
                    .set_crc_en(true)
                    .set_invert_iq(false),
            )
            .map_err(|_| Sx1262Error::Radio)?;

        // Start TX with chip timeout
        self.radio
            .set_tx(Timeout::from_millis_sat(TX_CHIP_TIMEOUT_MS as u32))
            .map_err(|_| Sx1262Error::Radio)?;
        self.wait_on_busy();

        if cfg!(feature = "debug") && let Ok(status) = self.radio.status() {
                debug_println!("  send: TX started, chip status = {:?}", status);
        }
        

        // Poll IRQ for TxDone/Timeout
        let start_ms = platform::millis();
        let result = loop {
            let elapsed = platform::millis().wrapping_sub(start_ms) as u64;
            if elapsed > TX_POLL_TIMEOUT_MS {
                debug_println!(
                    "  TX timeout (no TxDone IRQ after {}ms)",
                    TX_POLL_TIMEOUT_MS
                );
                let _ = self.radio.clear_irq_status(0xFFFF);
                break Err(Sx1262Error::Timeout);
            }
            if let Ok((_, irq)) = self.radio.irq_status() {
                let tx_done = irq & Irq::TxDone.mask() != 0;
                let timeout = irq & Irq::Timeout.mask() != 0;
                if tx_done || timeout {
                    let _ = self.radio.clear_irq_status(0xFFFF);
                    break if tx_done { Ok(()) } else { Err(Sx1262Error::Timeout) };
                }
            }
        };

        // Re-enter continuous RX immediately after TX
        if self.radio.set_rx(Timeout::DISABLED).is_ok() {
            self.wait_on_busy();
            self.rx_active = true;
        }

        result
    }

    fn max_packet_len(&self) -> usize {
        255
    }
}
