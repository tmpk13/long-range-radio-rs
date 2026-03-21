//! SX1262 radio driver implementing [`PacketRadio`].

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;
use nano_mesh::PacketRadio;
use sx126x::conf::Config;
use sx126x::op::*;
use sx126x::SX126x;

/// Errors from the SX1262 radio.
#[derive(Debug)]
pub enum Sx1262Error {
    Spi,
    Pin,
    Timeout,
}

/// Wrapper around `sx126x::SX126x` that implements [`PacketRadio`].
pub struct Sx1262Driver<SPI: SpiDevice, NRST, BUSY, ANT, DIO1> {
    radio: SX126x<SPI, NRST, BUSY, ANT, DIO1>,
    rx_active: bool,
}

impl<SPI, NRST, BUSY, ANT, DIO1, SPIERR, PINERR>
    Sx1262Driver<SPI, NRST, BUSY, ANT, DIO1>
where
    PINERR: core::fmt::Debug,
    SPIERR: core::fmt::Debug,
    SPI: SpiDevice<Error = SPIERR>,
    NRST: OutputPin<Error = PINERR>,
    BUSY: InputPin<Error = PINERR>,
    ANT: OutputPin<Error = PINERR>,
    DIO1: InputPin<Error = PINERR>,
{
    /// Create a new SX1262 driver. Call [`init`](Self::init) before use.
    pub fn new(spi: SPI, nrst: NRST, busy: BUSY, ant: ANT, dio1: DIO1) -> Self {
        Self {
            radio: SX126x::new(spi, (nrst, busy, ant, dio1)),
            rx_active: false,
        }
    }

    /// Initialise the SX1262 radio with LoRa settings.
    ///
    /// `rf_frequency` is in Hz, e.g. `915_000_000` for 915 MHz.
    ///
    /// # Panics
    ///
    /// Panics if the SX1262 hardware fails to initialise.
    pub fn init(&mut self, rf_frequency: u32) {
        let mod_params: ModParams = LoraModParams::default()
            .set_spread_factor(LoRaSpreadFactor::SF7)
            .set_bandwidth(LoRaBandWidth::BW125)
            .set_coding_rate(LoraCodingRate::CR4_5)
            .set_low_dr_opt(false)
            .into();

        let packet_params: PacketParams = LoRaPacketParams::default()
            .set_preamble_len(8)
            .set_header_type(LoRaHeaderType::VarLen)
            .set_payload_len(255)
            .set_crc_type(LoRaCrcType::CrcOn)
            .set_invert_iq(LoRaInvertIq::Standard)
            .into();

        let pa_config = PaConfig::default()
            .set_pa_duty_cycle(0x04)
            .set_hp_max(0x07)
            .set_device_sel(DeviceSel::SX1262);

        let tx_params = TxParams::default()
            .set_power_dbm(22)
            .set_ramp_time(RampTime::Ramp200u);

        let conf = Config {
            packet_type: PacketType::LoRa,
            sync_word: 0x3444, // public network
            calib_param: CalibParam::all(),
            mod_params,
            pa_config,
            packet_params: Some(packet_params),
            tx_params,
            dio1_irq_mask: IrqMask::none()
                .combine(IrqMaskBit::RxDone)
                .combine(IrqMaskBit::TxDone)
                .combine(IrqMaskBit::Timeout),
            dio2_irq_mask: IrqMask::none(),
            dio3_irq_mask: IrqMask::none(),
            rf_freq: sx126x::calc_rf_freq(rf_frequency as f32, 32_000_000.0),
            rf_frequency,
            tcxo_opts: Some((TcxoVoltage::Volt1_8, TcxoDelay::from_ms(10))),
        };

        self.radio.init(conf).expect("SX1262 init failed");
    }

    /// Print radio diagnostics to confirm the SX1262 is connected and healthy.
    /// Returns `true` if the radio responded, `false` if SPI communication failed.
    pub fn print_diagnostics(&mut self) -> bool {
        let status = match self.radio.get_status() {
            Ok(s) => s,
            Err(e) => {
                esp_println::println!("  RADIO NOT DETECTED - SPI error: {:?}", e);
                return false;
            }
        };
        debug_println!("  Status: {:?}", status);
        match self.radio.get_device_errors() {
            Ok(errors) => debug_println!("  Errors: {:?}", errors),
            Err(e) => debug_println!("  Could not read errors: {:?}", e),
        }
        true
    }
}

impl<SPI, NRST, BUSY, ANT, DIO1, SPIERR, PINERR> PacketRadio
    for Sx1262Driver<SPI, NRST, BUSY, ANT, DIO1>
where
    PINERR: core::fmt::Debug,
    SPIERR: core::fmt::Debug,
    SPI: SpiDevice<Error = SPIERR>,
    NRST: OutputPin<Error = PINERR>,
    BUSY: InputPin<Error = PINERR>,
    ANT: OutputPin<Error = PINERR>,
    DIO1: InputPin<Error = PINERR>,
{
    type Error = Sx1262Error;

    fn poll_recv(&mut self, buf: &mut [u8]) -> Result<Option<(usize, i16)>, Self::Error> {
        // Enter continuous RX if not already listening
        if !self.rx_active {
            self.radio
                .set_rx(RxTxTimeout::continuous_rx())
                .map_err(|_| Sx1262Error::Spi)?;
            self.rx_active = true;
        }

        // Poll IRQ status over SPI
        let irq = self.radio.get_irq_status().map_err(|_| Sx1262Error::Spi)?;

        if !irq.rx_done() {
            return Ok(None);
        }

        // Clear all pending IRQs
        let _ = self.radio.clear_irq_status(IrqMask::all());

        let rx_status = self
            .radio
            .get_rx_buffer_status()
            .map_err(|_| Sx1262Error::Spi)?;
        let len = rx_status.payload_length_rx() as usize;
        let offset = rx_status.rx_start_buffer_pointer();

        if len > buf.len() {
            self.rx_active = false;
            return Ok(None);
        }

        self.radio
            .read_buffer(offset, &mut buf[..len])
            .map_err(|_| Sx1262Error::Spi)?;

        let pkt_status = self
            .radio
            .get_packet_status()
            .map_err(|_| Sx1262Error::Spi)?;
        let rssi = pkt_status.rssi_pkt() as i16;

        // Stay in RX — continuous mode persists
        Ok(Some((len, rssi)))
    }

    fn send(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.rx_active = false;

        // Must go to standby before TX (especially when coming from continuous RX)
        self.radio
            .set_standby(StandbyConfig::StbyRc)
            .map_err(|_| Sx1262Error::Spi)?;

        // Clear any pending IRQs from RX mode
        let _ = self.radio.clear_irq_status(IrqMask::all());

        self.radio
            .write_buffer(0x00, data)
            .map_err(|_| Sx1262Error::Spi)?;

        let params: PacketParams = LoRaPacketParams::default()
            .set_preamble_len(8)
            .set_header_type(LoRaHeaderType::VarLen)
            .set_payload_len(data.len() as u8)
            .set_crc_type(LoRaCrcType::CrcOn)
            .set_invert_iq(LoRaInvertIq::Standard)
            .into();
        self.radio
            .set_packet_params(params)
            .map_err(|_| Sx1262Error::Spi)?;

        self.radio
            .set_tx(RxTxTimeout::from_ms(3000))
            .map_err(|_| Sx1262Error::Spi)?;

        if cfg!(feature = "debug") {
            if let Ok(status) = self.radio.get_status() {
                debug_println!("  send: post-set_tx {:?}", status);
            }
        }

        // Poll IRQ for TxDone/Timeout — SF7 TX should complete in <100ms
        let start = esp_hal::time::Instant::now();
        let timeout = esp_hal::time::Duration::from_millis(500);
        loop {
            if start.elapsed() > timeout {
                debug_println!("  TX timeout (no TxDone IRQ after 500ms)");
                let _ = self.radio.clear_irq_status(IrqMask::all());
                return Err(Sx1262Error::Timeout);
            }
            if let Ok(irq) = self.radio.get_irq_status() {
                if irq.tx_done() || irq.timeout() {
                    let done = irq.tx_done();
                    let _ = self.radio.clear_irq_status(IrqMask::all());
                    return if done {
                        Ok(())
                    } else {
                        Err(Sx1262Error::Timeout)
                    };
                }
            }
        }
    }

    fn max_packet_len(&self) -> usize {
        255
    }
}
