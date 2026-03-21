//! SX1262 radio driver implementing RadioHead's `RadioDriver` trait.

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;
use radiohead::RadioDriver;
use sx126x::conf::Config;
use sx126x::op::*;
use sx126x::SX126x;

/// Wrapper around `sx126x::SX126x` that implements [`RadioDriver`].
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
            tcxo_opts: None,
        };

        self.radio.init(conf).expect("SX1262 init failed");
    }
}

impl<SPI, NRST, BUSY, ANT, DIO1, SPIERR, PINERR> RadioDriver
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
    fn poll_recv(&mut self, buf: &mut [u8]) -> Option<(u8, i16)> {
        // Enter continuous RX if not already listening
        if !self.rx_active {
            self.radio.set_rx(RxTxTimeout::continuous_rx()).ok()?;
            self.rx_active = true;
        }

        // Poll IRQ status over SPI
        let irq = self.radio.get_irq_status().ok()?;

        if !irq.rx_done() {
            return None;
        }

        // Clear all pending IRQs
        let _ = self.radio.clear_irq_status(IrqMask::all());

        let rx_status = self.radio.get_rx_buffer_status().ok()?;
        let len = rx_status.payload_length_rx();
        let offset = rx_status.rx_start_buffer_pointer();

        if (len as usize) > buf.len() {
            self.rx_active = false;
            return None;
        }

        self.radio
            .read_buffer(offset, &mut buf[..len as usize])
            .ok()?;

        let pkt_status = self.radio.get_packet_status().ok()?;
        let rssi = pkt_status.rssi_pkt() as i16;

        // Stay in RX — continuous mode persists
        Some((len, rssi))
    }

    fn send(&mut self, data: &[u8]) -> bool {
        self.rx_active = false;
        self.radio
            .write_bytes(data, RxTxTimeout::from_ms(3000), 8, LoRaCrcType::CrcOn)
            .is_ok()
    }

    fn max_message_length(&self) -> u8 {
        255
    }
}
