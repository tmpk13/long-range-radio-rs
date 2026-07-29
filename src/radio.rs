//! SX1262 radio driver implementing [`PacketRadio`] via the STM32WLE5 SubGHz peripheral.

use crate::config::{RADIO_PRESET, RadioPreset, TX_CHIP_TIMEOUT_MS, TX_POLL_TIMEOUT_MS};
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
use cortex_m::interrupt::CriticalSection;
use stm32wlxx_hal::gpio::{Output, OutputArgs, PinState, Speed, pins};
use stm32wlxx_hal::spi::{SgMiso, SgMosi};
use stm32wlxx_hal::subghz::{
    CalibrateImage, CfgIrq, CodingRate, FallbackMode, HeaderType, Irq, LoRaBandwidth,
    LoRaModParams, LoRaPacketParams, LoRaSyncWord, Ocp, PaConfig, PaSel, PacketType, RampTime,
    RegMode, RfFreq, SpreadingFactor, StandbyClk, SubGhz, TcxoMode, TcxoTrim, Timeout, TxParams,
};

/// Errors from the SubGHz radio.
#[derive(Debug)]
pub enum Sx1262Error {
    Radio,
    Timeout,
}

/// Which way the module's antenna switch is pointed.
#[derive(Clone, Copy, PartialEq, Eq)]
enum RfPath {
    /// Antenna isolated from both the PA and the receiver.
    Off,
    /// Antenna to the receiver.
    Rx,
    /// Antenna to the high-power PA, the only transmit output this board
    /// uses ([`PaSel::Hp`] below).
    TxHp,
}

/// The module's antenna switch, on PA4 (control 1) and PA5 (control 2).
///
/// The radio die has no bonded DIO2, so there is no `SetDio2AsRfSwitchCtrl`
/// to hand the job to the radio the way a discrete SX1262 would: the path
/// has to be selected from the MCU before every transmit and every receive.
/// Left unconfigured the pins sit as analog inputs and the switch floats, so
/// signal reaches the antenna only through the switch's off-state isolation
/// - tens of dB down, which works across a bench and nowhere else.
///
/// Both lines low isolates the antenna, which is where the switch belongs
/// whenever the radio is neither transmitting nor listening.
///
/// The control lines are static logic levels next to an RF path, so they are
/// driven at the slowest edge rate the GPIO offers.
pub struct RfSwitch {
    ctrl1: Output<pins::A4>,
    ctrl2: Output<pins::A5>,
}

impl RfSwitch {
    /// Take the two control pins, leaving the switch isolated.
    pub fn new(a4: pins::A4, a5: pins::A5, cs: &CriticalSection) -> Self {
        const ARGS: OutputArgs = OutputArgs {
            speed: Speed::Low,
            level: PinState::Low,
            ..OutputArgs::new()
        };
        Self {
            ctrl1: Output::new(a4, &ARGS, cs),
            ctrl2: Output::new(a5, &ARGS, cs),
        }
    }

    fn set(&mut self, path: RfPath) {
        let (c1, c2) = match path {
            RfPath::Off => (PinState::Low, PinState::Low),
            RfPath::Rx => (PinState::High, PinState::Low),
            RfPath::TxHp => (PinState::Low, PinState::High),
        };
        self.ctrl1.set_level(c1);
        self.ctrl2.set_level(c2);
    }
}

/// SetRx timeout value that selects continuous RX. On the SX126x the SetRx
/// timeout doubles as a mode select: 0x000000 is single mode - the receiver
/// stays on only until it decodes one packet, then drops to the fallback
/// mode - while 0xFFFFFF keeps it in RX across packets. A node arms RX once
/// and expects to keep hearing the network, so it must be the latter; single
/// mode would leave a node that rarely transmits deaf after its first packet.
const RX_CONTINUOUS: Timeout = Timeout::from_raw(0x00FF_FFFF);

/// Payload length written into the packet params before receiving.
///
/// With an explicit header this field is not the length of anything - the
/// header carries that - it is the largest payload the receiver will accept.
/// Every transmit has to narrow it to the size of the frame being sent, so
/// receiving means putting it back.
const RX_MAX_PAYLOAD: u8 = 255;

/// SMPS control 0. Bit 6 enables clock detection, which has to be on before
/// the SMPS is selected; every other bit belongs to the regulator and must
/// be preserved.
const REG_SMPS_C0: u16 = 0x0916;

/// [`REG_SMPS_C0`] clock-detection enable.
const SMPS_CLK_DET_EN: u8 = 1 << 6;

/// TX clamp configuration. Bits 4:1 all set improves the PA's tolerance of
/// an antenna mismatch (SX1261/2 datasheet, "Better resistance of the
/// SX1262 Tx to antenna mismatch").
const REG_TX_CLAMP: u16 = 0x08D8;

/// TX modulation configuration. Bit 2 must be cleared for a 500 kHz LoRa
/// bandwidth and set for every other bandwidth (SX1261/2 datasheet,
/// "Modulation quality with 500 kHz LoRa bandwidth").
const REG_TX_MODULATION: u16 = 0x0889;

/// Image calibration bounds for operation at `freq_hz`.
///
/// Image rejection is calibrated for a band, and calibrating for a band the
/// radio is not operating in throws away the rejection - which is
/// sensitivity, and so range. Anything outside the bands the datasheet
/// tabulates gets the 4 MHz-aligned window that brackets it rather than the
/// nearest named band.
fn image_band(freq_hz: u32) -> CalibrateImage {
    match freq_hz / 1_000_000 {
        902..=928 => CalibrateImage::ISM_902_928,
        863..=870 => CalibrateImage::ISM_863_870,
        779..=787 => CalibrateImage::ISM_779_787,
        470..=510 => CalibrateImage::ISM_470_510,
        430..=440 => CalibrateImage::ISM_430_440,
        // `from_freq` takes MHz bounds on a 4 MHz grid, low first.
        mhz => CalibrateImage::from_freq((mhz / 4 * 4) as u16, (mhz / 4 * 4 + 4) as u16),
    }
}

/// The LoRa packet params this firmware always uses, for a payload of
/// `payload_len` bytes: an 8-symbol preamble, an explicit header, the
/// hardware CRC on, and no IQ inversion.
fn packet_params(payload_len: u8) -> LoRaPacketParams {
    LoRaPacketParams::new()
        .set_preamble_len(8)
        .set_header_type(HeaderType::Variable)
        .set_payload_len(payload_len)
        .set_crc_en(true)
        .set_invert_iq(false)
}

/// SubGHz radio driver that implements [`PacketRadio`].
///
/// On the STM32WLE5 the SX1262 is integrated - the [`SubGhz`] peripheral
/// handles the internal SPI3 interface, BUSY signal, and DIO lines.  The
/// module's antenna switch is the one part that is not on-die and still
/// needs MCU pins: see [`RfSwitch`].
pub struct Sx1262Driver {
    radio: SubGhz<SgMiso, SgMosi>,
    rf_switch: RfSwitch,
    rx_active: bool,
}

impl Sx1262Driver {
    /// Create a new SubGHz radio driver. Call [`init`](Self::init) before use.
    pub fn new(radio: SubGhz<SgMiso, SgMosi>, rf_switch: RfSwitch) -> Self {
        Self {
            radio,
            rf_switch,
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
        self.rx_active = false;

        // Nothing is on the air during configuration, and a live antenna
        // path while the PA and receiver are being reconfigured is a path
        // nobody is driving deliberately.
        self.rf_switch.set(RfPath::Off);
        // Reset the radio and enter standby
        self.radio.set_standby(StandbyClk::Rc).expect("set_standby");

        // Use DCDC regulator for better efficiency.
        //
        // Clock detection has to be enabled before the SMPS is, not after.
        // Read-modify-write, not the HAL's `set_smps_clock_det_en`: that
        // setter writes the whole register, so it enables clock detection by
        // clearing every other bit of the regulator's configuration.
        let smps = self.read_reg(REG_SMPS_C0);
        self.write_reg(REG_SMPS_C0, smps | SMPS_CLK_DET_EN);
        self.radio.set_regulator_mode(RegMode::Smps).ok();

        // Configure TCXO: Wio-E5 has a 32 MHz TCXO on DIO3
        self.radio
            .set_tcxo_mode(
                &TcxoMode::new()
                    .set_txco_trim(TcxoTrim::Volts1pt8)
                    .set_timeout(Timeout::from_millis_sat(10)),
            )
            .expect("set_tcxo_mode");

        // Recalibrate every block now that there is a 32 MHz clock. The
        // automatic calibration at power-up ran before the TCXO was enabled,
        // so the RC64k, RC13M, PLL, ADC and image results it produced were
        // all derived from a clock that was not running - which shows up as
        // frequency error and lost sensitivity rather than as a failure.
        // 0x7F selects every block.
        self.radio.calibrate(0x7F).expect("calibrate");
        self.wait_on_busy();

        // Image rejection is calibrated for the band actually in use.
        self.radio
            .calibrate_image(image_band(rf_frequency))
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

        // Applied after the PA is configured, since configuring it is what
        // this compensates for. The board's antenna is a connector and a
        // short wire, so the mismatch this guards the PA against is the
        // normal case rather than a fault.
        let clamp = self.read_reg(REG_TX_CLAMP);
        self.write_reg(REG_TX_CLAMP, clamp | 0x1E);

        // LoRa modulation: SF / BW / CR selected by the build-time preset.
        // LDRO must be enabled when the symbol duration exceeds 16.38 ms
        // (SF11/SF12 at BW125, SF12 at BW62.5), otherwise the link is
        // unreliable.
        let (sf, bw, cr, ldro) = match RADIO_PRESET {
            RadioPreset::Fast => (
                SpreadingFactor::Sf7,
                LoRaBandwidth::Bw125,
                CodingRate::Cr45,
                false,
            ),
            RadioPreset::Long => (
                SpreadingFactor::Sf10,
                LoRaBandwidth::Bw125,
                CodingRate::Cr45,
                false,
            ),
            RadioPreset::Max => (
                SpreadingFactor::Sf12,
                LoRaBandwidth::Bw125,
                CodingRate::Cr48,
                true,
            ),
            RadioPreset::Extreme => (
                SpreadingFactor::Sf12,
                LoRaBandwidth::Bw62,
                CodingRate::Cr48,
                true,
            ),
        };
        self.radio
            .set_lora_mod_params(
                &LoRaModParams::new()
                    .set_sf(sf)
                    .set_bw(bw)
                    .set_cr(cr)
                    .set_ldro_en(ldro),
            )
            .expect("set_lora_mod_params");

        // Bandwidth-dependent, and the modulation params are what carry the
        // bandwidth, so this follows them. Only 500 kHz wants the bit clear;
        // every other bandwidth wants it set, which is also the reset value.
        let txmod = self.read_reg(REG_TX_MODULATION);
        let txmod = if bw == LoRaBandwidth::Bw500 {
            txmod & !0x04
        } else {
            txmod | 0x04
        };
        self.write_reg(REG_TX_MODULATION, txmod);

        // LoRa packet params: 8-sym preamble, variable header, 255-byte max
        self.radio
            .set_lora_packet_params(&packet_params(RX_MAX_PAYLOAD))
            .expect("set_lora_packet_params");

        // LoRa sync word: public network (0x3444)
        self.radio
            .set_lora_sync_word(LoRaSyncWord::Public)
            .expect("set_lora_sync_word");

        // Buffer base addresses: TX at 0x00, RX at 0x00
        self.radio
            .set_buffer_base_address(0x00, 0x00)
            .expect("set_buffer_base_address");

        // IRQ: route RxDone, TxDone, Err (CRC) and Timeout to all lines
        self.radio
            .set_irq_cfg(
                &CfgIrq::new()
                    .irq_enable_all(Irq::RxDone)
                    .irq_enable_all(Irq::TxDone)
                    .irq_enable_all(Irq::Err)
                    .irq_enable_all(Irq::Timeout),
            )
            .expect("set_irq_cfg");

        // Set fallback mode to standby after TX/RX
        self.radio
            .set_tx_rx_fallback_mode(FallbackMode::Standby)
            .ok();

        // Over-current protection
        self.radio.set_pa_ocp(Ocp::Max140m).ok();
        debug_println!("LoRa preset: {}", RADIO_PRESET.name());
        debug_println!("SubGHz init complete.");
    }

    /// Print radio diagnostics. Returns `true` if the radio responds.
    pub fn print_diagnostics(&mut self) -> bool {
        debug_println!("Checking radio hardware:");
        match self.radio.status() {
            Ok(s) => {
                debug_println!("  Status: {:?}", s);
                // The status byte reports the mode the radio is in, not
                // whether it got there intact. GetError is the only thing
                // that names a TCXO that never started, a calibration or PLL
                // lock that failed, or a PA that would not ramp - a radio
                // that came up deaf for any of those still reports a
                // perfectly healthy standby.
                if let Ok((_, err)) = self.radio.op_error()
                    && err != 0
                {
                    rtt_target::rprintln!("WARNING: radio op error 0x{:04X}", err);
                    self.radio.clear_error().ok();
                }
                true
            }
            Err(_) => {
                rtt_target::rprintln!("WARNING: Radio not responding!");
                false
            }
        }
    }

    /// LoRa packet statistics since the last stats reset:
    /// `(received, CRC errors, header errors)`.
    pub fn lora_stats(&mut self) -> Result<(u16, u16, u16), Sx1262Error> {
        let stats = self.radio.lora_stats().map_err(|_| Sx1262Error::Radio)?;
        Ok((stats.pkt_rx(), stats.pkt_crc(), stats.pkt_hdr_err()))
    }

    /// Arm continuous receive, restoring the maximum acceptable payload
    /// length first.
    ///
    /// That restore is the whole reason this is a function rather than a
    /// `set_rx` call at each site: a transmit leaves the packet params
    /// carrying the length of the frame it just sent, and re-entering
    /// receive with that still in place caps the receiver at the size of
    /// this node's own last transmission - so a node whose last packet was
    /// short goes deaf to every longer frame on the network.
    ///
    /// Packet params are configuration, so the radio is put back in standby
    /// to take them - the caller may be re-arming from continuous RX after
    /// dropping an oversize packet.
    fn enter_rx(&mut self) -> Result<(), Sx1262Error> {
        self.rf_switch.set(RfPath::Off);
        self.radio
            .set_standby(StandbyClk::Rc)
            .map_err(|_| Sx1262Error::Radio)?;
        self.wait_on_busy();

        self.radio
            .set_lora_packet_params(&packet_params(RX_MAX_PAYLOAD))
            .map_err(|_| Sx1262Error::Radio)?;

        self.rf_switch.set(RfPath::Rx);
        self.radio
            .set_rx(RX_CONTINUOUS)
            .map_err(|_| Sx1262Error::Radio)?;
        self.wait_on_busy();
        self.rx_active = true;
        Ok(())
    }

    /// Run one raw SUBGHZSPI transaction, replacing `buf` with what the
    /// radio shifted back.
    ///
    /// `stm32wlxx-hal` keeps its register table private and covers only the
    /// registers it has methods for, so the errata workarounds above have no
    /// route through it. This is the same transaction the HAL performs for
    /// its own register access: wait out BUSY, pull NSS low, shift the bytes,
    /// release NSS. SPI3 belongs to the `SubGhz` this method takes `&mut
    /// self` on, and every radio call in this firmware runs from one task, so
    /// the access is exclusive.
    fn subghz_xfer(&mut self, buf: &mut [u8]) {
        // SUBGHZSPI data register. Byte-wide accesses: the peripheral is in
        // 8-bit frame mode, and a 32-bit write would shift out four bytes.
        const SPI3_DR: *mut u8 = 0x5801_000C as *mut u8;

        self.wait_on_busy();
        unsafe {
            let pwr = &*stm32wlxx_hal::pac::PWR::PTR;
            let spi = &*stm32wlxx_hal::pac::SPI3::PTR;
            pwr.subghzspicr.write(|w| w.nss().clear_bit());
            for byte in buf.iter_mut() {
                while spi.sr.read().ftlvl().is_full() {}
                core::ptr::write_volatile(SPI3_DR, *byte);
                while spi.sr.read().frlvl().is_empty() {}
                *byte = core::ptr::read_volatile(SPI3_DR);
            }
            pwr.subghzspicr.write(|w| w.nss().set_bit());
        }
        self.wait_on_busy();
    }

    /// Read one SubGHz configuration register.
    fn read_reg(&mut self, addr: u16) -> u8 {
        // ReadRegister (0x1D): opcode, big-endian address, one byte during
        // which the radio returns its status, then the register value.
        let mut buf = [0x1D, (addr >> 8) as u8, addr as u8, 0x00, 0x00];
        self.subghz_xfer(&mut buf);
        buf[4]
    }

    /// Write one SubGHz configuration register.
    fn write_reg(&mut self, addr: u16, value: u8) {
        // WriteRegister (0x0D): opcode, big-endian address, value.
        let mut buf = [0x0D, (addr >> 8) as u8, addr as u8, value];
        self.subghz_xfer(&mut buf);
    }

    /// Poll the RFBUSYS bit to wait for the radio to be ready.
    fn wait_on_busy(&self) {
        // On STM32WLE5 the BUSY signal is exposed as RFBUSYS in PWR->SR2.
        // The SX126x silently ignores SPI commands sent while BUSY is high,
        // so this must be called after every set_standby/set_tx/set_rx
        // before the next command or IRQ poll.
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
            self.enter_rx()?;
        }

        // Poll IRQ status
        let (_, irq) = self.radio.irq_status().map_err(|_| Sx1262Error::Radio)?;

        if irq & Irq::RxDone.mask() == 0 {
            return Ok(None);
        }

        // The SX126x raises RxDone alongside Err when a packet arrives with
        // a bad CRC, and the payload is still sitting in the buffer. The
        // mesh layer above does not checksum, so a corrupt packet handed up
        // would be parsed as a real frame - drop it here.
        let crc_bad = irq & Irq::Err.mask() != 0;

        // Clear all pending IRQs
        let _ = self.radio.clear_irq_status(0xFFFF);

        if crc_bad {
            debug_println!("Dropped packet with bad CRC");
            return Ok(None);
        }

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

        #[cfg(feature = "board")]
        crate::board::activity::note_rx();

        #[cfg(feature = "gps-radio-log")]
        crate::gpslog::events::note_rx(
            len as u8,
            rssi,
            // snr_pkt() is Ratio<i16> with denominator 4: quarter dB.
            *pkt_status.snr_pkt().numer(),
            pkt_status.signal_rssi_pkt().to_integer(),
        );

        // Stay in RX - continuous mode persists
        Ok(Some((len, rssi)))
    }

    fn send(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.rx_active = false;

        // Standby before TX, with the antenna isolated while the packet
        // params change under it.
        self.rf_switch.set(RfPath::Off);
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

        // Packet params must carry the actual payload length, or TxDone
        // never fires. `enter_rx` puts the receive ceiling back afterwards.
        self.radio
            .set_lora_packet_params(&packet_params(data.len() as u8))
            .map_err(|_| Sx1262Error::Radio)?;

        // Point the antenna at the PA before the ramp starts, never after:
        // a PA ramping into an isolated switch is the transmission that goes
        // nowhere.
        self.rf_switch.set(RfPath::TxHp);
        // Start TX with chip timeout
        self.radio
            .set_tx(Timeout::from_millis_sat(TX_CHIP_TIMEOUT_MS as u32))
            .map_err(|_| Sx1262Error::Radio)?;
        self.wait_on_busy();

        #[cfg(feature = "board")]
        crate::board::activity::note_tx();

        if cfg!(feature = "debug")
            && let Ok(status) = self.radio.status()
        {
            debug_println!("  send: TX started, chip status = {:?}", status);
        }

        // Poll IRQ for TxDone/Timeout.
        //
        // The deadline scales with the preset and reaches 7.5 s on `extreme`
        // against a 5 s watchdog, so a transmit that never completes resets
        // the board before the poll can give up - and `max` at 4 s is inside
        // the LSI tolerance of the same 5 s. Rather than stretch the timeout,
        // which would blunt it everywhere, the wait feeds the watchdog
        // itself; it is bounded by TX_POLL_TIMEOUT_MS either way, so this
        // cannot hide a radio that has stopped answering.
        let start_ms = platform::millis();
        let result = loop {
            crate::watchdog::feed_now();
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
                    break if tx_done {
                        Ok(())
                    } else {
                        Err(Sx1262Error::Timeout)
                    };
                }
            }
        };

        // Re-enter continuous RX immediately: the node is deaf while it
        // transmits, so every millisecond spent out of RX after TxDone is
        // another chance to miss someone else's transmission.
        let _ = self.enter_rx();

        #[cfg(feature = "gps-radio-log")]
        crate::gpslog::events::note_tx(data.len() as u8, result.is_ok());

        result
    }

    fn max_packet_len(&self) -> usize {
        255
    }
}
