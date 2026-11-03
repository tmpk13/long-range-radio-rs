//! The bridge against the simulated controller, UART and SWD target.

use super::*;
use crate::bridge::{Bridge, PRODUCT};
use crate::swd::{Deps, Jtag, NoLeds, NoSwo, Swd};
use crate::usb::Ch572Usb;
use dap_rs::dap::{Dap, DapVersion};
use usb_device::bus::UsbBusAllocator;

const IN: usize = 0;
const OUT: usize = 1;

fn setup(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> [u8; 8] {
    let [v0, v1] = value.to_le_bytes();
    let [i0, i1] = index.to_le_bytes();
    let [l0, l1] = length.to_le_bytes();
    [request_type, request, v0, v1, i0, i1, l0, l1]
}

fn get_descriptor(kind: u8, index: u8, length: u16) -> [u8; 8] {
    setup(0x80, 6, (kind as u16) << 8 | index as u16, 0x0409, length)
}

type SimBridge<'a> = Bridge<'a, Ch572Usb<&'a SimUsb>, SimTarget, NoDelay>;

/// A USB host: drives transactions into the simulated controller and
/// polls the bridge between them, the way firmware would run.
struct Host<'a> {
    sim: &'a SimUsb,
    bridge: SimBridge<'a>,
    uart: FakeUart,
    addr: u8,
    /// The toggle the next packet should carry, per endpoint and direction.
    toggle: [[bool; 2]; 4],
}

impl<'a> Host<'a> {
    fn new(
        sim: &'a SimUsb,
        bus: &'a UsbBusAllocator<Ch572Usb<&'a SimUsb>>,
        target: SimTarget,
    ) -> Self {
        Self {
            sim,
            bridge: Bridge::new(bus, target, NoDelay, "TEST-1"),
            uart: FakeUart::default(),
            addr: 0,
            toggle: [[false; 2]; 4],
        }
    }

    fn poll(&mut self) {
        self.bridge.poll(&mut self.uart);
    }

    fn settle(&mut self) {
        for _ in 0..4 {
            self.poll();
        }
    }

    fn in_packet(&mut self, ep: usize) -> Vec<u8> {
        for _ in 0..50 {
            self.poll();
            match self.sim.in_(self.addr, ep) {
                Ok((data, data1)) => {
                    assert_eq!(data1, self.toggle[ep][IN], "IN toggle on EP{ep}");
                    self.toggle[ep][IN] = !data1;
                    return data;
                }
                Err(Handshake::Nak) => continue,
                Err(e) => panic!("IN EP{ep}: {e:?}"),
            }
        }
        panic!("IN EP{ep}: NAKed throughout");
    }

    fn out_packet(&mut self, ep: usize, data: &[u8]) {
        for _ in 0..50 {
            self.poll();
            match self.sim.out(self.addr, ep, data, self.toggle[ep][OUT]) {
                Handshake::Ack => {
                    self.toggle[ep][OUT] = !self.toggle[ep][OUT];
                    return;
                }
                Handshake::Nak => continue,
                e => panic!("OUT EP{ep}: {e:?}"),
            }
        }
        panic!("OUT EP{ep}: NAKed throughout");
    }

    fn control_in(&mut self, request: [u8; 8]) -> Vec<u8> {
        self.poll();
        self.control_in_now(request)
    }

    /// `control_in` without first letting firmware catch up, so the SETUP
    /// can land on top of a transaction it has not processed yet.
    fn control_in_now(&mut self, request: [u8; 8]) -> Vec<u8> {
        let length = u16::from_le_bytes([request[6], request[7]]) as usize;
        assert_eq!(self.sim.setup(self.addr, request), Handshake::Ack);
        self.toggle[0] = [true, true];
        let mut data = Vec::new();
        loop {
            let packet = self.in_packet(0);
            let short = packet.len() < 64;
            data.extend(packet);
            if short || data.len() >= length {
                break;
            }
        }
        self.out_packet(0, &[]);
        self.settle();
        data
    }

    fn control_out(&mut self, request: [u8; 8], data: &[u8]) {
        self.poll();
        assert_eq!(self.sim.setup(self.addr, request), Handshake::Ack);
        self.toggle[0] = [true, true];
        for chunk in data.chunks(64) {
            self.out_packet(0, chunk);
        }
        assert!(self.in_packet(0).is_empty(), "status stage carries no data");
        self.settle();
    }

    fn enumerate(&mut self) -> (Vec<u8>, Vec<u8>) {
        self.sim.bus_reset();
        self.settle();
        let device = self.control_in(get_descriptor(1, 0, 64));
        self.control_out(setup(0x00, 5, 7, 0, 0), &[]);
        self.addr = 7;
        assert_eq!(self.sim.address(), 7);
        let head = self.control_in(get_descriptor(2, 0, 9));
        let total = u16::from_le_bytes([head[2], head[3]]);
        let config = self.control_in(get_descriptor(2, 0, total));
        assert_eq!(config.len(), total as usize);
        self.control_out(setup(0x00, 9, 1, 0, 0), &[]);
        assert!(self.bridge.configured());
        (device, config)
    }

    fn string(&mut self, index: u8) -> String {
        let raw = self.control_in(get_descriptor(3, index, 255));
        let units: Vec<u16> = raw[2..]
            .chunks(2)
            .map(|c| u16::from_le_bytes([c[0], c[1]]))
            .collect();
        String::from_utf16(&units).unwrap()
    }

    /// One CMSIS-DAP command over the bulk endpoints.
    fn dap(&mut self, command: &[u8]) -> Vec<u8> {
        self.out_packet(2, command);
        self.in_packet(3)
    }
}

/// (interface number, class, iInterface, endpoint addresses) per interface.
fn interfaces(config: &[u8]) -> Vec<(u8, u8, u8, Vec<u8>)> {
    let mut out: Vec<(u8, u8, u8, Vec<u8>)> = Vec::new();
    let mut i = 0;
    while i < config.len() {
        let (len, kind) = (config[i] as usize, config[i + 1]);
        match kind {
            4 => out.push((config[i + 2], config[i + 5], config[i + 8], Vec::new())),
            5 => out.last_mut().unwrap().3.push(config[i + 2]),
            _ => {}
        }
        i += len;
    }
    out
}

#[test]
fn enumerates_as_serial_port_plus_cmsis_dap() {
    let sim = SimUsb::new();
    let bus = UsbBusAllocator::new(Ch572Usb::new(&sim));
    let mut host = Host::new(&sim, &bus, SimTarget::new());
    let (device, config) = host.enumerate();

    assert_eq!(device[1], 1);
    // Composite with interface association descriptors.
    assert_eq!(&device[4..7], &[0xEF, 0x02, 0x01]);
    assert_eq!(device[7], 64);
    assert_eq!(&device[8..12], &[0x09, 0x12, 0x01, 0x00]);
    assert!(host.string(device[15]).contains("CMSIS-DAP"));
    assert_eq!(host.string(device[15]), PRODUCT);
    assert_eq!(host.string(device[16]), "TEST-1");

    let ifs = interfaces(&config);
    assert_eq!(ifs.len(), 3);
    assert_eq!((ifs[0].1, ifs[0].3.clone()), (0x02, vec![0x81]));
    assert_eq!((ifs[1].1, ifs[1].3.clone()), (0x0A, vec![0x82, 0x01]));
    assert_eq!((ifs[2].1, ifs[2].3.clone()), (0xFF, vec![0x02, 0x83]));
    assert_eq!(host.string(ifs[2].2), "CMSIS-DAP v2 Interface");

    // Each endpoint's DMA register got its buffer.
    let dma = sim.state.borrow().dma;
    assert_eq!(dma, [0x0100, 0x0180, 0x0200, 0x0280]);
}

#[test]
fn serial_port_follows_line_coding_and_moves_data_both_ways() {
    let sim = SimUsb::new();
    let bus = UsbBusAllocator::new(Ch572Usb::new(&sim));
    let mut host = Host::new(&sim, &bus, SimTarget::new());
    host.enumerate();

    // SET_LINE_CODING 115200 8N1, then DTR and RTS up.
    let mut coding = 115_200u32.to_le_bytes().to_vec();
    coding.extend([0, 0, 8]);
    host.control_out(setup(0x21, 0x20, 0, 0, 7), &coding);
    host.control_out(setup(0x21, 0x22, 3, 0, 0), &[]);
    assert_eq!(host.uart.setting.map(|s| (s.dl, s.lcr)), Some((109, 0x03)));

    host.out_packet(1, b"hello");
    host.settle();
    assert_eq!(host.uart.sent, b"hello");

    // A retransmission with the same toggle (the host missed our ACK) is
    // acknowledged and dropped.
    host.toggle[1][OUT] = !host.toggle[1][OUT];
    host.out_packet(1, b"hello");
    host.settle();
    assert_eq!(host.uart.sent, b"hello");
    host.out_packet(1, b"!");
    host.settle();
    assert_eq!(host.uart.sent, b"hello!");

    host.uart.incoming.extend(b"world");
    let mut got = Vec::new();
    while got.len() < 5 {
        got.extend(host.in_packet(2));
    }
    assert_eq!(got, b"world");
    // Collected packets are not sent twice.
    host.settle();
    assert_eq!(sim.in_(host.addr, 2), Err(Handshake::Nak));
}

#[test]
fn closing_the_port_at_1200_baud_requests_the_bootloader() {
    let sim = SimUsb::new();
    let bus = UsbBusAllocator::new(Ch572Usb::new(&sim));
    let mut host = Host::new(&sim, &bus, SimTarget::new());
    host.enumerate();

    let mut coding = 1_200u32.to_le_bytes().to_vec();
    coding.extend([0, 0, 8]);
    host.control_out(setup(0x21, 0x22, 3, 0, 0), &[]);
    host.control_out(setup(0x21, 0x20, 0, 0, 7), &coding);
    host.poll();
    assert!(host.sim.setup(host.addr, setup(0x21, 0x22, 0, 0, 0)) == Handshake::Ack);
    let mut requested = false;
    for _ in 0..20 {
        requested |= host.bridge.poll(&mut host.uart).enter_bootloader;
        let _ = host.sim.in_(host.addr, 0);
    }
    assert!(requested);
}

#[test]
fn cmsis_dap_reads_the_target_idcode_over_usb() {
    let sim = SimUsb::new();
    let bus = UsbBusAllocator::new(Ch572Usb::new(&sim));
    let target = SimTarget::new();
    let mut host = Host::new(&sim, &bus, target.clone());
    host.enumerate();

    // DAP_Info capabilities: SWD only (plus the streaming bit dap-rs sets).
    assert_eq!(host.dap(&[0x00, 0xF0]), [0x00, 0x01, 0x41]);
    // DAP_Info packet size: 64.
    assert_eq!(host.dap(&[0x00, 0xFF]), [0x00, 0x02, 64, 0]);
    assert_eq!(host.dap(&[0x02, 0x01]), [0x02, 0x01]);
    // Line reset, JTAG-to-SWD, line reset, idle: probe-rs's attach.
    let reset = [0x12, 51, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
    assert_eq!(host.dap(&reset), [0x12, 0x00]);
    assert_eq!(host.dap(&[0x12, 16, 0x9E, 0xE7]), [0x12, 0x00]);
    assert_eq!(host.dap(&reset), [0x12, 0x00]);
    assert_eq!(host.dap(&[0x12, 8, 0x00]), [0x12, 0x00]);
    // DAP_Transfer: read DPIDR.
    assert_eq!(
        host.dap(&[0x05, 0, 1, 0x02]),
        [0x05, 1, 1, 0x77, 0x24, 0xA0, 0x5B]
    );
    host.settle();
    assert_eq!(
        sim.in_(host.addr, 3),
        Err(Handshake::Nak),
        "one response per command"
    );
    {
        let t = target.0.borrow();
        assert!(!t.contention);
        assert!(t.services > 0, "transfers must keep the UART serviced");
        assert!(t.host_drives);
    }

    // The host goes away without DAP_Disconnect: the pins are released.
    sim.bus_reset();
    host.settle();
    assert!(!target.0.borrow().host_drives);
}

#[test]
fn setup_behind_a_pending_in_completion_starts_a_fresh_transfer() {
    let sim = SimUsb::new();
    let bus = UsbBusAllocator::new(Ch572Usb::new(&sim));
    let mut host = Host::new(&sim, &bus, SimTarget::new());
    let (device, config) = host.enumerate();
    assert!(config.len() > 64);

    // Start reading the long configuration descriptor, collect its first
    // packet, then abort with a new SETUP before firmware has seen the IN
    // completion.
    host.poll();
    assert_eq!(
        sim.setup(host.addr, get_descriptor(2, 0, 255)),
        Handshake::Ack
    );
    host.toggle[0] = [true, true];
    assert_eq!(host.in_packet(0).len(), 64);
    let again = host.control_in_now(get_descriptor(1, 0, 18));
    assert_eq!(again, device[..18]);
}

#[test]
fn set_address_applies_even_if_usb_device_skips_its_completion() {
    let sim = SimUsb::new();
    let bus = UsbBusAllocator::new(Ch572Usb::new(&sim));
    let mut host = Host::new(&sim, &bus, SimTarget::new());
    sim.bus_reset();
    host.settle();
    host.poll();
    assert_eq!(sim.setup(0, setup(0x00, 5, 9, 0, 0)), Handshake::Ack);
    host.toggle[0] = [true, true];
    assert!(host.in_packet(0).is_empty());
    // A SETUP lands before the status completion is processed, so
    // usb-device ignores that completion; the driver must not.
    assert_eq!(sim.setup(0, setup(0x80, 0, 0, 0, 2)), Handshake::Ack);
    host.settle();
    assert_eq!(sim.address(), 9);
}

#[test]
fn clear_halt_resets_the_toggle() {
    let sim = SimUsb::new();
    let bus = UsbBusAllocator::new(Ch572Usb::new(&sim));
    let mut host = Host::new(&sim, &bus, SimTarget::new());
    host.enumerate();
    host.out_packet(1, b"a");
    host.settle();
    assert!(host.toggle[1][OUT]);
    // CLEAR_FEATURE(ENDPOINT_HALT) on EP1 OUT: both sides back to DATA0.
    host.control_out(setup(0x02, 0x01, 0, 0x01, 0), &[]);
    host.toggle[1][OUT] = false;
    host.out_packet(1, b"b");
    host.settle();
    assert_eq!(host.uart.sent, b"ab");
}

// ---- SWD through dap-rs, without USB -------------------------------------

type SimDap =
    Dap<'static, Deps<SimTarget>, NoLeds, NoDelay, Jtag<SimTarget>, Swd<SimTarget>, NoSwo>;

fn dap(target: &SimTarget) -> SimDap {
    Dap::new(Deps(target.clone()), NoLeds, NoDelay, None, "test")
}

fn run(dap: &mut SimDap, command: &[u8]) -> Vec<u8> {
    let mut response = [0u8; 64];
    let n = dap.process_command(command, &mut response, DapVersion::V2);
    response[..n].to_vec()
}

fn attach(dap: &mut SimDap) {
    let reset = [0x12, 51, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
    run(dap, &[0x02, 0x01]);
    run(dap, &reset);
    run(dap, &[0x12, 16, 0x9E, 0xE7]);
    run(dap, &reset);
    run(dap, &[0x12, 8, 0x00]);
}

#[test]
fn swd_write_read_and_posted_ap_reads() {
    let target = SimTarget::new();
    let mut dap = dap(&target);
    attach(&mut dap);
    // Write CTRL/STAT with both power-up requests, read it back acked.
    assert_eq!(
        run(&mut dap, &[0x05, 0, 2, 0x04, 0x00, 0x00, 0x00, 0x50, 0x06]),
        [0x05, 2, 1, 0x00, 0x00, 0x00, 0xF0]
    );
    // AP register 0x0C write, then read: dap-rs fetches the posted result
    // from RDBUFF.
    let value = 0xDEAD_BEEFu32.to_le_bytes();
    let mut write = vec![0x05, 0, 1, 0x0D];
    write.extend(value);
    assert_eq!(run(&mut dap, &write), [0x05, 1, 1]);
    let mut read = vec![0x05, 1, 1];
    read.extend(value);
    assert_eq!(run(&mut dap, &[0x05, 0, 1, 0x0F]), read);
    assert!(!target.0.borrow().contention);
}

#[test]
fn swd_wait_is_retried_and_errors_are_reported() {
    let target = SimTarget::new();
    let mut dap = dap(&target);
    attach(&mut dap);
    target.0.borrow_mut().waits = 3;
    assert_eq!(run(&mut dap, &[0x05, 0, 1, 0x02])[..3], [0x05, 1, 1]);
    // More WAITs than dap-rs retries: reported as WAIT.
    target.0.borrow_mut().waits = 100;
    assert_eq!(run(&mut dap, &[0x05, 0, 1, 0x02]), [0x05, 1, 2]);
    target.0.borrow_mut().waits = 0;
    target.0.borrow_mut().corrupt_parity = true;
    assert_eq!(run(&mut dap, &[0x05, 0, 1, 0x02]), [0x05, 1, 0x0F]);
    // The next transfer is clean again.
    assert_eq!(run(&mut dap, &[0x05, 0, 1, 0x02])[..3], [0x05, 1, 1]);
    assert!(!target.0.borrow().contention);
}

#[test]
fn swd_without_line_reset_gets_no_acknowledge() {
    let target = SimTarget::new();
    let mut dap = dap(&target);
    run(&mut dap, &[0x02, 0x01]);
    assert_eq!(run(&mut dap, &[0x05, 0, 1, 0x02]), [0x05, 1, 0x0F]);
}

#[test]
fn swj_pins_drives_and_reports_reset() {
    let target = SimTarget::new();
    let mut dap = dap(&target);
    // Assert nRESET (bit 7 low in the output, set in the mask).
    let pins = run(&mut dap, &[0x10, 0x00, 0x80, 0, 0, 0, 0]);
    assert!(target.0.borrow().reset_asserted);
    assert_eq!(pins[1] & 0x80, 0);
    let pins = run(&mut dap, &[0x10, 0x80, 0x80, 0, 0, 0, 0]);
    assert!(!target.0.borrow().reset_asserted);
    assert_eq!(pins[1] & 0x80, 0x80);
}
