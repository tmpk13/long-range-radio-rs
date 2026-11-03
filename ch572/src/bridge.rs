//! The bridge: one composite USB device with a CDC-ACM serial port wired
//! to the Wio-E5's USART2 and a CMSIS-DAP v2 interface driving its SWD
//! port.  No hardware access of its own, so the host tests run the same
//! code against the simulated controller and target.

use crate::FSYS_HZ;
use crate::dapclass::{CmsisDapV2, PACKET_SIZE};
use crate::linecode::{UartSetting, uart_setting};
use crate::swd::{Deps, Jtag, NoLeds, NoSwo, Swd, SwdIo};
use crate::touch::Touch;
use dap_rs::dap::{Dap, DapVersion, DelayNs};
use usb_device::LangID;
use usb_device::bus::{UsbBus, UsbBusAllocator};
use usb_device::device::{
    StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid,
};
use usbd_serial::SerialPort;

/// pid.codes test VID/PID, for development hardware only; a board that
/// leaves the bench needs a PID of its own.
pub const VID_PID: UsbVidPid = UsbVidPid(0x1209, 0x0001);
pub const MANUFACTURER: &str = "Forest Sensor Platform";
/// "CMSIS-DAP" in the product string as well: OpenOCD looks for it there.
pub const PRODUCT: &str = "FSP Wio-E5 link CMSIS-DAP";

/// The UART towards the Wio-E5.
pub trait UartPort {
    /// Apply a new line setting; bytes still queued in either direction
    /// are dropped, since they were framed for the old one.
    fn configure(&mut self, setting: UartSetting);
    /// Move received bytes into `buf`; returns the count.
    fn read(&mut self, buf: &mut [u8]) -> usize;
    /// Queue bytes for sending; returns how many fit.
    fn write(&mut self, data: &[u8]) -> usize;
    /// Room left in the send queue.
    fn tx_free(&self) -> usize;
}

/// What one poll of the bridge did, for the LEDs and the main loop.
#[derive(Default, Debug)]
pub struct Events {
    /// The host asked for the ROM bootloader (see [`crate::touch`]).
    pub enter_bootloader: bool,
    /// Serial data moved or a DAP command ran.
    pub activity: bool,
}

pub struct Bridge<'a, B: UsbBus, I: SwdIo, D: DelayNs> {
    device: UsbDevice<'a, B>,
    serial: SerialPort<'a, B>,
    dap_if: CmsisDapV2<'a, B>,
    dap: Dap<'a, Deps<I>, NoLeds, D, Jtag<I>, Swd<I>, NoSwo>,
    /// A DAP response the host has not collected yet.  DAP_Info reports a
    /// packet count of 1, so no new command is read until it goes out.
    dap_response: [u8; PACKET_SIZE as usize],
    dap_pending: usize,
    /// Bytes from the Wio-E5 the serial port has not accepted yet.
    to_host: [u8; PACKET_SIZE as usize],
    to_host_len: usize,
    line: Option<UartSetting>,
    touch: Touch,
    was_configured: bool,
}

impl<'a, B: UsbBus, I: SwdIo, D: DelayNs> Bridge<'a, B, I, D> {
    pub fn new(bus: &'a UsbBusAllocator<B>, swd: I, delay: D, serial_number: &'a str) -> Self {
        // Allocation order fixes the endpoint numbers: the serial port
        // takes EP1 IN (notifications), EP1 OUT and EP2 IN, the DAP
        // interface EP2 OUT and EP3 IN.
        let serial = SerialPort::new(bus);
        let dap_if = CmsisDapV2::new(bus);
        let device = UsbDeviceBuilder::new(bus, VID_PID)
            .strings(&[StringDescriptors::new(LangID::EN_US)
                .manufacturer(MANUFACTURER)
                .product(PRODUCT)
                .serial_number(serial_number)])
            .expect("one language")
            .composite_with_iads()
            .max_packet_size_0(64)
            .expect("64 is a valid EP0 size")
            .build();
        let dap = Dap::new(Deps(swd), NoLeds, delay, None, env!("CARGO_PKG_VERSION"));
        Self {
            device,
            serial,
            dap_if,
            dap,
            dap_response: [0; PACKET_SIZE as usize],
            dap_pending: 0,
            to_host: [0; PACKET_SIZE as usize],
            to_host_len: 0,
            line: None,
            touch: Touch::new(),
            was_configured: false,
        }
    }

    pub fn configured(&self) -> bool {
        self.device.state() == UsbDeviceState::Configured
    }

    pub fn poll<U: UartPort>(&mut self, uart: &mut U) -> Events {
        let mut events = Events::default();
        self.device.poll(&mut [&mut self.serial, &mut self.dap_if]);

        // The UART follows the line setting the host gave the serial port.
        let coding = self.serial.line_coding();
        let setting = uart_setting(
            FSYS_HZ,
            coding.data_rate(),
            coding.data_bits(),
            coding.parity_type(),
            coding.stop_bits(),
        );
        if self.line != Some(setting) {
            uart.configure(setting);
            self.line = Some(setting);
            self.to_host_len = 0;
        }
        events.enter_bootloader = self.touch.update(coding.data_rate(), self.serial.dtr());

        // Host to Wio-E5.  Reading only what the UART can take leaves the
        // rest in the serial port, which then NAKs the host: flow control.
        let room = uart.tx_free().min(PACKET_SIZE as usize);
        if room > 0 {
            let mut buf = [0u8; PACKET_SIZE as usize];
            if let Ok(n) = self.serial.read(&mut buf[..room]) {
                uart.write(&buf[..n]);
                events.activity |= n > 0;
            }
        }

        // Wio-E5 to host.  With the port closed the host stops collecting,
        // the serial port reports WouldBlock and the UART side fills up and
        // drops; there is no flow control towards the Wio-E5.
        self.to_host_len += uart.read(&mut self.to_host[self.to_host_len..]);
        if self.to_host_len > 0
            && let Ok(n) = self.serial.write(&self.to_host[..self.to_host_len])
        {
            self.to_host.copy_within(n..self.to_host_len, 0);
            self.to_host_len -= n;
            events.activity = true;
        }

        // A debug session ends with the USB link even if the host never
        // sent DAP_Disconnect: let go of SWD and reset, so a probe on the
        // J2 header can use them, and drop a reply nobody will collect.
        let configured = self.configured();
        if self.was_configured && !configured {
            self.dap.suspend();
            self.dap_pending = 0;
        }
        self.was_configured = configured;
        if !configured {
            return events;
        }

        // CMSIS-DAP, one command in flight.
        if self.dap_pending == 0 {
            let mut request = [0u8; PACKET_SIZE as usize];
            if let Some(n) = self.dap_if.read_packet(&mut request) {
                self.dap_pending =
                    self.dap
                        .process_command(&request[..n], &mut self.dap_response, DapVersion::V2);
                events.activity = true;
            }
        }
        if self.dap_pending > 0
            && self
                .dap_if
                .write_packet(&self.dap_response[..self.dap_pending])
                .is_ok()
        {
            self.dap_pending = 0;
        }
        events
    }
}
