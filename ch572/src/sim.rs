//! Simulated hardware for the host tests: the CH572's USB device
//! controller with a host driving transactions into it, an SWD target,
//! and a UART.
//!
//! The controller model implements what the driver relies on, as the
//! datasheet (chapter 16) and WCH's interrupt handler describe it: the
//! response bits per endpoint, DMA into the endpoint buffers, toggle
//! checking on OUT, RB_UC_INT_BUSY NAKing while RB_UIF_TRANSFER is set,
//! and the two-step clear when a SETUP is pending behind a token.  It is
//! a model of a reading of the documentation, so it catches driver logic
//! errors, not mismatches between the documentation and the silicon.

use crate::bridge::UartPort;
use crate::linecode::UartSetting;
use crate::regs::*;
use crate::swd::SwdIo;
use crate::usb::{UsbRegs, off};
use std::cell::{RefCell, UnsafeCell};
use std::collections::VecDeque;
use std::rc::Rc;

// ---- USB controller ------------------------------------------------------

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Handshake {
    Ack,
    Nak,
    Stall,
    /// No answer: wrong address, or the endpoint direction is disabled.
    Timeout,
}

struct UsbState {
    regs: [u8; 0x40],
    dma: [u16; 4],
    /// RB_UIF_* bits 0 to 4.
    flags: u8,
    tog_ok: bool,
    /// MASK_UIS_TOKEN | MASK_UIS_ENDP of the pending transaction, or
    /// UIS_TOKEN_SETUP when none is pending.
    token: u8,
    setup_act: bool,
    rx_len: u8,
    attached: bool,
}

pub struct SimUsb {
    state: RefCell<UsbState>,
    buffers: UnsafeCell<[[u8; 128]; 4]>,
}

// The tests run each simulation on one thread.
unsafe impl Sync for SimUsb {}

impl SimUsb {
    pub fn new() -> Self {
        Self {
            state: RefCell::new(UsbState {
                regs: [0; 0x40],
                dma: [0; 4],
                flags: 0,
                tog_ok: false,
                token: UIS_TOKEN_SETUP,
                setup_act: false,
                rx_len: 0,
                attached: false,
            }),
            buffers: UnsafeCell::new([[0; 128]; 4]),
        }
    }

    fn buf(&self, ep: usize) -> *mut u8 {
        unsafe { (*self.buffers.get())[ep].as_mut_ptr() }
    }

    pub fn address(&self) -> u8 {
        self.state.borrow().regs[off::USB_DEV_AD] & MASK_USB_ADDR
    }

    fn reachable(&self, addr: u8) -> bool {
        let s = self.state.borrow();
        s.attached
            && s.regs[off::UDEV_CTRL] & RB_UD_PORT_EN != 0
            && s.regs[off::USB_DEV_AD] & MASK_USB_ADDR == addr
    }

    fn busy(&self) -> bool {
        let s = self.state.borrow();
        s.flags & RB_UIF_TRANSFER != 0 && s.regs[off::USB_CTRL] & RB_UC_INT_BUSY != 0
    }

    fn rx_enabled(&self, ep: usize) -> bool {
        let s = self.state.borrow();
        match ep {
            0 => true,
            1 => s.regs[off::UEP4_1_MOD] & RB_UEP1_RX_EN != 0,
            2 => s.regs[off::UEP2_3_MOD] & RB_UEP2_RX_EN != 0,
            3 => s.regs[off::UEP2_3_MOD] & RB_UEP3_RX_EN != 0,
            _ => false,
        }
    }

    fn tx_enabled(&self, ep: usize) -> bool {
        let s = self.state.borrow();
        match ep {
            0 => true,
            1 => s.regs[off::UEP4_1_MOD] & RB_UEP1_TX_EN != 0,
            2 => s.regs[off::UEP2_3_MOD] & RB_UEP2_TX_EN != 0,
            3 => s.regs[off::UEP2_3_MOD] & RB_UEP3_TX_EN != 0,
            _ => false,
        }
    }

    pub fn bus_reset(&self) {
        self.state.borrow_mut().flags |= RB_UIF_BUS_RST;
    }

    /// A SETUP transaction.  The controller takes SETUPs even while a
    /// previous transaction is still flagged.
    pub fn setup(&self, addr: u8, packet: [u8; 8]) -> Handshake {
        if !self.reachable(addr) {
            return Handshake::Timeout;
        }
        unsafe { std::ptr::copy_nonoverlapping(packet.as_ptr(), self.buf(0), 8) };
        let mut s = self.state.borrow_mut();
        s.setup_act = true;
        s.flags |= RB_UIF_TRANSFER;
        Handshake::Ack
    }

    pub fn out(&self, addr: u8, ep: usize, data: &[u8], data1: bool) -> Handshake {
        if !self.reachable(addr) || !self.rx_enabled(ep) {
            return Handshake::Timeout;
        }
        if self.busy() {
            return Handshake::Nak;
        }
        let ctrl = self.state.borrow().regs[off::uep_ctrl(ep)];
        match ctrl & MASK_UEP_R_RES {
            UEP_R_RES_NAK => return Handshake::Nak,
            UEP_R_RES_STALL => return Handshake::Stall,
            _ => {}
        }
        unsafe { std::ptr::copy_nonoverlapping(data.as_ptr(), self.buf(ep), data.len()) };
        let mut s = self.state.borrow_mut();
        s.rx_len = data.len() as u8;
        s.tog_ok = data1 == (ctrl & RB_UEP_R_TOG != 0);
        s.token = UIS_TOKEN_OUT | ep as u8;
        s.flags |= RB_UIF_TRANSFER;
        Handshake::Ack
    }

    /// An IN transaction: the packet and whether it was sent as DATA1.
    pub fn in_(&self, addr: u8, ep: usize) -> Result<(Vec<u8>, bool), Handshake> {
        if !self.reachable(addr) || !self.tx_enabled(ep) {
            return Err(Handshake::Timeout);
        }
        if self.busy() {
            return Err(Handshake::Nak);
        }
        let (ctrl, len) = {
            let s = self.state.borrow();
            (
                s.regs[off::uep_ctrl(ep)],
                s.regs[off::uep_t_len(ep)] as usize,
            )
        };
        match ctrl & MASK_UEP_T_RES {
            UEP_T_RES_NAK => return Err(Handshake::Nak),
            UEP_T_RES_STALL => return Err(Handshake::Stall),
            _ => {}
        }
        let base = if ep == 0 { 0 } else { 64 };
        let data = unsafe { std::slice::from_raw_parts(self.buf(ep).add(base), len) }.to_vec();
        let mut s = self.state.borrow_mut();
        s.token = UIS_TOKEN_IN | ep as u8;
        s.flags |= RB_UIF_TRANSFER;
        Ok((data, ctrl & RB_UEP_T_TOG != 0))
    }
}

impl UsbRegs for &SimUsb {
    fn read(&self, offset: usize) -> u8 {
        let s = self.state.borrow();
        match offset {
            o if o == off::USB_INT_FG => s.flags | (s.tog_ok as u8) << 6 | 0x20,
            o if o == off::USB_INT_ST => (s.setup_act as u8) << 7 | (s.tog_ok as u8) << 6 | s.token,
            o if o == off::USB_RX_LEN => s.rx_len,
            o if o == off::USB_MIS_ST => 0x20,
            o => s.regs[o],
        }
    }

    fn write(&self, offset: usize, value: u8) {
        let mut s = self.state.borrow_mut();
        if offset != off::USB_INT_FG {
            s.regs[offset] = value;
            return;
        }
        // Write-one-to-clear, with RB_UIF_TRANSFER cleared in two steps
        // when a SETUP is pending behind a token transaction.
        if value & RB_UIF_TRANSFER != 0 {
            if s.token != UIS_TOKEN_SETUP {
                s.token = UIS_TOKEN_SETUP;
                if !s.setup_act {
                    s.flags &= !RB_UIF_TRANSFER;
                }
            } else {
                s.setup_act = false;
                s.flags &= !RB_UIF_TRANSFER;
            }
        }
        s.flags &= !(value & 0x1D);
    }

    fn write16(&self, offset: usize, value: u16) {
        let n = (offset - off::uep_dma(0)) / UEP_STRIDE;
        self.state.borrow_mut().dma[n] = value;
    }

    fn buffer(&self, ep: usize) -> *mut u8 {
        self.buf(ep)
    }

    fn dma_address(&self, ep: usize) -> u16 {
        0x0100 + 0x80 * ep as u16
    }

    fn attach(&self, on: bool) {
        self.state.borrow_mut().attached = on;
    }
}

// ---- UART ----------------------------------------------------------------

#[derive(Default)]
pub struct FakeUart {
    pub setting: Option<UartSetting>,
    pub sent: Vec<u8>,
    pub incoming: VecDeque<u8>,
}

impl UartPort for FakeUart {
    fn configure(&mut self, setting: UartSetting) {
        self.setting = Some(setting);
        self.incoming.clear();
    }

    fn read(&mut self, buf: &mut [u8]) -> usize {
        let n = buf.len().min(self.incoming.len());
        for slot in &mut buf[..n] {
            *slot = self.incoming.pop_front().unwrap();
        }
        n
    }

    fn write(&mut self, data: &[u8]) -> usize {
        self.sent.extend_from_slice(data);
        data.len()
    }

    fn tx_free(&self) -> usize {
        256
    }
}

pub struct NoDelay;

impl embedded_hal::delay::DelayNs for NoDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

// ---- SWD target ----------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    /// Ignoring everything until a line reset.
    Lockout,
    Idle,
    Request,
    /// The turnaround after the request.
    TrnRequest,
    Ack(u8),
    ReadData(u8),
    ReadParity,
    /// The turnaround before write data.
    TrnWrite,
    WriteData(u8),
    WriteParity,
    /// The turnaround that ends a read or a WAIT/FAULT.
    TrnEnd,
}

pub struct TargetState {
    clk: bool,
    host_level: bool,
    host_drives: bool,
    target_drives: bool,
    target_level: bool,
    phase: Phase,
    ones: u32,
    request: u8,
    nbits: u8,
    ack: u8,
    data: u32,
    pub dpidr: u32,
    ctrl_stat: u32,
    pub select: u32,
    ap: [u32; 4],
    rdbuff: u32,
    /// WAIT answers to give before the next OK.
    pub waits: u32,
    /// Send the next read with its parity bit flipped.
    pub corrupt_parity: bool,
    pub reset_asserted: bool,
    pub transfers: u32,
    pub services: u32,
    pub contention: bool,
}

/// The SWD target; clones share one state so the test can look inside
/// after handing a clone to the DAP.
#[derive(Clone)]
pub struct SimTarget(pub Rc<RefCell<TargetState>>);

impl SimTarget {
    pub fn new() -> Self {
        SimTarget(Rc::new(RefCell::new(TargetState {
            clk: true,
            host_level: true,
            host_drives: false,
            target_drives: false,
            target_level: false,
            // Like a part fresh out of reset: nothing until a line reset.
            phase: Phase::Lockout,
            ones: 0,
            request: 0,
            nbits: 0,
            ack: 0,
            data: 0,
            // The STM32WLE5's SW-DP (RM0461 DP_DPIDR reset value).
            dpidr: 0x5BA0_2477,
            ctrl_stat: 0,
            select: 0,
            ap: [0; 4],
            rdbuff: 0,
            waits: 0,
            corrupt_parity: false,
            reset_asserted: false,
            transfers: 0,
            services: 0,
            contention: false,
        })))
    }
}

impl TargetState {
    fn line(&self) -> bool {
        if self.host_drives {
            self.host_level
        } else if self.target_drives {
            self.target_level
        } else {
            // SWDIO has a pull-up on the target side.
            true
        }
    }

    fn read_reg(&mut self) -> u32 {
        let ap = self.request & 0x02 != 0;
        let a = (self.request >> 3) & 0x03;
        if ap {
            // AP reads are posted: this returns the previous result.
            let previous = self.rdbuff;
            self.rdbuff = self.ap[a as usize];
            previous
        } else {
            match a {
                0 => self.dpidr,
                // Power-up requests (bits 30, 28) are acknowledged at once.
                1 => self.ctrl_stat | (self.ctrl_stat & 0x5000_0000) << 1,
                3 => self.rdbuff,
                _ => 0,
            }
        }
    }

    fn write_reg(&mut self, value: u32) {
        let ap = self.request & 0x02 != 0;
        let a = (self.request >> 3) & 0x03;
        if ap {
            self.ap[a as usize] = value;
        } else {
            match a {
                1 => self.ctrl_stat = value,
                2 => self.select = value,
                _ => {}
            }
        }
    }

    /// One rising edge of SWCLK: the end of a bit time.
    fn rising_edge(&mut self) {
        if self.host_drives && self.target_drives {
            self.contention = true;
        }
        let line = self.line();
        if self.host_drives {
            self.ones = if line { self.ones + 1 } else { 0 };
            if self.ones >= 50 {
                self.phase = Phase::Idle;
                self.target_drives = false;
                return;
            }
        }
        let read = self.request & 0x04 != 0;
        self.phase = match self.phase {
            Phase::Lockout => Phase::Lockout,
            Phase::Idle if self.host_drives && line => {
                self.request = 1;
                self.nbits = 1;
                Phase::Request
            }
            Phase::Idle => Phase::Idle,
            Phase::Request => {
                self.request |= (line as u8) << self.nbits;
                self.nbits += 1;
                if self.nbits < 8 {
                    Phase::Request
                } else {
                    let r = self.request;
                    let parity = (r >> 1 & 0x0F).count_ones() & 1 == 1;
                    let valid = r & 0x01 != 0
                        && r & 0x40 == 0
                        && r & 0x80 != 0
                        && parity == (r & 0x20 != 0);
                    if valid {
                        Phase::TrnRequest
                    } else {
                        Phase::Lockout
                    }
                }
            }
            Phase::TrnRequest => {
                self.ack = if self.waits > 0 {
                    self.waits -= 1;
                    0b010
                } else {
                    0b001
                };
                self.target_drives = true;
                self.target_level = self.ack & 1 != 0;
                Phase::Ack(1)
            }
            Phase::Ack(i) if i < 3 => {
                self.target_level = self.ack >> i & 1 != 0;
                Phase::Ack(i + 1)
            }
            Phase::Ack(_) if self.ack != 0b001 => {
                self.target_drives = false;
                Phase::TrnEnd
            }
            Phase::Ack(_) if read => {
                self.data = self.read_reg();
                self.target_level = self.data & 1 != 0;
                Phase::ReadData(1)
            }
            Phase::Ack(_) => {
                self.target_drives = false;
                Phase::TrnWrite
            }
            Phase::ReadData(i) if i < 32 => {
                self.target_level = self.data >> i & 1 != 0;
                Phase::ReadData(i + 1)
            }
            Phase::ReadData(_) => {
                let parity = self.data.count_ones() & 1 == 1;
                self.target_level = parity != self.corrupt_parity;
                self.corrupt_parity = false;
                Phase::ReadParity
            }
            Phase::ReadParity => {
                self.target_drives = false;
                Phase::TrnEnd
            }
            Phase::TrnEnd => {
                self.transfers += 1;
                Phase::Idle
            }
            Phase::TrnWrite => {
                self.data = 0;
                Phase::WriteData(0)
            }
            Phase::WriteData(i) => {
                self.data |= (line as u32) << i;
                if i < 31 {
                    Phase::WriteData(i + 1)
                } else {
                    Phase::WriteParity
                }
            }
            Phase::WriteParity => {
                if (self.data.count_ones() & 1 == 1) == line {
                    self.write_reg(self.data);
                }
                self.transfers += 1;
                Phase::Idle
            }
        };
    }
}

impl SwdIo for SimTarget {
    fn set_clk(&mut self, high: bool) {
        let mut s = self.0.borrow_mut();
        let rising = !s.clk && high;
        s.clk = high;
        if rising {
            s.rising_edge();
        }
    }

    fn set_dio(&mut self, high: bool) {
        self.0.borrow_mut().host_level = high;
    }

    fn read_dio(&mut self) -> bool {
        self.0.borrow().line()
    }

    fn dio_output(&mut self, output: bool) {
        self.0.borrow_mut().host_drives = output;
    }

    fn half_period(&mut self) {}

    fn set_frequency(&mut self, hz: u32) -> bool {
        hz > 0
    }

    fn connect(&mut self) {
        let mut s = self.0.borrow_mut();
        s.clk = true;
        s.host_level = true;
        s.host_drives = true;
    }

    fn release(&mut self) {
        self.0.borrow_mut().host_drives = false;
    }

    fn set_reset(&mut self, assert: bool) {
        self.0.borrow_mut().reset_asserted = assert;
    }

    fn levels(&mut self) -> (bool, bool, bool) {
        let s = self.0.borrow();
        (s.clk, s.line(), !s.reset_asserted)
    }

    fn service(&mut self) {
        self.0.borrow_mut().services += 1;
    }

    fn delay_us(&mut self, _us: u32) {
        self.service();
    }
}

#[cfg(test)]
mod tests;
