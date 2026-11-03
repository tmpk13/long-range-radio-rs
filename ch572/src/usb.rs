//! usb-device bus driver for the CH572's full-speed USB device controller.
//!
//! The controller (datasheet chapter 16) leaves every transaction to
//! firmware.  Each endpoint has a DMA buffer, a transmit length and a
//! control register whose response bits decide whether the next IN or
//! OUT is ACKed, NAKed or STALLed; one completion flag (RB_UIF_TRANSFER)
//! names the token and endpoint of the last transaction.  With
//! RB_UC_INT_BUSY set the controller NAKs everything until that flag is
//! cleared, which is what lets this driver run from a polling loop with
//! interrupts off.
//!
//! The handling follows WCH's CH572 USB examples (openwch/ch570 EVT,
//! USB/Device/COM): firmware flips the data toggles after each successful
//! transaction instead of using RB_UEP_AUTO_TOG, an endpoint is set to NAK
//! as its transaction completes, and a pending SETUP (RB_UIS_SETUP_ACT) is
//! handled after the token transaction it arrived behind, each with its
//! own clear of RB_UIF_TRANSFER.
//!
//! Endpoints 0 to 3 are used.  Endpoint 4 shares endpoint 0's DMA area and
//! has no automatic toggle, so it is left alone.  Endpoints 1 to 3 are all
//! configured with both directions and single buffers, which fixes each
//! one's layout at OUT data in its first 64 bytes and IN data in the next
//! 64, whichever directions the classes allocate.

use crate::regs::*;
use core::cell::RefCell;
use core::ptr::copy_nonoverlapping;
use core::sync::atomic::{Ordering, compiler_fence};
use critical_section::Mutex;
use usb_device::bus::{PollResult, UsbBus};
use usb_device::endpoint::{EndpointAddress, EndpointType};
use usb_device::{Result, UsbDirection, UsbError};

/// Endpoints 0 to 3.
pub const ENDPOINTS: usize = 4;
/// Largest packet on any endpoint.
pub const MAX_PACKET: usize = 64;

/// Register offsets from R8_USB_CTRL.
pub mod off {
    use crate::regs::*;

    pub const USB_CTRL: usize = 0;
    pub const UDEV_CTRL: usize = R8_UDEV_CTRL - R8_USB_CTRL;
    pub const USB_INT_EN: usize = R8_USB_INT_EN - R8_USB_CTRL;
    pub const USB_DEV_AD: usize = R8_USB_DEV_AD - R8_USB_CTRL;
    pub const USB_MIS_ST: usize = R8_USB_MIS_ST - R8_USB_CTRL;
    pub const USB_INT_FG: usize = R8_USB_INT_FG - R8_USB_CTRL;
    pub const USB_INT_ST: usize = R8_USB_INT_ST - R8_USB_CTRL;
    pub const USB_RX_LEN: usize = R8_USB_RX_LEN - R8_USB_CTRL;
    pub const UEP4_1_MOD: usize = R8_UEP4_1_MOD - R8_USB_CTRL;
    pub const UEP2_3_MOD: usize = R8_UEP2_3_MOD - R8_USB_CTRL;

    pub const fn uep_dma(n: usize) -> usize {
        R16_UEP0_DMA - R8_USB_CTRL + n * UEP_STRIDE
    }
    pub const fn uep_t_len(n: usize) -> usize {
        R8_UEP0_T_LEN - R8_USB_CTRL + n * UEP_STRIDE
    }
    pub const fn uep_ctrl(n: usize) -> usize {
        R8_UEP0_CTRL - R8_USB_CTRL + n * UEP_STRIDE
    }
}

/// Access to the controller, so the driver can run against a model of it
/// in host tests.
pub trait UsbRegs {
    /// Read the 8-bit register `offset` bytes above R8_USB_CTRL.
    fn read(&self, offset: usize) -> u8;
    fn write(&self, offset: usize, value: u8);
    fn write16(&self, offset: usize, value: u16);
    /// Endpoint `ep`'s DMA buffer: 64 bytes for endpoint 0, 128 for 1 to 3.
    fn buffer(&self, ep: usize) -> *mut u8;
    /// What R16_UEPn_DMA holds for that buffer: its low 16 address bits.
    fn dma_address(&self, ep: usize) -> u16;
    /// Connect the USB pins and the D+ pull-up (in R16_PIN_ALTERNATE,
    /// outside the USB block), or disconnect them.
    fn attach(&self, on: bool);
}

#[derive(Default)]
struct State {
    setup: [u8; 8],
    setup_ready: bool,
    /// OUT packets received and not yet read, one bit per endpoint.
    rx_ready: u8,
    rx_len: [u8; ENDPOINTS],
    /// IN packets handed to the controller and not yet collected.
    in_busy: u8,
    /// Address from a SET_ADDRESS, applied when its status stage completes.
    pending_address: Option<u8>,
}

pub struct Ch572Usb<R> {
    regs: R,
    alloc_in: u8,
    alloc_out: u8,
    state: Mutex<RefCell<State>>,
}

impl<R: UsbRegs> Ch572Usb<R> {
    pub fn new(regs: R) -> Self {
        Self {
            regs,
            alloc_in: 0,
            alloc_out: 0,
            state: Mutex::new(RefCell::new(State::default())),
        }
    }

    fn in_buf(&self, n: usize) -> *mut u8 {
        let base = self.regs.buffer(n);
        if n == 0 {
            base
        } else {
            // SAFETY: endpoints 1 to 3 have 128-byte buffers.
            unsafe { base.add(MAX_PACKET) }
        }
    }

    fn ctrl(&self, n: usize) -> u8 {
        self.regs.read(off::uep_ctrl(n))
    }

    fn set_ctrl(&self, n: usize, value: u8) {
        self.regs.write(off::uep_ctrl(n), value);
    }

    /// Control register value after a bus reset: OUT endpoints accept,
    /// IN endpoints have nothing to send, toggles at DATA0.
    fn idle_ctrl(&self, n: usize) -> u8 {
        let out = if self.alloc_out & (1 << n) != 0 {
            UEP_R_RES_ACK
        } else {
            UEP_R_RES_NAK
        };
        out | UEP_T_RES_NAK
    }

    fn write_address(&self, addr: u8) {
        let ad = self.regs.read(off::USB_DEV_AD);
        self.regs.write(
            off::USB_DEV_AD,
            (ad & !MASK_USB_ADDR) | (addr & MASK_USB_ADDR),
        );
    }

    /// An OUT transaction completed on endpoint `n`.
    fn out_done(&self, s: &mut State, n: usize, toggle_ok: bool) {
        if !toggle_ok {
            // A retransmission of a packet already taken (the host missed
            // our ACK).  The controller ACKed it again; drop it.
            return;
        }
        let ctrl = self.ctrl(n);
        // NAK further OUTs until the class has read this packet.
        self.set_ctrl(n, ((ctrl ^ RB_UEP_R_TOG) & !MASK_UEP_R_RES) | UEP_R_RES_NAK);
        s.rx_len[n] = self.regs.read(off::USB_RX_LEN);
        s.rx_ready |= 1 << n;
    }

    /// An IN transaction completed on endpoint `n`.
    fn in_done(&self, s: &mut State, n: usize) {
        let ctrl = self.ctrl(n);
        // NAK further INs until the class writes again, or the host would
        // be sent the same buffer a second time.
        self.set_ctrl(n, ((ctrl ^ RB_UEP_T_TOG) & !MASK_UEP_T_RES) | UEP_T_RES_NAK);
        s.in_busy &= !(1 << n);
        if n == 0
            && let Some(addr) = s.pending_address.take()
        {
            // The status stage of SET_ADDRESS just completed: the new
            // address applies from now on.  usb-device sets it too, but only
            // if it sees this completion, which it skips when a SETUP
            // arrives in the same poll.
            self.write_address(addr);
        }
    }

    /// A SETUP packet arrived on endpoint 0.
    fn setup_done(&self, s: &mut State) {
        let mut setup = [0u8; 8];
        compiler_fence(Ordering::SeqCst);
        // SAFETY: the endpoint 0 buffer holds at least 8 bytes.
        unsafe { copy_nonoverlapping(self.regs.buffer(0), setup.as_mut_ptr(), 8) };
        s.setup = setup;
        s.setup_ready = true;
        // A SETUP starts a new control transfer; whatever endpoint 0 still
        // held from the previous one is stale.
        s.rx_ready &= !1;
        s.in_busy &= !1;
        let (request_type, request) = (setup[0], setup[1]);
        let value = u16::from_le_bytes([setup[2], setup[3]]);
        s.pending_address =
            (request_type == 0x00 && request == 0x05).then_some(value as u8 & MASK_USB_ADDR);
        if request_type == 0x00 && request == 0x09 {
            // SET_CONFIGURATION puts every data endpoint back to DATA0.
            for n in 1..ENDPOINTS {
                let ctrl = self.ctrl(n);
                self.set_ctrl(n, ctrl & !(RB_UEP_R_TOG | RB_UEP_T_TOG));
            }
        }
        // Data and status stages start at DATA1.  Accept OUT; IN waits
        // until usb-device writes the reply.
        self.set_ctrl(
            0,
            RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK,
        );
    }
}

impl<R: UsbRegs + Sync> UsbBus for Ch572Usb<R> {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        _interval: u8,
    ) -> Result<EndpointAddress> {
        if max_packet_size as usize > MAX_PACKET {
            return Err(UsbError::EndpointMemoryOverflow);
        }
        if matches!(ep_type, EndpointType::Isochronous { .. }) {
            return Err(UsbError::Unsupported);
        }
        let taken = match ep_dir {
            UsbDirection::In => &mut self.alloc_in,
            UsbDirection::Out => &mut self.alloc_out,
        };
        let n = match ep_addr {
            Some(addr) => {
                let n = addr.index();
                if n >= ENDPOINTS || *taken & (1 << n) != 0 {
                    return Err(UsbError::InvalidEndpoint);
                }
                n
            }
            None => (1..ENDPOINTS)
                .find(|&n| *taken & (1 << n) == 0)
                .ok_or(UsbError::EndpointOverflow)?,
        };
        *taken |= 1 << n;
        Ok(EndpointAddress::from_parts(n, ep_dir))
    }

    /// The sequence of WCH's USB_DeviceInit.
    fn enable(&mut self) {
        let r = &self.regs;
        r.write(off::USB_CTRL, 0);
        r.write(off::UEP4_1_MOD, RB_UEP1_RX_EN | RB_UEP1_TX_EN);
        r.write(
            off::UEP2_3_MOD,
            RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_RX_EN | RB_UEP3_TX_EN,
        );
        for n in 0..ENDPOINTS {
            r.write16(off::uep_dma(n), r.dma_address(n));
            r.write(off::uep_ctrl(n), self.idle_ctrl(n));
        }
        r.write(off::USB_DEV_AD, 0);
        r.write(
            off::USB_CTRL,
            RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN,
        );
        r.attach(true);
        r.write(off::USB_INT_FG, 0xFF);
        r.write(off::UDEV_CTRL, RB_UD_PD_DIS | RB_UD_PORT_EN);
        // The flags are polled; the USB interrupt itself stays disabled in
        // the core, so these only select which events raise a flag.
        r.write(
            off::USB_INT_EN,
            RB_UIE_SUSPEND | RB_UIE_BUS_RST | RB_UIE_TRANSFER,
        );
    }

    fn reset(&self) {
        critical_section::with(|cs| {
            *self.state.borrow_ref_mut(cs) = State::default();
            self.regs.write(off::USB_DEV_AD, 0);
            for n in 0..ENDPOINTS {
                self.set_ctrl(n, self.idle_ctrl(n));
            }
            self.regs.write(off::USB_INT_FG, RB_UIF_TRANSFER);
        });
    }

    fn set_device_address(&self, addr: u8) {
        self.write_address(addr);
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        let n = ep_addr.index();
        if ep_addr.direction() != UsbDirection::In
            || n >= ENDPOINTS
            || self.alloc_in & (1 << n) == 0
        {
            return Err(UsbError::InvalidEndpoint);
        }
        if buf.len() > MAX_PACKET {
            return Err(UsbError::BufferOverflow);
        }
        critical_section::with(|cs| {
            let mut s = self.state.borrow_ref_mut(cs);
            if s.in_busy & (1 << n) != 0 {
                return Err(UsbError::WouldBlock);
            }
            // SAFETY: at most MAX_PACKET bytes into a 64-byte IN area.
            unsafe { copy_nonoverlapping(buf.as_ptr(), self.in_buf(n), buf.len()) };
            compiler_fence(Ordering::SeqCst);
            self.regs.write(off::uep_t_len(n), buf.len() as u8);
            let ctrl = self.ctrl(n);
            self.set_ctrl(n, (ctrl & !MASK_UEP_T_RES) | UEP_T_RES_ACK);
            s.in_busy |= 1 << n;
            Ok(buf.len())
        })
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        let n = ep_addr.index();
        if ep_addr.direction() != UsbDirection::Out
            || n >= ENDPOINTS
            || self.alloc_out & (1 << n) == 0
        {
            return Err(UsbError::InvalidEndpoint);
        }
        critical_section::with(|cs| {
            let mut s = self.state.borrow_ref_mut(cs);
            if n == 0 && s.setup_ready {
                if buf.len() < 8 {
                    return Err(UsbError::BufferOverflow);
                }
                buf[..8].copy_from_slice(&s.setup);
                s.setup_ready = false;
                return Ok(8);
            }
            if s.rx_ready & (1 << n) == 0 {
                return Err(UsbError::WouldBlock);
            }
            let len = s.rx_len[n] as usize;
            if buf.len() < len {
                return Err(UsbError::BufferOverflow);
            }
            compiler_fence(Ordering::SeqCst);
            // SAFETY: rx_len came from R8_USB_RX_LEN, at most 64.
            unsafe { copy_nonoverlapping(self.regs.buffer(n), buf.as_mut_ptr(), len) };
            s.rx_ready &= !(1 << n);
            let ctrl = self.ctrl(n);
            if ctrl & MASK_UEP_R_RES != UEP_R_RES_STALL {
                self.set_ctrl(n, (ctrl & !MASK_UEP_R_RES) | UEP_R_RES_ACK);
            }
            Ok(len)
        })
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        let n = ep_addr.index();
        if n >= ENDPOINTS {
            return;
        }
        critical_section::with(|cs| {
            let mut s = self.state.borrow_ref_mut(cs);
            let ctrl = self.ctrl(n);
            let t_res = ctrl & MASK_UEP_T_RES;
            let r_res = ctrl & MASK_UEP_R_RES;
            let new = match (ep_addr.direction(), stalled) {
                (UsbDirection::In, true) => {
                    // A packet still armed is discarded with the stall.
                    s.in_busy &= !(1 << n);
                    (ctrl & !MASK_UEP_T_RES) | UEP_T_RES_STALL
                }
                (UsbDirection::Out, true) => (ctrl & !MASK_UEP_R_RES) | UEP_R_RES_STALL,
                // Every SETUP rewrites endpoint 0's control register,
                // toggles included, and usb-device unstalls endpoint 0 after
                // each one; only lift a stall that is still in place.
                (UsbDirection::In, false) if n == 0 => {
                    if t_res == UEP_T_RES_STALL {
                        (ctrl & !MASK_UEP_T_RES) | UEP_T_RES_NAK
                    } else {
                        ctrl
                    }
                }
                (UsbDirection::Out, false) if n == 0 => {
                    if r_res == UEP_R_RES_STALL {
                        (ctrl & !MASK_UEP_R_RES) | UEP_R_RES_ACK
                    } else {
                        ctrl
                    }
                }
                // CLEAR_FEATURE(ENDPOINT_HALT): the toggle goes back to
                // DATA0 too.  A packet already handed over stays armed, so
                // the class still sees it complete.
                (UsbDirection::In, false) => {
                    let res = if s.in_busy & (1 << n) != 0 {
                        UEP_T_RES_ACK
                    } else {
                        UEP_T_RES_NAK
                    };
                    (ctrl & !(MASK_UEP_T_RES | RB_UEP_T_TOG)) | res
                }
                (UsbDirection::Out, false) => {
                    let res = if s.rx_ready & (1 << n) != 0 {
                        UEP_R_RES_NAK
                    } else {
                        UEP_R_RES_ACK
                    };
                    (ctrl & !(MASK_UEP_R_RES | RB_UEP_R_TOG)) | res
                }
            };
            self.set_ctrl(n, new);
        });
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        let n = ep_addr.index();
        if n >= ENDPOINTS {
            return false;
        }
        let ctrl = self.ctrl(n);
        match ep_addr.direction() {
            UsbDirection::In => ctrl & MASK_UEP_T_RES == UEP_T_RES_STALL,
            UsbDirection::Out => ctrl & MASK_UEP_R_RES == UEP_R_RES_STALL,
        }
    }

    fn suspend(&self) {}

    fn resume(&self) {}

    fn poll(&self) -> PollResult {
        critical_section::with(|cs| {
            let mut s = self.state.borrow_ref_mut(cs);
            let r = &self.regs;
            let fg = r.read(off::USB_INT_FG);
            if fg & RB_UIF_BUS_RST != 0 {
                r.write(off::USB_INT_FG, RB_UIF_BUS_RST);
                return PollResult::Reset;
            }
            let mut ep_in_complete = 0u16;
            if fg & RB_UIF_TRANSFER != 0 {
                let st = r.read(off::USB_INT_ST);
                let token = st & MASK_UIS_TOKEN;
                let mut handled = false;
                // UIS_TOKEN_SETUP doubles as "no token pending".
                if token != UIS_TOKEN_SETUP {
                    let n = (st & MASK_UIS_ENDP) as usize;
                    if n < ENDPOINTS {
                        match token {
                            UIS_TOKEN_OUT => self.out_done(&mut s, n, fg & RB_U_TOG_OK != 0),
                            UIS_TOKEN_IN => {
                                self.in_done(&mut s, n);
                                ep_in_complete |= 1 << n;
                            }
                            _ => {}
                        }
                    }
                    r.write(off::USB_INT_FG, RB_UIF_TRANSFER);
                    handled = true;
                }
                if r.read(off::USB_INT_ST) & RB_UIS_SETUP_ACT != 0 {
                    self.setup_done(&mut s);
                    ep_in_complete &= !1;
                    r.write(off::USB_INT_FG, RB_UIF_TRANSFER);
                    handled = true;
                }
                if !handled {
                    // Neither a token nor a SETUP: clear the flag anyway, or
                    // RB_UC_INT_BUSY NAKs everything from here on.
                    r.write(off::USB_INT_FG, RB_UIF_TRANSFER);
                }
            } else if fg & RB_UIF_SUSPEND != 0 {
                r.write(off::USB_INT_FG, RB_UIF_SUSPEND);
                return if r.read(off::USB_MIS_ST) & RB_UMS_SUSPEND != 0 {
                    PollResult::Suspend
                } else {
                    PollResult::Resume
                };
            }
            // OUT and SETUP are reported until read, IN completions once.
            let ep_out = s.rx_ready as u16;
            let ep_setup = s.setup_ready as u16;
            if ep_out | ep_in_complete | ep_setup == 0 {
                PollResult::None
            } else {
                PollResult::Data {
                    ep_out,
                    ep_in_complete,
                    ep_setup,
                }
            }
        })
    }
}
