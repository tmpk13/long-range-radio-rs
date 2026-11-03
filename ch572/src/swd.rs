//! Serial Wire Debug, bit-banged, and its connection to dap-rs.
//!
//! The bit sequences follow ARM's CMSIS-DAP reference implementation
//! (SW_DP.c): the probe writes SWDIO while SWCLK is low and the target
//! samples it on the rising edge; the target changes SWDIO after a rising
//! edge and the probe samples it while SWCLK is low.  Turnaround is one
//! cycle and there is no forced data phase after WAIT or FAULT, the only
//! SWD configuration dap-rs accepts (and the one probe-rs uses).

use dap_rs::swd::{self, APnDP, DPRegister, RnW};
use dap_rs::swj::{self, Pins};
use dap_rs::{jtag, swo};

/// Pin-level access to the target's debug port.
pub trait SwdIo {
    fn set_clk(&mut self, high: bool);
    fn set_dio(&mut self, high: bool);
    fn read_dio(&mut self) -> bool;
    /// SWDIO direction: true drives it, false lets the target drive.
    fn dio_output(&mut self, output: bool);
    /// Wait half a clock period at the configured rate.
    fn half_period(&mut self);
    /// Pick the clock rate; false for 0 Hz.
    fn set_frequency(&mut self, hz: u32) -> bool;
    /// Take SWCLK and SWDIO as outputs, both high (the SWD idle state).
    fn connect(&mut self);
    /// Release SWCLK, SWDIO and nRESET, so an external probe on the debug
    /// header can take over.
    fn release(&mut self);
    /// Drive nRESET low (true) or release it (false).
    fn set_reset(&mut self, assert: bool);
    /// Current (SWCLK, SWDIO, nRESET) levels.
    fn levels(&mut self) -> (bool, bool, bool);
    /// Housekeeping that must not wait for a long DAP command to finish:
    /// called before every transfer and while waiting.
    fn service(&mut self);
    /// Busy-wait, calling `service` along the way.
    fn delay_us(&mut self, us: u32);
}

fn write_bit<I: SwdIo>(io: &mut I, bit: bool) {
    io.set_dio(bit);
    io.set_clk(false);
    io.half_period();
    io.set_clk(true);
    io.half_period();
}

fn read_bit<I: SwdIo>(io: &mut I) -> bool {
    io.set_clk(false);
    io.half_period();
    let bit = io.read_dio();
    io.set_clk(true);
    io.half_period();
    bit
}

fn clock_cycle<I: SwdIo>(io: &mut I) {
    io.set_clk(false);
    io.half_period();
    io.set_clk(true);
    io.half_period();
}

/// Take SWDIO back after the target had it, idling high.  The level is
/// set before the direction so the line never glitches low.
fn reclaim_dio<I: SwdIo>(io: &mut I) {
    io.set_dio(true);
    io.dio_output(true);
}

const ACK_OK: u8 = 0b001;
const ACK_WAIT: u8 = 0b010;
const ACK_FAULT: u8 = 0b100;

/// One SWD transaction: the 8-bit request (start, APnDP, RnW, A[3:2],
/// parity, stop, park), turnaround, the 3-bit acknowledge, then for OK the
/// 32 data bits and their parity in whichever direction RnW says.
pub fn transfer<I: SwdIo>(io: &mut I, request: u8, wdata: u32) -> swd::Result<u32> {
    io.service();
    let read = request & (1 << 2) != 0;
    for i in 0..8 {
        write_bit(io, request >> i & 1 != 0);
    }
    io.dio_output(false);
    clock_cycle(io);
    let mut ack = 0u8;
    for i in 0..3 {
        ack |= (read_bit(io) as u8) << i;
    }
    match ack {
        ACK_OK if read => {
            let mut value = 0u32;
            let mut parity = false;
            for i in 0..32 {
                let bit = read_bit(io);
                value |= (bit as u32) << i;
                parity ^= bit;
            }
            let parity_bit = read_bit(io);
            clock_cycle(io);
            reclaim_dio(io);
            if parity_bit == parity {
                Ok(value)
            } else {
                Err(swd::Error::BadParity)
            }
        }
        ACK_OK => {
            clock_cycle(io);
            io.dio_output(true);
            let mut parity = false;
            for i in 0..32 {
                let bit = wdata >> i & 1 != 0;
                write_bit(io, bit);
                parity ^= bit;
            }
            write_bit(io, parity);
            io.set_dio(true);
            Ok(0)
        }
        ACK_WAIT | ACK_FAULT => {
            clock_cycle(io);
            reclaim_dio(io);
            Err(if ack == ACK_WAIT {
                swd::Error::AckWait
            } else {
                swd::Error::AckFault
            })
        }
        _ => {
            // No valid acknowledge: clock through where a data phase and
            // the turnaround would be, so a target that did see the
            // request is not left mid-transfer.
            for _ in 0..(1 + 32 + 1) {
                clock_cycle(io);
            }
            reclaim_dio(io);
            Err(if ack == 0b111 {
                swd::Error::AckProtocol
            } else {
                swd::Error::AckUnknown(ack)
            })
        }
    }
}

/// Clock out `nbits` of `data`, least significant bit of byte 0 first.
pub fn write_bits<I: SwdIo>(io: &mut I, nbits: usize, data: &[u8]) {
    io.dio_output(true);
    for i in 0..nbits {
        write_bit(io, data[i / 8] >> (i % 8) & 1 != 0);
    }
}

/// Clock in `nbits` into `data`, least significant bit of byte 0 first.
pub fn read_bits<I: SwdIo>(io: &mut I, nbits: usize, data: &mut [u8]) {
    io.dio_output(false);
    data[..nbits.div_ceil(8)].fill(0);
    for i in 0..nbits {
        if read_bit(io) {
            data[i / 8] |= 1 << (i % 8);
        }
    }
    reclaim_dio(io);
}

// ---- dap-rs glue ---------------------------------------------------------
//
// dap-rs moves one pin owner between three states: `Deps` for direct pin
// commands (SWJ_Pins, SWJ_Sequence), `Swd` for transfers and `Jtag`.  The
// conversions happen on nearly every command, so they must not touch the
// pins beyond making sure SWD owns them.

pub struct Deps<I>(pub I);
pub struct Swd<I>(I);
pub struct Jtag<I>(I);

impl<I: SwdIo> From<Swd<I>> for Deps<I> {
    fn from(swd: Swd<I>) -> Self {
        Deps(swd.0)
    }
}

impl<I: SwdIo> From<Jtag<I>> for Deps<I> {
    fn from(jtag: Jtag<I>) -> Self {
        Deps(jtag.0)
    }
}

impl<I: SwdIo> From<Deps<I>> for Swd<I> {
    fn from(deps: Deps<I>) -> Self {
        let mut io = deps.0;
        io.connect();
        Swd(io)
    }
}

impl<I: SwdIo> From<Deps<I>> for Jtag<I> {
    fn from(deps: Deps<I>) -> Self {
        Jtag(deps.0)
    }
}

impl<I: SwdIo> swj::Dependencies<Swd<I>, Jtag<I>> for Deps<I> {
    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        let io = &mut self.0;
        if mask.intersects(Pins::SWCLK | Pins::SWDIO) {
            io.connect();
        }
        if mask.contains(Pins::SWCLK) {
            io.set_clk(output.contains(Pins::SWCLK));
        }
        if mask.contains(Pins::SWDIO) {
            io.set_dio(output.contains(Pins::SWDIO));
        }
        if mask.contains(Pins::NRESET) {
            // nRESET high in `output` means released.
            io.set_reset(!output.contains(Pins::NRESET));
        }
        // Wait for the selected pins to read back as requested, e.g. for
        // the reset line's RC (10k, 100 nF) to charge after release.
        let wanted = output & mask;
        let mut waited = 0;
        loop {
            let now = pins_from(io.levels());
            if now & mask == wanted || waited >= wait_us {
                return now;
            }
            io.delay_us(1);
            waited += 1;
        }
    }

    fn process_swj_sequence(&mut self, data: &[u8], nbits: usize) {
        self.0.connect();
        write_bits(&mut self.0, nbits, data);
    }

    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        self.0.set_frequency(max_frequency)
    }

    fn high_impedance_mode(&mut self) {
        self.0.release();
    }
}

fn pins_from((clk, dio, nreset): (bool, bool, bool)) -> Pins {
    let mut pins = Pins::empty();
    pins.set(Pins::SWCLK, clk);
    pins.set(Pins::SWDIO, dio);
    pins.set(Pins::NRESET, nreset);
    pins
}

impl<I: SwdIo> swd::Swd<Deps<I>> for Swd<I> {
    const AVAILABLE: bool = true;

    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        transfer(&mut self.0, swd::make_request(apndp, RnW::R, a), 0)
    }

    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> swd::Result<()> {
        transfer(&mut self.0, swd::make_request(apndp, RnW::W, a), data).map(|_| ())
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.0.set_frequency(max_frequency)
    }

    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
        write_bits(&mut self.0, num_bits, data);
        Ok(())
    }

    fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        read_bits(&mut self.0, num_bits, data);
        Ok(())
    }
}

impl<I: SwdIo> jtag::Jtag<Deps<I>> for Jtag<I> {
    const AVAILABLE: bool = false;

    fn sequences(&mut self, _data: &[u8], _rxbuf: &mut [u8]) -> u32 {
        0
    }

    fn set_clock(&mut self, _max_frequency: u32) -> bool {
        false
    }
}

/// No SWO on this board; dap-rs takes `Option<NoSwo>` and gets `None`.
pub enum NoSwo {}

impl swo::Swo for NoSwo {
    fn set_transport(&mut self, _transport: swo::SwoTransport) {
        match *self {}
    }
    fn set_mode(&mut self, _mode: swo::SwoMode) {
        match *self {}
    }
    fn set_baudrate(&mut self, _baudrate: u32) -> u32 {
        match *self {}
    }
    fn set_control(&mut self, _control: swo::SwoControl) {
        match *self {}
    }
    fn polling_data(&mut self, _buf: &mut [u8]) -> u32 {
        match *self {}
    }
    fn streaming_data(&mut self) {
        match *self {}
    }
    fn is_active(&self) -> bool {
        match *self {}
    }
    fn bytes_available(&self) -> u32 {
        match *self {}
    }
    fn buffer_size(&self) -> u32 {
        match *self {}
    }
    fn support(&self) -> swo::SwoSupport {
        match *self {}
    }
    fn status(&mut self) -> swo::SwoStatus {
        match *self {}
    }
}

/// dap-rs reports host connect/run status through this; the bridge's
/// LEDs show USB and traffic state instead.
pub struct NoLeds;

impl dap_rs::dap::DapLeds for NoLeds {
    fn react_to_host_status(&mut self, _host_status: dap_rs::dap::HostStatus) {}
}
