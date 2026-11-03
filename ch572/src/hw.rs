//! The CH572D itself: clock, pins, UART, USB registers, SWD pins, timing,
//! watchdog and resets.  Everything here touches registers; the logic it
//! serves lives in the hardware-independent modules.
//!
//! CH572D pins on the Forest Sensor Platform (usb.kicad_sch):
//!
//! | Pin  | Net          | Use                                            |
//! |------|--------------|------------------------------------------------|
//! | PA0  | D-           | USB (the CH572's own SWDIO when USB is off)    |
//! | PA1  | D+           | USB (the CH572's own SWCLK when USB is off)    |
//! | PA2  | D11 cathode  | activity LED, active low (anode via 10k)       |
//! | PA3  | D10 cathode  | USB LED, active low (anode via 10k)            |
//! | PA4  | USB_VOUT_EN  | U7, 5 V out onto VBUS: held low, never used    |
//! | PA5  | SWD-WIO-CLK  | Wio-E5 SWCLK                                   |
//! | PA6  | WIO-RST      | Wio-E5 reset (10k pull-up, 100 nF)             |
//! | PA7  | SWD-WIO-IO   | Wio-E5 SWDIO                                   |
//! | PA8  | UART-WIO-RX  | UART TXD (remap 101b), into Wio-E5 PA3         |
//! | PA9  | UART-WIO-TX  | UART RXD (remap 101b), from Wio-E5 PA2         |
//! | PA10 | CH_USR_BTN   | test pad TP7, pulled up, unused                |
//! | PA11 | CH_GPIO      | test pad TP9, pulled up, unused                |
//!
//! The Wio-E5 side of SWD and reset is shared with the J2 debug header,
//! so those pins float whenever no debug session holds them.

use crate::FSYS_HZ;
use crate::bridge::UartPort;
use crate::linecode::UartSetting;
use crate::regs::*;
use crate::ring::Ring;
use crate::swd::SwdIo;
use crate::usb::UsbRegs;
use core::cell::{RefCell, UnsafeCell};
use core::ptr::{read_volatile, write_volatile};
use critical_section::Mutex;

const LED_ACT: u32 = 1 << 2;
const LED_USB: u32 = 1 << 3;
const VOUT_EN: u32 = 1 << 4;
const SWCLK: u32 = 1 << 5;
const NRESET: u32 = 1 << 6;
const SWDIO: u32 = 1 << 7;
const TXD: u32 = 1 << 8;
const RXD: u32 = 1 << 9;
const TP7: u32 = 1 << 10;
const TP9: u32 = 1 << 11;

/// R16_PIN_ALTERNATE_H code selecting PA8 for TXD and PA9 for RXD.
const UART_REMAP_PA8_PA9: u16 = 0b101;

/// SysTick ticks (HCLK) per microsecond.
pub const TICKS_PER_US: u32 = FSYS_HZ / 1_000_000;

#[inline(always)]
fn rd8(addr: usize) -> u8 {
    unsafe { read_volatile(addr as *const u8) }
}
#[inline(always)]
fn wr8(addr: usize, value: u8) {
    unsafe { write_volatile(addr as *mut u8, value) }
}
#[inline(always)]
fn rd16(addr: usize) -> u16 {
    unsafe { read_volatile(addr as *const u16) }
}
#[inline(always)]
fn wr16(addr: usize, value: u16) {
    unsafe { write_volatile(addr as *mut u16, value) }
}
#[inline(always)]
fn rd32(addr: usize) -> u32 {
    unsafe { read_volatile(addr as *const u32) }
}
#[inline(always)]
fn wr32(addr: usize, value: u32) {
    unsafe { write_volatile(addr as *mut u32, value) }
}

#[inline(always)]
fn nop() {
    unsafe { core::arch::asm!("nop", options(nomem, nostack, preserves_flags)) }
}

/// `fence.i`, which WCH's SAFEOPERATE uses around the safe-access
/// sequence.  Spelled as its encoding: the imc target lacks Zifencei.
#[inline(always)]
fn fence_i() {
    unsafe { core::arch::asm!(".word 0x0000100f", options(nostack, preserves_flags)) }
}

/// Open the safe-access window: registers the datasheet marks RWA or WA
/// accept writes for about 112 Fsys cycles after the 0x57, 0xA8 signature
/// (datasheet 4.2).  Same sequence as WCH's sys_safe_access_enable, minus
/// the interrupt masking: this firmware never enables interrupts.
#[inline(always)]
fn safe_on() {
    fence_i();
    wr8(R8_SAFE_ACCESS_SIG, SAFE_ACCESS_SIG1);
    wr8(R8_SAFE_ACCESS_SIG, SAFE_ACCESS_SIG2);
    fence_i();
}

#[inline(always)]
fn safe_off() {
    wr8(R8_SAFE_ACCESS_SIG, SAFE_ACCESS_SIG0);
    fence_i();
}

/// R8_CLK_SYS_CFG for Fsys = PLL / 6 = 100 MHz (WCH's
/// CLK_SOURCE_HSE_PLL_100MHz).
const CLK_PLL_DIV6: u8 = 0x40 | 6;

/// Switch Fsys to the PLL at [`FSYS_HZ`], following WCH's SetSysClock for
/// a PLL source.  The PLL also feeds USB its 48 MHz.
///
/// Runs from RAM, as WCH runs SetSysClock (`__HIGH_CODE`): it rewrites
/// the flash timing (R8_FLASH_CFG, R8_FLASH_SCK), and code fetched from
/// flash must not run across that change.  Everything it calls is
/// inlined, so nothing executes from flash in between.
#[unsafe(link_section = ".highcode")]
#[inline(never)]
pub fn init_clocks() {
    // The crystal runs from reset; restart it as WCH does if something
    // (the ROM bootloader) stopped it.
    if rd8(R8_HFCK_PWR_CTRL) & RB_CLK_XT32M_PON == 0 {
        let tune = rd8(R8_XT32M_TUNE);
        safe_on();
        wr8(R8_XT32M_TUNE, tune | 0x03);
        wr8(R8_HFCK_PWR_CTRL, rd8(R8_HFCK_PWR_CTRL) | RB_CLK_XT32M_PON);
        safe_off();
        let cfg = rd8(R8_CLK_SYS_CFG);
        safe_on();
        wr8(R8_CLK_SYS_CFG, cfg | 0xC0);
        safe_off();
        for _ in 0..9 {
            nop();
        }
        safe_on();
        wr8(R8_CLK_SYS_CFG, cfg);
        wr8(R8_XT32M_TUNE, tune);
        safe_off();
    }
    let pll_was_off = rd8(R8_HFCK_PWR_CTRL) & RB_CLK_PLL_PON == 0;
    safe_on();
    wr8(R8_HFCK_PWR_CTRL, rd8(R8_HFCK_PWR_CTRL) | RB_CLK_PLL_PON);
    wr8(R8_FLASH_CFG, 0x01);
    wr8(R8_FLASH_SCK, rd8(R8_FLASH_SCK) | 0x10);
    safe_off();
    if pll_was_off {
        // WCH switches straight over because its PLL is on from reset;
        // give a PLL that was off ~2 ms (at the 6.4 MHz reset clock) to
        // lock.
        for _ in 0..4_000 {
            nop();
        }
    }
    safe_on();
    wr8(R8_SLP_POWER_CTRL, rd8(R8_SLP_POWER_CTRL) | 0x40);
    wr8(R8_CLK_SYS_CFG, CLK_PLL_DIV6);
    safe_off();
}

/// Start SysTick counting HCLK, free-running upwards.
pub fn systick_start() {
    wr32(R32_STK_CTLR, STK_STCLK | STK_STE);
}

/// SysTick count: [`TICKS_PER_US`] per microsecond, wrapping every ~43 s.
#[inline(always)]
pub fn ticks() -> u32 {
    rd32(R32_STK_CNTL)
}

/// Busy-wait `ms` milliseconds, keeping the watchdog and UART serviced.
pub fn delay_ms(ms: u32) {
    let start = ticks();
    let span = ms * 1_000 * TICKS_PER_US;
    while ticks().wrapping_sub(start) < span {
        service();
    }
}

/// Watchdog timeout reset: R8_WDOG_COUNT steps every 131072 / Fsys
/// (1.3 ms) and resets the chip when it wraps, ~335 ms after a feed.
pub fn watchdog_start() {
    watchdog_feed();
    safe_on();
    wr8(R8_RST_WDOG_CTRL, RB_WDOG_RST_EN);
    safe_off();
}

#[inline(always)]
pub fn watchdog_feed() {
    wr8(R8_WDOG_COUNT, 0);
}

/// What has to keep running even inside a long DAP command.
pub fn service() {
    watchdog_feed();
    uart_service();
}

pub fn software_reset() -> ! {
    safe_on();
    wr8(R8_RST_WDOG_CTRL, rd8(R8_RST_WDOG_CTRL) | RB_SOFTWARE_RESET);
    safe_off();
    // The reset takes effect at once; this only satisfies the type.
    loop {
        nop();
    }
}

/// Restart into the ROM bootloader for USB ISP (wchisp).
///
/// The datasheet's description of RB_SOFTWARE_RESET gives the condition:
/// with RB_WDOG_INT_EN and RB_ROM_CODE_WE set, RB_ROM_DATA_WE clear and
/// R8_WDOG_COUNT in 128..192, the reset starts the bootloader and sets
/// RB_BOOT_LOAD_MAN.  That wording comes from the CH58x, where bit 7 was
/// RB_ROM_CODE_WE and bit 6 RB_ROM_DATA_WE; the CH572 merges both into a
/// 2-bit RB_ROM_CODE_WE, so the condition is read here as bits 7:6 = 10b.
/// No WCH example uses this path.  If the reading is wrong the chip just
/// restarts this firmware.
pub fn enter_bootloader() -> ! {
    safe_on();
    // The watchdog as a plain timer: its count is part of the condition
    // and it must not reset the chip in the meantime.
    wr8(R8_RST_WDOG_CTRL, RB_WDOG_INT_EN);
    wr8(R8_GLOB_ROM_CFG, (rd8(R8_GLOB_ROM_CFG) & 0x30) | 0x80);
    safe_off();
    // 0x80 leaves 64 counts (~84 ms) before the count leaves 128..192.
    wr8(R8_WDOG_COUNT, 0x80);
    safe_on();
    wr8(R8_RST_WDOG_CTRL, RB_WDOG_INT_EN | RB_SOFTWARE_RESET);
    safe_off();
    loop {
        nop();
    }
}

/// Pin directions and idle levels for the table in the module doc.
pub fn pins_init() {
    // Levels first, so outputs come up already idle: LEDs off (high),
    // TXD idle high, USB_VOUT_EN low.
    wr32(R32_PA_SET, LED_ACT | LED_USB | TXD);
    wr32(R32_PA_CLR, VOUT_EN);
    // PD_DRV means 5 mA drive for outputs and no pull-down for inputs.
    let all = LED_ACT | LED_USB | VOUT_EN | TXD | RXD | TP7 | TP9 | SWCLK | SWDIO | NRESET;
    wr32(R32_PA_PD_DRV, rd32(R32_PA_PD_DRV) & !all);
    wr32(
        R32_PA_PU,
        (rd32(R32_PA_PU) | RXD | TP7 | TP9) & !(SWCLK | SWDIO | NRESET),
    );
    let dir = rd32(R32_PA_DIR) & !(RXD | TP7 | TP9 | SWCLK | SWDIO | NRESET);
    wr32(R32_PA_DIR, dir | LED_ACT | LED_USB | VOUT_EN | TXD);
}

/// D10: on while the host has the device configured.
pub fn led_usb(on: bool) {
    wr32(if on { R32_PA_CLR } else { R32_PA_SET }, LED_USB);
}

/// D11: serial or debug traffic.
pub fn led_activity(on: bool) {
    wr32(if on { R32_PA_CLR } else { R32_PA_SET }, LED_ACT);
}

// ---- UART ----------------------------------------------------------------

static UART_RX: Mutex<RefCell<Ring<512>>> = Mutex::new(RefCell::new(Ring::new()));
static UART_TX: Mutex<RefCell<Ring<256>>> = Mutex::new(RefCell::new(Ring::new()));

/// Route TXD to PA8 and RXD to PA9.  The line setting comes from the
/// host through [`Uart::configure`].
pub fn uart_init() {
    let alt = rd16(R16_PIN_ALTERNATE_H) & !(RB_UART_TXD | RB_UART_RXD);
    wr16(
        R16_PIN_ALTERNATE_H,
        alt | UART_REMAP_PA8_PA9 << 3 | UART_REMAP_PA8_PA9,
    );
}

/// Move bytes between the UART's 8-byte FIFOs and the rings.  At 115200
/// baud the receive FIFO fills in 0.7 ms, so this runs from the main loop
/// and from the SWD transfer loop alike.
pub fn uart_service() {
    critical_section::with(|cs| {
        let mut rx = UART_RX.borrow_ref_mut(cs);
        while rd8(R8_UART_LSR) & RB_LSR_DATA_RDY != 0 {
            // A full ring drops the newest bytes.
            rx.push(rd8(R8_UART_RBR));
        }
        let mut tx = UART_TX.borrow_ref_mut(cs);
        while rd8(R8_UART_TFC) < 8 {
            match tx.pop() {
                Some(byte) => wr8(R8_UART_THR, byte),
                None => break,
            }
        }
    });
}

/// The UART towards the Wio-E5, as the bridge sees it.
pub struct Uart;

impl UartPort for Uart {
    /// The order of WCH's UART_DefInit: divisor, FIFO (trigger at 4
    /// bytes), line control, TXD enable, pre-divider 1.
    fn configure(&mut self, setting: UartSetting) {
        critical_section::with(|cs| {
            *UART_RX.borrow_ref_mut(cs) = Ring::new();
            *UART_TX.borrow_ref_mut(cs) = Ring::new();
        });
        wr16(R16_UART_DL, setting.dl);
        wr8(R8_UART_FCR, (2 << 6) | RB_FCR_FIFO_EN);
        wr8(R8_UART_LCR, setting.lcr);
        wr8(R8_UART_IER, RB_IER_TXD_EN);
        wr8(R8_UART_DIV, 1);
    }

    fn read(&mut self, buf: &mut [u8]) -> usize {
        uart_service();
        critical_section::with(|cs| UART_RX.borrow_ref_mut(cs).pop_slice(buf))
    }

    fn write(&mut self, data: &[u8]) -> usize {
        let n = critical_section::with(|cs| UART_TX.borrow_ref_mut(cs).push_slice(data));
        uart_service();
        n
    }

    fn tx_free(&self) -> usize {
        critical_section::with(|cs| UART_TX.borrow_ref(cs).free())
    }
}

// ---- USB -----------------------------------------------------------------

/// Endpoint DMA buffers: 128 bytes each (OUT then IN for endpoints 1 to
/// 3; endpoint 0 uses the first 64).  The controller wants them 4-byte
/// aligned.
#[repr(C, align(4))]
struct EpBuffers([[u8; 128]; 4]);

struct DmaBuffers(UnsafeCell<EpBuffers>);

// Only the USB driver touches these, from the one thread of execution.
unsafe impl Sync for DmaBuffers {}

static EP_BUFFERS: DmaBuffers = DmaBuffers(UnsafeCell::new(EpBuffers([[0; 128]; 4])));

/// The USB device controller's registers.
pub struct UsbHw;

impl UsbRegs for UsbHw {
    fn read(&self, offset: usize) -> u8 {
        rd8(R8_USB_CTRL + offset)
    }

    fn write(&self, offset: usize, value: u8) {
        wr8(R8_USB_CTRL + offset, value);
    }

    fn write16(&self, offset: usize, value: u16) {
        wr16(R8_USB_CTRL + offset, value);
    }

    fn buffer(&self, ep: usize) -> *mut u8 {
        unsafe { (*EP_BUFFERS.0.get()).0[ep].as_mut_ptr() }
    }

    fn dma_address(&self, ep: usize) -> u16 {
        self.buffer(ep) as usize as u16
    }

    /// USB owns PA0/PA1 only with the CH572's own debug interface off
    /// (RB_PIN_DEBUG_EN must be clear before RB_PIN_USB_EN works).
    fn attach(&self, on: bool) {
        let alt = rd16(R16_PIN_ALTERNATE);
        let alt = if on {
            (alt & !RB_PIN_DEBUG_EN) | RB_PIN_USB_EN | RB_UDP_PU_EN
        } else {
            alt & !(RB_PIN_USB_EN | RB_UDP_PU_EN)
        };
        wr16(R16_PIN_ALTERNATE, alt);
    }
}

/// Drop off the bus: pull-up off and port disabled, so the host sees an
/// unplug before the next attach or the bootloader's.
pub fn usb_detach() {
    UsbHw.attach(false);
    wr8(R8_UDEV_CTRL, RB_UD_PD_DIS);
    wr8(R8_USB_CTRL, 0);
}

// ---- SWD -----------------------------------------------------------------

/// The Wio-E5's debug port on PA5 (SWCLK), PA7 (SWDIO) and PA6 (nRESET).
pub struct SwdPins {
    /// Busy-loop iterations per half clock period.
    half_period_loops: u32,
}

impl SwdPins {
    pub const fn new() -> Self {
        Self {
            half_period_loops: half_period_loops(1_000_000),
        }
    }
}

impl Default for SwdPins {
    fn default() -> Self {
        Self::new()
    }
}

/// Loop count for a half period at `hz`, assuming 2 cycles per `nop`
/// iteration and no other overhead, so the real clock comes out at or
/// below the request, never above.
const fn half_period_loops(hz: u32) -> u32 {
    FSYS_HZ / hz / 4
}

/// The level of a pin: the input register is only valid for inputs, so an
/// output reports what it drives.
fn level(mask: u32) -> bool {
    let reg = if rd32(R32_PA_DIR) & mask != 0 {
        R32_PA_OUT
    } else {
        R32_PA_PIN
    };
    rd32(reg) & mask != 0
}

impl SwdIo for SwdPins {
    #[inline(always)]
    fn set_clk(&mut self, high: bool) {
        wr32(if high { R32_PA_SET } else { R32_PA_CLR }, SWCLK);
    }

    #[inline(always)]
    fn set_dio(&mut self, high: bool) {
        wr32(if high { R32_PA_SET } else { R32_PA_CLR }, SWDIO);
    }

    #[inline(always)]
    fn read_dio(&mut self) -> bool {
        rd32(R32_PA_PIN) & SWDIO != 0
    }

    #[inline(always)]
    fn dio_output(&mut self, output: bool) {
        let dir = rd32(R32_PA_DIR);
        wr32(R32_PA_DIR, if output { dir | SWDIO } else { dir & !SWDIO });
    }

    #[inline(always)]
    fn half_period(&mut self) {
        for _ in 0..self.half_period_loops {
            nop();
        }
    }

    fn set_frequency(&mut self, hz: u32) -> bool {
        if hz == 0 {
            return false;
        }
        self.half_period_loops = half_period_loops(hz.min(FSYS_HZ));
        true
    }

    fn connect(&mut self) {
        wr32(R32_PA_SET, SWCLK | SWDIO);
        wr32(R32_PA_DIR, rd32(R32_PA_DIR) | SWCLK | SWDIO);
    }

    fn release(&mut self) {
        wr32(R32_PA_DIR, rd32(R32_PA_DIR) & !(SWCLK | SWDIO | NRESET));
    }

    /// Open drain by direction: low as an output, released as an input
    /// for R29 to pull up.
    fn set_reset(&mut self, assert: bool) {
        if assert {
            wr32(R32_PA_CLR, NRESET);
            wr32(R32_PA_DIR, rd32(R32_PA_DIR) | NRESET);
        } else {
            wr32(R32_PA_DIR, rd32(R32_PA_DIR) & !NRESET);
        }
    }

    fn levels(&mut self) -> (bool, bool, bool) {
        (level(SWCLK), level(SWDIO), level(NRESET))
    }

    fn service(&mut self) {
        service();
    }

    fn delay_us(&mut self, us: u32) {
        let start = ticks();
        let span = us.saturating_mul(TICKS_PER_US);
        while ticks().wrapping_sub(start) < span {
            service();
        }
    }
}

/// dap-rs's delay (DAP_Delay), keeping the UART and watchdog serviced.
pub struct Delay;

impl embedded_hal::delay::DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        let start = ticks();
        let span = (ns as u64 * TICKS_PER_US as u64).div_ceil(1_000) as u32;
        while ticks().wrapping_sub(start) < span {
            service();
        }
    }
}
