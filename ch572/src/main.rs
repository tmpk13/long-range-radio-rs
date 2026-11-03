//! CH572D firmware entry: startup, then the bridge's polling loop.  See
//! the library docs for what the bridge does and notes/ch572.md for how
//! to flash it.

#![no_std]
#![no_main]

use ch572_bridge::bridge::Bridge;
use ch572_bridge::hw;
use ch572_bridge::usb::Ch572Usb;
use usb_device::bus::UsbBusAllocator;

// Reset entry at address 0, where the core starts.  Like WCH's
// startup_CH572.S: global and stack pointers, RAM functions and .data
// copied from flash, .bss cleared, instruction prefetch on (corecfgr
// 0xBC0 = 0x25).  Unlike it, this stays in machine mode with interrupts
// off: the firmware polls.  Every trap resets the chip.
core::arch::global_asm!(
    r#"
    .section .init, "ax"
    .global _start
_start:
    .option push
    .option norvc
    j _reset
    .option pop

    .section .text._reset, "ax"
    .align 2
_reset:
    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop
    la sp, _stack_top

    la a0, _highcode_lma
    la a1, _highcode_start
    la a2, _highcode_end
1:  bgeu a1, a2, 2f
    lw t0, 0(a0)
    sw t0, 0(a1)
    addi a0, a0, 4
    addi a1, a1, 4
    j 1b
2:
    la a0, _data_lma
    la a1, _data_start
    la a2, _data_end
3:  bgeu a1, a2, 4f
    lw t0, 0(a0)
    sw t0, 0(a1)
    addi a0, a0, 4
    addi a1, a1, 4
    j 3b
4:
    la a0, _bss_start
    la a1, _bss_end
5:  bgeu a0, a1, 6f
    sw zero, 0(a0)
    addi a0, a0, 4
    j 5b
6:
    li t0, 0x25
    csrw 0xbc0, t0
    la t0, _trap
    csrw mtvec, t0
    call firmware_main
7:  j 7b

    .section .text._trap, "ax"
    .align 2
_trap:
    j trap_handler
"#
);

#[unsafe(no_mangle)]
extern "C" fn trap_handler() -> ! {
    hw::software_reset()
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    hw::software_reset()
}

/// Critical sections mask machine interrupts, which this firmware never
/// enables in the first place.
struct SingleHart;
critical_section::set_impl!(SingleHart);

unsafe impl critical_section::Impl for SingleHart {
    unsafe fn acquire() -> critical_section::RawRestoreState {
        let mstatus: usize;
        unsafe { core::arch::asm!("csrrci {}, mstatus, 8", out(reg) mstatus) };
        mstatus & 8 != 0
    }

    unsafe fn release(was_enabled: critical_section::RawRestoreState) {
        if was_enabled {
            unsafe { core::arch::asm!("csrsi mstatus, 8") };
        }
    }
}

/// USB serial number; set BRIDGE_SERIAL when building to tell boards
/// apart (probe-rs selects probes by it).
const SERIAL: &str = match option_env!("BRIDGE_SERIAL") {
    Some(serial) => serial,
    None => "FSP-1",
};

const MS: u32 = 1_000 * hw::TICKS_PER_US;
/// How long D11 stays lit after traffic.
const ACTIVITY_PULSE: u32 = 20 * MS;
/// D10 blinks at this half-period until the host configures the device.
const UNCONFIGURED_BLINK: u32 = 500 * MS;

#[unsafe(no_mangle)]
extern "C" fn firmware_main() -> ! {
    hw::init_clocks();
    hw::systick_start();
    hw::pins_init();
    hw::uart_init();
    hw::watchdog_start();
    // A pull-up left on by the bootloader or an earlier run would hide
    // the restart from the host; detach long enough for it to notice.
    hw::usb_detach();
    hw::delay_ms(20);

    let bus = UsbBusAllocator::new(Ch572Usb::new(hw::UsbHw));
    let mut bridge = Bridge::new(&bus, hw::SwdPins::new(), hw::Delay, SERIAL);
    let mut uart = hw::Uart;
    let mut last_activity = hw::ticks().wrapping_sub(ACTIVITY_PULSE);

    loop {
        hw::service();
        let events = bridge.poll(&mut uart);
        if events.enter_bootloader {
            hw::usb_detach();
            hw::delay_ms(50);
            hw::enter_bootloader();
        }
        let now = hw::ticks();
        if events.activity {
            last_activity = now;
        }
        hw::led_activity(now.wrapping_sub(last_activity) < ACTIVITY_PULSE);
        hw::led_usb(bridge.configured() || (now / UNCONFIGURED_BLINK).is_multiple_of(2));
    }
}
