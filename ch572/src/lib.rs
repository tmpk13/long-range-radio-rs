//! CH572D firmware for the Forest Sensor Platform: a USB bridge to the
//! Wio-E5.  The board wires the CH572D's USB to the USB-C connector, its
//! UART to the Wio-E5's USART2 and three GPIOs to the Wio-E5's SWD port
//! and reset, so one USB cable gives the host a serial port and a
//! CMSIS-DAP debug probe.
//!
//! Everything except `hw` is free of register access and is tested on the
//! host against simulated hardware (`cargo test`, see the mise tasks).

#![cfg_attr(not(test), no_std)]

pub mod bridge;
pub mod dapclass;
#[cfg(target_arch = "riscv32")]
pub mod hw;
pub mod linecode;
pub mod regs;
pub mod ring;
pub mod swd;
pub mod touch;
pub mod usb;

#[cfg(test)]
mod sim;

/// System clock: the 600 MHz PLL divided by 6, WCH's
/// CLK_SOURCE_HSE_PLL_100MHz.  Every UART rate up to 460800 comes within
/// 0.5% of nominal at this clock (see `linecode`).
pub const FSYS_HZ: u32 = 100_000_000;
