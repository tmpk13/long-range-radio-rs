//! Flash partition map of the Wio-E5's STM32WLE5JC, the boot-state log and
//! slot swap ([`boot`]), and the image CRC ([`crc32`]), shared by the
//! bootloader, the node firmware and the basestation.  Each of those crates
//! also generates its linker `memory.x` from these constants in its build
//! script, through [`write_memory_x`].
//!
//! The 256 KB flash is 128 pages of 2 KB:
//!
//! | Partition    | Pages   | Size   | Start         | Written by                          |
//! |--------------|---------|--------|---------------|-------------------------------------|
//! | Bootloader   | 0-7     | 16 KB  | `0x0800_0000` | probe-rs, once per board            |
//! | Active app   | 8-63    | 112 KB | `0x0800_4000` | probe-rs, or the bootloader's swap  |
//! | DFU staging  | 64-119  | 112 KB | `0x0802_0000` | the OTA receiver, the bootloader    |
//! | Swap scratch | 120     | 2 KB   | `0x0803_C000` | the bootloader                      |
//! | Boot state   | 121     | 2 KB   | `0x0803_C800` | the app and the bootloader          |
//! | Swap scratch | 122-127 | 12 KB  | `0x0803_D000` | the bootloader                      |
//!
//! The swap uses its seven scratch pages in turn ([`SCRATCH_PAGES`]), so no
//! one of them wears out first.
//!
//! These numbers are a contract with every bootloader already flashed: an
//! over-the-air update replaces only the app, so an app built with a
//! different map would stage images where the deployed bootloader does not
//! look for them.  The boot-state log's format is part of the same contract
//! (see [`boot`]).

#![no_std]

use core::fmt;

pub mod boot;
pub mod crc32;

/// First byte of internal flash.
pub const FLASH_BASE: u32 = 0x0800_0000;
/// Internal flash size in bytes (STM32WLE5JC: 256 KB).
pub const FLASH_SIZE: u32 = 256 * 1024;
/// Flash erase unit in bytes.
pub const PAGE_SIZE: u32 = 2048;

/// First byte of SRAM.
pub const RAM_BASE: u32 = 0x2000_0000;
/// SRAM size in bytes, all of it given to whichever program runs.
pub const RAM_SIZE: u32 = 64 * 1024;

/// First page of the bootloader.
pub const BOOTLOADER_PAGE: u32 = 0;
/// Pages reserved for the bootloader.
pub const BOOTLOADER_PAGES: u32 = 8;
/// First page of the active app slot, where the running app is linked.
pub const ACTIVE_PAGE: u32 = 8;
/// Pages in the active slot; the DFU slot holds the same number.
pub const ACTIVE_PAGES: u32 = 56;
/// First page of the DFU slot, where an incoming image is staged.
pub const DFU_PAGE: u32 = 64;
/// Page the bootloader parks one active page in while it swaps the slots;
/// the first of [`SCRATCH_PAGES`].
pub const SCRATCH_PAGE: u32 = 120;
/// Page holding the boot-state log (see [`boot`]).
pub const STATE_PAGE: u32 = 121;
/// Every scratch page, used in turn by slot page (`boot::Geometry::scratch_for`).
pub const SCRATCH_PAGES: [u32; 7] = [SCRATCH_PAGE, 122, 123, 124, 125, 126, 127];

/// Address of a flash page.
pub const fn page_addr(page: u32) -> u32 {
    FLASH_BASE + page * PAGE_SIZE
}

/// Start of the bootloader.
pub const BOOTLOADER_BASE: u32 = page_addr(BOOTLOADER_PAGE);
/// Bootloader size limit in bytes.
pub const BOOTLOADER_SIZE: u32 = BOOTLOADER_PAGES * PAGE_SIZE;
/// Start of the active app slot; the app's vector table sits here.
pub const ACTIVE_BASE: u32 = page_addr(ACTIVE_PAGE);
/// Largest app image in bytes: the size of either slot.
pub const MAX_FW_SIZE: u32 = ACTIVE_PAGES * PAGE_SIZE;
/// Start of the DFU slot.
pub const DFU_BASE: u32 = page_addr(DFU_PAGE);
/// Start of the swap scratch page.
pub const SCRATCH_BASE: u32 = page_addr(SCRATCH_PAGE);
/// Address of the boot-state log.
pub const STATE_ADDR: u32 = page_addr(STATE_PAGE);

// The partitions must keep their order without overlapping, the DFU slot must
// hold a whole active image, and everything must fit in the flash.
const _: () = {
    assert!(BOOTLOADER_PAGE + BOOTLOADER_PAGES <= ACTIVE_PAGE);
    assert!(ACTIVE_PAGE + ACTIVE_PAGES <= DFU_PAGE);
    assert!(DFU_PAGE + ACTIVE_PAGES <= SCRATCH_PAGE);
    assert!(SCRATCH_PAGE < STATE_PAGE);
    assert!(page_addr(STATE_PAGE + 1) <= FLASH_BASE + FLASH_SIZE);
    // The HAL addresses pages by a u8 index.
    assert!(STATE_PAGE <= u8::MAX as u32);
    // Every scratch page lies past the DFU slot, inside the flash, and off
    // the state page.
    let mut i = 0;
    while i < SCRATCH_PAGES.len() {
        let p = SCRATCH_PAGES[i];
        assert!(p >= DFU_PAGE + ACTIVE_PAGES && p != STATE_PAGE);
        assert!(page_addr(p + 1) <= FLASH_BASE + FLASH_SIZE);
        i += 1;
    }
};

/// Write a cortex-m-rt `memory.x` that links a program into `length` bytes of
/// flash at `origin`, with all of SRAM.  `length` must be a whole number of
/// KB.  Build scripts call this so that no linker script restates the map.
pub fn write_memory_x(out: &mut impl fmt::Write, origin: u32, length: u32) -> fmt::Result {
    assert!(
        length % 1024 == 0,
        "flash region is not a whole number of KB"
    );
    writeln!(
        out,
        "/* Generated by build.rs from the flash-layout crate. */"
    )?;
    writeln!(out, "MEMORY")?;
    writeln!(out, "{{")?;
    writeln!(
        out,
        "    FLASH : ORIGIN = {origin:#010x}, LENGTH = {}K",
        length / 1024
    )?;
    writeln!(
        out,
        "    RAM   : ORIGIN = {RAM_BASE:#010x}, LENGTH = {}K",
        RAM_SIZE / 1024
    )?;
    writeln!(out, "}}")
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::string::String;

    // Deployed bootloaders were built with these addresses; changing any of
    // them breaks over-the-air updates to existing boards.
    #[test]
    fn addresses_match_deployed_bootloaders() {
        assert_eq!(BOOTLOADER_BASE, 0x0800_0000);
        assert_eq!(ACTIVE_BASE, 0x0800_4000);
        assert_eq!(DFU_BASE, 0x0802_0000);
        assert_eq!(SCRATCH_BASE, 0x0803_C000);
        assert_eq!(STATE_ADDR, 0x0803_C800);
        assert_eq!(MAX_FW_SIZE, 112 * 1024);
    }

    #[test]
    fn app_memory_x() {
        let mut s = String::new();
        write_memory_x(&mut s, ACTIVE_BASE, MAX_FW_SIZE).unwrap();
        assert!(s.contains("FLASH : ORIGIN = 0x08004000, LENGTH = 112K\n"));
        assert!(s.contains("RAM   : ORIGIN = 0x20000000, LENGTH = 64K\n"));
    }

    #[test]
    fn bootloader_memory_x() {
        let mut s = String::new();
        write_memory_x(&mut s, BOOTLOADER_BASE, BOOTLOADER_SIZE).unwrap();
        assert!(s.contains("FLASH : ORIGIN = 0x08000000, LENGTH = 16K\n"));
    }

    #[test]
    #[should_panic(expected = "whole number of KB")]
    fn memory_x_rejects_partial_kb() {
        write_memory_x(&mut String::new(), ACTIVE_BASE, 1000).unwrap();
    }
}
