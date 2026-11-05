//! The app's side of the boot-state log (`flash_layout::boot`): it reads
//! the log once at boot, appends `Confirmed` when a trial image has proven
//! itself, and asks for a swap when the OTA receiver has staged a verified
//! image.  The bootloader does the rest.
//!
//! Writes go through the HAL's flash driver; reads through
//! [`crate::flash_ecc`], so a double-word a power cut left unreadable is
//! skipped instead of hanging the app in the NMI handler.

use flash_layout::boot::{Fault, Flash as LogFlash, GEOMETRY, Image, Log, Outcome, State};
use flash_layout::{FLASH_BASE, PAGE_SIZE};
use stm32wlxx_hal::flash::{AlignedAddr, Flash, Page};
use stm32wlxx_hal::pac;

use crate::flash_ecc::{self, Unreadable};

/// The log as this app sees it.
pub struct BootState {
    log: Log,
    outcome: Option<Outcome>,
}

impl BootState {
    /// Read the log.  `outcome` is what the bootloader says it did this boot
    /// (`reset_record::Record::handoff`); `None` means it did not start this
    /// app.  An app started some other way (probe-rs sets the program
    /// counter itself) is a freshly flashed build that nothing pending
    /// applies to, so the log is settled at once: otherwise the next boot
    /// would revert over it, or carry on a swap a power cut interrupted, or
    /// swap in an image staged just before the flash.
    pub fn load(flash: &mut pac::FLASH, outcome: Option<Outcome>) -> Self {
        let mut f = AppFlash { periph: flash };
        let mut log = Log::scan(&mut f, &GEOMETRY);
        if outcome.is_none() {
            let _ = log.settle(&mut f, &GEOMETRY);
        }
        Self { log, outcome }
    }

    /// The log's current state.
    pub fn state(&self) -> State {
        self.log.state()
    }

    /// What the bootloader did this boot, or `None` when it did not start
    /// this app.
    pub fn outcome(&self) -> Option<Outcome> {
        self.outcome
    }

    /// This image runs on trial: the next reset before [`confirm`]
    /// (Self::confirm) reverts it.
    pub fn on_trial(&self) -> bool {
        self.log.state() == State::Trial
    }

    /// Nothing pending: the node may take a new image.
    pub fn settled(&self) -> bool {
        self.log.state() == State::Idle
    }

    /// Keep this image: append `Confirmed` if it is on trial.
    pub fn confirm(&mut self, flash: &mut pac::FLASH) -> Result<(), Fault> {
        self.log.confirm(&mut AppFlash { periph: flash }, &GEOMETRY)
    }

    /// Ask the bootloader to verify and swap in the image staged in the DFU
    /// slot, at the next reset.  Fails unless [`settled`](Self::settled).
    pub fn request_swap(
        &mut self,
        flash: &mut pac::FLASH,
        size: u32,
        crc: u32,
    ) -> Result<(), Fault> {
        self.log.request_swap(
            &mut AppFlash { periph: flash },
            &GEOMETRY,
            Image { size, crc },
        )
    }
}

/// The flash for the log.  The app never swaps, so it has no page copy.
struct AppFlash<'a> {
    periph: &'a mut pac::FLASH,
}

fn dw_addr(page: u32, index: usize) -> u32 {
    FLASH_BASE + page * PAGE_SIZE + index as u32 * 8
}

impl LogFlash for AppFlash<'_> {
    fn read(&mut self, page: u32, index: usize) -> Result<u64, Unreadable> {
        flash_ecc::read_dw(dw_addr(page, index))
    }

    fn program(&mut self, page: u32, index: usize, value: u64) -> Result<(), Fault> {
        let mut flash = Flash::unlock(self.periph);
        // SAFETY: dw_addr is 8-byte aligned and inside the log page.
        let addr = unsafe { AlignedAddr::new_unchecked(dw_addr(page, index) as usize) };
        // SAFETY: the log page holds no code or data the app runs from.
        unsafe { flash.program_bytes(&value.to_le_bytes(), addr) }.map_err(|_| Fault::Failed)
    }

    fn erase(&mut self, page: u32) -> Result<(), Fault> {
        let page = u8::try_from(page)
            .ok()
            .and_then(Page::from_index)
            .ok_or(Fault::Failed)?;
        let mut flash = Flash::unlock(self.periph);
        // SAFETY: as for `program`.
        unsafe { flash.page_erase(page) }.map_err(|_| Fault::Failed)
    }

    fn copy(&mut self, _src: u32, _dst: u32) -> Result<(), Fault> {
        Err(Fault::Failed)
    }
}
