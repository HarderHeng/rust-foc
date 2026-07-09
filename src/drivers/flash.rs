//! STM32G431 PAC flash driver — dual-slot.
//!
//! Bootloader at 0x0800_0000 (8 KB), slot A at 0x0800_2000 (58 KB),
//! slot B at 0x0801_0800 (58 KB), slot config at 0x0801_FFF8 (8 B).

use embassy_stm32::pac;
use foc_shared::{SLOT_A_BASE, SLOT_B_BASE, SLOT_SIZE, SLOT_CONFIG_ADDR};

const FLASH_BASE: u32 = 0x0800_0000;
const ERASE_SIZE: u32 = 2048;
const WRITE_SIZE: u32 = 8;

pub const METADATA_SIZE: u32 = 32;
pub const METADATA_MAGIC: u32 = 0xF0C1_001A;

#[derive(defmt::Format)]
pub enum FlashError { Unaligned, OutOfBounds, CrossPage, ProgramError }

pub fn current_slot() -> u8 {
    let vtor: u32 = unsafe { core::ptr::read_volatile(0xE000_ED08 as *const u32) };
    if vtor >= SLOT_B_BASE { 1 } else { 0 }
}

pub fn other_slot_range() -> (u32, u32) {
    if current_slot() == 0 { (SLOT_B_BASE, SLOT_B_BASE + SLOT_SIZE) }
    else                    { (SLOT_A_BASE, SLOT_A_BASE + SLOT_SIZE) }
}

pub fn other_metadata_addr() -> u32 {
    let (_, end) = other_slot_range();
    end - METADATA_SIZE
}

#[inline(never)]
fn page_of(offset: u32) -> u32 { (offset - FLASH_BASE) / ERASE_SIZE }

pub unsafe fn erase_region(start: u32, end: u32) -> Result<(), FlashError> {
    unlock_sequence();
    let flash = pac::FLASH;
    flash.cr().modify(|w| w.set_per(true));
    for page in page_of(start)..page_of(end) {
        flash.cr().modify(|w| w.set_pnb(page as u8));
        flash.cr().modify(|w| w.set_strt(true));
        wait_busy();
        check_and_clear_errors()?;
    }
    flash.cr().modify(|w| { w.set_per(false); w.set_strt(false); });
    lock();
    Ok(())
}

pub unsafe fn write_u64(offset: u32, region_start: u32, region_end: u32, value: u64) -> Result<(), FlashError> {
    if offset % WRITE_SIZE != 0 { return Err(FlashError::Unaligned); }
    if offset < region_start || offset + WRITE_SIZE > region_end { return Err(FlashError::OutOfBounds); }
    if page_of(offset) != page_of(offset + WRITE_SIZE - 1) { return Err(FlashError::CrossPage); }
    unlock_sequence();
    let flash = pac::FLASH;
    flash.cr().modify(|w| w.set_pg(true));
    core::ptr::write_volatile(offset as *mut u64, value);
    wait_busy();
    check_and_clear_errors()?;
    flash.cr().modify(|w| w.set_pg(false));
    lock();
    Ok(())
}

pub unsafe fn write_metadata(addr: u32, magic: u32, image_size: u32, image_crc: u32, version: &[u8; 16], build_timestamp: u32) -> Result<(), FlashError> {
    if page_of(addr) != page_of(addr + 31) { return Err(FlashError::CrossPage); }
    unlock_sequence();
    let flash = pac::FLASH;
    flash.cr().modify(|w| w.set_pg(true));
    let word0: u64 = (magic as u64) | ((image_size as u64) << 32);
    core::ptr::write_volatile(addr as *mut u64, word0); wait_busy();
    let r = check_and_clear_errors();
    let word1: u64 = (image_crc as u64) | ((build_timestamp as u64) << 32);
    if r.is_ok() { core::ptr::write_volatile((addr + 8) as *mut u64, word1); wait_busy(); let _ = check_and_clear_errors(); }
    let version_lo: u64 = u64::from_le_bytes(version[0..8].try_into().unwrap());
    if r.is_ok() { core::ptr::write_volatile((addr + 16) as *mut u64, version_lo); wait_busy(); let _ = check_and_clear_errors(); }
    let version_hi: u64 = u64::from_le_bytes(version[8..16].try_into().unwrap());
    if r.is_ok() { core::ptr::write_volatile((addr + 24) as *mut u64, version_hi); wait_busy(); let _ = check_and_clear_errors(); }
    flash.cr().modify(|w| w.set_pg(false)); lock(); r
}

pub unsafe fn write_slot_config(value: u64) -> Result<(), FlashError> {
    erase_region(SLOT_CONFIG_ADDR & !(ERASE_SIZE - 1), SLOT_CONFIG_ADDR + 8)?;
    write_u64(SLOT_CONFIG_ADDR, SLOT_CONFIG_ADDR, SLOT_CONFIG_ADDR + 8, value)
}

#[inline(never)] unsafe fn unlock_sequence() {
    let flash = pac::FLASH; while flash.sr().read().bsy() {}
    flash.keyr().write_value(0x4567_0123); flash.keyr().write_value(0xCDEF_89AB);
}
#[inline(never)] unsafe fn lock() { pac::FLASH.cr().modify(|w| w.set_lock(true)); }
#[inline(never)] unsafe fn wait_busy() { while pac::FLASH.sr().read().bsy() {} }
#[inline(never)] unsafe fn check_and_clear_errors() -> Result<(), FlashError> {
    let sr = pac::FLASH.sr().read();
    if sr.progerr() || sr.wrperr() || sr.pgaerr() || sr.sizerr() || sr.pgserr() {
        pac::FLASH.sr().write(|w| { w.set_progerr(true); w.set_wrperr(true); w.set_pgaerr(true); w.set_sizerr(true); w.set_pgserr(true); });
        return Err(FlashError::ProgramError);
    }
    Ok(())
}
pub unsafe fn read_u32(offset: u32) -> u32 { core::ptr::read_volatile(offset as *const u32) }
