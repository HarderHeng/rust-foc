//! STM32G431 flash driver, embassy-boot swap model.

use embassy_stm32::pac;

const FLASH_BASE: u32 = 0x0800_0000;
const ERASE_SIZE: u32 = 2048;

#[derive(defmt::Format)]
pub enum FlashError { Unaligned, OutOfBounds, CrossPage, ProgramError }

#[inline(never)] fn page_of(offset: u32) -> u32 { (offset - FLASH_BASE) / ERASE_SIZE }
pub unsafe fn read_u32(offset: u32) -> u32 { core::ptr::read_volatile(offset as *const u32) }
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
pub unsafe fn write_u64(offset: u32, start: u32, end: u32, value: u64) -> Result<(), FlashError> {
    if offset % 8 != 0 { return Err(FlashError::Unaligned); }
    if offset < start || offset + 8 > end { return Err(FlashError::OutOfBounds); }
    if page_of(offset) != page_of(offset + 7) { return Err(FlashError::CrossPage); }
    unlock_sequence(); let flash = pac::FLASH; flash.cr().modify(|w| w.set_pg(true));
    core::ptr::write_volatile(offset as *mut u64, value); wait_busy();
    check_and_clear_errors()?; flash.cr().modify(|w| w.set_pg(false)); lock(); Ok(())
}
pub unsafe fn erase_region(start: u32, end: u32) -> Result<(), FlashError> {
    unlock_sequence(); let flash = pac::FLASH; flash.cr().modify(|w| w.set_per(true));
    for page in page_of(start)..page_of(end) {
        flash.cr().modify(|w| w.set_pnb(page as u8)); flash.cr().modify(|w| w.set_strt(true));
        wait_busy(); check_and_clear_errors()?;
    }
    flash.cr().modify(|w| { w.set_per(false); w.set_strt(false); }); lock(); Ok(())
}
