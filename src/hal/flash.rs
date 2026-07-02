//! STM32G431 PAC-level flash driver used by the OTA path.
//!
//! Minimal in-app OTA: app occupies 0x0800_0000 through ~0x0801_F7FF.
//! We drive the FLASH peripheral registers directly because no HAL
//! is appropriate for a write path that runs from the same flash
//! being written to (HALs tend to assume the chip is in a "blank"
//! state and crash on the second erase).
//!
//! ## Safety model (Phase 4 v1)
//!
//! - The app region is written **while the UDS handler is
//!   running from the same region**. Standard STM32 flash write
//!   semantics mean: any write to a flash address that contains
//!   the current PC or stack-relative data will brick the chip.
//! - v1 doesn't relocate the OTA state machine to RAM; the user
//!   accepts the risk that a power loss during write may brick.
//! - v1 uses a 4-byte-per-block write granularity (each UDS
//!   TransferData call writes 2 bytes; the SDO write response
//!   happens after both bytes land, so the UDS handler's PC is
//!   never inside the 2-byte window being written — but the
//!   surrounding code is).
//!
//! Page size is 2 KB; write granularity is 8 bytes (one u64).
//! Block at end-of-write must be padded to 8 bytes.

use embassy_stm32::pac;

/// 1 KB per page, 2 KB per "large" page. STM32G431 page size
/// depends on flash size; 128 KB flash ⇒ 2 KB page.
const ERASE_SIZE: u32 = 2048;

/// 8-byte (u64) write granularity. STM32G4 flash programs in
/// double-words; mismatched sizes would need cache-aligned
/// software handling we don't want to write.
const WRITE_SIZE: u32 = 8;

/// Start of the user app region. Phase 4 v1: the app occupies
/// the entire user flash (no bootloader stub).
pub const APP_START: u32 = 0x0800_0000;

/// End of the user app region (exclusive). The 2 KB metadata
/// block at 0x0801_F800 is reserved and not part of the OTA
/// image.
pub const APP_END: u32 = 0x0801_F800;

#[derive(Debug)]
pub enum FlashError {
    Unaligned,
    OutOfBounds,
    ProgramError,
}

/// Erase the entire app region. Call once at the start of
/// an OTA download. The metadata block (last 2 KB) is NOT
/// erased — that's where we record the post-OTA image size
/// + CRC32.
pub unsafe fn erase_app_region() -> Result<(), FlashError> {
    unlock_sequence();
    let flash = pac::FLASH;
    flash.cr().modify(|w| w.set_per(true));
    let start_page = APP_START / ERASE_SIZE;
    let end_page = APP_END / ERASE_SIZE;
    for page in start_page..end_page {
        flash.cr().modify(|w| w.set_pnb(page as u8));
        flash.cr().modify(|w| w.set_strt(true));
        wait_busy();
        check_and_clear_errors()?;
    }
    flash.cr().modify(|w| {
        w.set_per(false);
        w.set_strt(false);
    });
    lock();
    Ok(())
}

/// Write 8 bytes (one u64) to flash at `offset`. The caller
/// must ensure `offset` is 8-byte aligned, `offset..offset+8`
/// is within `APP_START..APP_END`, and the destination page
/// has been erased.
///
/// The value is written little-endian (low 4 bytes first, then
/// high 4 bytes), matching STM32 flash memory layout.
pub unsafe fn write_u64(offset: u32, value: u64) -> Result<(), FlashError> {
    if offset % WRITE_SIZE != 0 {
        return Err(FlashError::Unaligned);
    }
    if offset < APP_START || offset + WRITE_SIZE > APP_END {
        return Err(FlashError::OutOfBounds);
    }
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

/// Read 4 bytes from flash. Used by the OTA CRC32 verification
/// pass.
pub fn read_u32(offset: u32) -> u32 {
    unsafe { core::ptr::read_volatile(offset as *const u32) }
}

unsafe fn unlock_sequence() {
    let flash = pac::FLASH;
    while flash.sr().read().bsy() {}
    flash.keyr().write_value(0x4567_0123);
    flash.keyr().write_value(0xCDEF_89AB);
}

unsafe fn lock() {
    pac::FLASH.cr().modify(|w| w.set_lock(true));
}

unsafe fn wait_busy() {
    while pac::FLASH.sr().read().bsy() {}
}

unsafe fn check_and_clear_errors() -> Result<(), FlashError> {
    let sr = pac::FLASH.sr().read();
    if sr.progerr() || sr.wrperr() || sr.pgaerr() || sr.sizerr() || sr.pgserr() {
        // Clear the error flags by writing 1 to them (r/c1 on
        // STM32G4).
        pac::FLASH.sr().modify(|w| {
            w.set_progerr(true);
            w.set_wrperr(true);
            w.set_pgaerr(true);
            w.set_sizerr(true);
            w.set_pgserr(true);
        });
        Err(FlashError::ProgramError)
    } else {
        Ok(())
    }
}
