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

/// Flash base address on STM32G4. Page numbers in the FLASH_CR
/// register are offsets from this address, NOT the absolute
/// address divided by page size — a previous version of this
/// file computed `APP_START / ERASE_SIZE = 0x1000` and then
/// truncated `as u8` to 0, which silently erased only the first
/// three pages (6 KB) of the 124 KB app region.
const FLASH_BASE: u32 = 0x0800_0000;

/// 1 KB per page, 2 KB per "large" page. STM32G431 page size
/// depends on flash size; 128 KB flash ⇒ 2 KB page.
const ERASE_SIZE: u32 = 2048;

/// 8-byte (u64) write granularity. STM32G4 flash programs in
/// double-words; mismatched sizes would need cache-aligned
/// software handling we don't want to write.
const WRITE_SIZE: u32 = 8;

/// Start of the user app region. Phase 4 v1: the app occupies
/// the entire user flash (no bootloader stub).
pub const APP_START: u32 = FLASH_BASE;

/// End of the user app region (exclusive). The 2 KB metadata
/// block at 0x0801_F800 is reserved and not part of the OTA
/// image.
pub const APP_END: u32 = 0x0801_F800;

/// Address of the post-OTA metadata block. Must match the
/// reader side (typically `src/metadata.rs`).
pub const METADATA_ADDR: u32 = 0x0801_F800;

#[derive(Debug)]
pub enum FlashError {
    Unaligned,
    OutOfBounds,
    /// The 8-byte write would span a 2 KB page boundary. STM32G4
    /// raises SIZERR if a double-word write isn't contained in
    /// one page. The caller must either re-align the data or
    /// split it across two writes at the page boundary.
    CrossPage,
    ProgramError,
}

/// Convert an absolute flash address to a page number suitable
/// for `FLASH_CR.PNB`. STM32G4 page numbers are offsets from
/// `FLASH_BASE`, not (address / page_size).
///
/// Lives in `.data` (RAM) because it's on the OTA write hot path:
/// `write_u64` calls it before any flash write, so jumping back
/// into flash is fine at that point. But putting it in RAM keeps
/// the call graph uniform — every function called from the OTA
/// path is in RAM, so we never have to reason about whether a
/// given helper's address is "still ahead of the write pointer".
#[inline(never)]
#[link_section = ".data"]
fn page_of(offset: u32) -> u32 {
    (offset - FLASH_BASE) / ERASE_SIZE
}

/// Erase the entire app region. Call once at the start of
/// an OTA download. The metadata block (last 2 KB) is NOT
/// erased — that's where we record the post-OTA image size
/// + CRC32.
///
/// Lives in `.data` (RAM) — the erase loop runs from flash,
/// and the call into a flash-resident helper after the erase
/// pointer has crossed that helper's address would crash. See
/// the module-level docs in `src/can/ota.rs` for the rationale.
#[inline(never)]
#[link_section = ".data"]
pub unsafe fn erase_app_region() -> Result<(), FlashError> {
    unlock_sequence();
    let flash = pac::FLASH;
    flash.cr().modify(|w| w.set_per(true));
    let start_page = page_of(APP_START);
    let end_page = page_of(APP_END);
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
///
/// Lives in `.data` (RAM) — this is the hot path during OTA and
/// must not be running from flash while we're writing flash.
/// See the module-level docs in `src/can/ota.rs` for the rationale.
#[inline(never)]
#[link_section = ".data"]
pub unsafe fn write_u64(offset: u32, value: u64) -> Result<(), FlashError> {
    if offset % WRITE_SIZE != 0 {
        return Err(FlashError::Unaligned);
    }
    if offset < APP_START || offset + WRITE_SIZE > APP_END {
        return Err(FlashError::OutOfBounds);
    }
    // STM32G4 cannot program a double-word across a page
    // boundary — the controller raises SIZERR. Reject up front
    // so the caller gets a clean error rather than a generic
    // ProgramError.
    if page_of(offset) != page_of(offset + WRITE_SIZE - 1) {
        return Err(FlashError::CrossPage);
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

/// Write the post-OTA metadata block. Layout (must match the
/// reader side, typically `src/metadata.rs`):
///
///   0x00: magic      (u32, LE)
///   0x04: image_size (u32, LE)
///   0x08: image_crc  (u32, LE)
///   0x0C: padding    (u32, 0xFFFF_FFFF — anything works)
///
/// The block is written as two u64s. Both writes must land in
/// the same page (the metadata block is one page); we check
/// that explicitly.
///
/// The metadata block is NOT erased by `erase_app_region` —
/// the OTA path needs the previous image's metadata to survive
/// until `write_metadata` overwrites it (the chip may reboot
/// between writes if power is lost, and the bootloader would
/// fall back to the previous image).
///
/// Lives in `.data` (RAM) — same reason as `write_u64`.
#[inline(never)]
#[link_section = ".data"]
pub unsafe fn write_metadata(
    magic: u32,
    image_size: u32,
    image_crc: u32,
) -> Result<(), FlashError> {
    // Single-page check — both u64 writes need to be in the
    // 0x0801_F800 page.
    if page_of(METADATA_ADDR) != page_of(METADATA_ADDR + WRITE_SIZE - 1) {
        // Should be unreachable given our constants, but defend.
        return Err(FlashError::CrossPage);
    }
    // The metadata block lives outside APP_END (it's the 2 KB
    // reserved area). write_u64 rejects offsets in that range,
    // so we write it directly here with the same unlock / PG /
    // wait_busy sequence.
    unlock_sequence();
    let flash = pac::FLASH;
    flash.cr().modify(|w| w.set_pg(true));

    // First u64: magic + image_size (LE).
    let word0: u64 = (magic as u64) | ((image_size as u64) << 32);
    core::ptr::write_volatile(METADATA_ADDR as *mut u64, word0);
    wait_busy();
    let r = check_and_clear_errors();

    // Second u64: image_crc + padding.
    if r.is_ok() {
        let word1: u64 = (image_crc as u64) | (0xFFFF_FFFF_u64 << 32);
        core::ptr::write_volatile((METADATA_ADDR + 8) as *mut u64, word1);
        wait_busy();
        let _ = check_and_clear_errors();
    }

    flash.cr().modify(|w| w.set_pg(false));
    lock();
    r
}

/// Unlock the flash controller. Key sequence is two writes
/// (KEY1 then KEY2) to FLASH_KEYR, per RM0440 §3.3.5.
///
/// Lives in `.data` (RAM) — see `write_u64`'s docs for the
/// rationale (same as the rest of the OTA path).
#[inline(never)]
#[link_section = ".data"]
unsafe fn unlock_sequence() {
    let flash = pac::FLASH;
    while flash.sr().read().bsy() {}
    flash.keyr().write_value(0x4567_0123);
    flash.keyr().write_value(0xCDEF_89AB);
}

/// Lock the flash controller. Always called from RAM-resident
/// OTA code, so lives in `.data`.
#[inline(never)]
#[link_section = ".data"]
unsafe fn lock() {
    pac::FLASH.cr().modify(|w| w.set_lock(true));
}

/// Wait until the flash controller reports !bsy. Always called
/// from RAM-resident OTA code, so lives in `.data`.
#[inline(never)]
#[link_section = ".data"]
unsafe fn wait_busy() {
    while pac::FLASH.sr().read().bsy() {}
}

#[inline(never)]
#[link_section = ".data"]
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
