//! STM32G4 flash driver for the app side.
//!
//! Wraps `embassy_stm32::pac` for page erase and 64-bit programming.
//! This is the same low-level approach as the bootloader's `flash.rs`
//! but deployed in the app crate. Uses absolute addresses
//! (compatible with `foc_common::FlashOtaFlag`).

use core::ptr::{read_volatile, write_volatile};
use embedded_storage::nor_flash::{ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash};
use embassy_stm32::pac;

/// Flash error type for the STM32G4 driver.
#[derive(Debug, defmt::Format)]
pub enum FlashError {
    /// Address or length not aligned to WRITE_SIZE / ERASE_SIZE / READ_SIZE.
    Unaligned,
    /// Operation on an invalid address range.
    OutOfBounds,
}

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::Unaligned => NorFlashErrorKind::NotAligned,
            _ => NorFlashErrorKind::Other,
        }
    }
}

/// STM32G4 flash driver.
///
/// All operations take `&mut self` to satisfy the `NorFlash` trait's exclusive
/// access requirement. Uses raw PAC access, so absolute addresses are used.
pub struct Stm32g4Flash {
    _phantom: core::marker::PhantomData<()>,
}

impl Stm32g4Flash {
    pub fn new() -> Self {
        Self { _phantom: core::marker::PhantomData }
    }

    #[inline]
    fn pac() -> pac::flash::Flash { pac::FLASH }

    unsafe fn unlock() {
        let flash = Self::pac();
        while flash.sr().read().bsy() {}
        flash.keyr().write_value(0x4567_0123);
        flash.keyr().write_value(0xCDEF_89AB);
    }

    /// Lock the flash control register.
    unsafe fn lock() {
        Self::pac().cr().modify(|w| w.set_lock(true));
    }

    /// Wait for the BSY bit to clear.
    unsafe fn wait_busy() {
        while Self::pac().sr().read().bsy() {}
    }

    /// Check SR for error flags and clear them.
    unsafe fn check_and_clear_errors() -> Result<(), FlashError> {
        let sr = Self::pac().sr().read();
        if sr.progerr() || sr.wrperr() || sr.pgaerr() || sr.sizerr() || sr.pgserr() {
            // Clear error flags (write-1-to-clear).
            Self::pac().sr().modify(|w| {
                w.set_progerr(true);
                w.set_wrperr(true);
                w.set_pgaerr(true);
                w.set_sizerr(true);
                w.set_pgserr(true);
            });
            Err(FlashError::OutOfBounds)
        } else {
            Ok(())
        }
    }
}

impl ErrorType for Stm32g4Flash {
    type Error = FlashError;
}

impl NorFlash for Stm32g4Flash {
    const WRITE_SIZE: usize = 8;   // 64-bit half-words
    const ERASE_SIZE: usize = 2048; // 2 KB pages

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from % Self::ERASE_SIZE as u32 != 0 || to % Self::ERASE_SIZE as u32 != 0 {
            return Err(FlashError::Unaligned);
        }

        unsafe {
            Self::unlock();

            let flash = Self::pac();

            // Enable page erase mode.
            flash.cr().modify(|w| w.set_per(true));

            for page in (from / Self::ERASE_SIZE as u32)..(to / Self::ERASE_SIZE as u32) {
                flash.cr().modify(|w| w.set_pnb(page as u8));
                flash.cr().modify(|w| w.set_strt(true));
                Self::wait_busy();
                Self::check_and_clear_errors()?;
            }

            flash.cr().modify(|w| {
                w.set_per(false);
                w.set_strt(false);
            });

            Self::lock();
        }

        Ok(())
    }

    fn write(&mut self, mut offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if offset % Self::WRITE_SIZE as u32 != 0 || bytes.len() % Self::WRITE_SIZE != 0 {
            return Err(FlashError::Unaligned);
        }

        unsafe {
            Self::unlock();

            let flash = Self::pac();

            // Enable programming (64-bit).
            flash.cr().modify(|w| w.set_pg(true));

            for chunk in bytes.chunks_exact(Self::WRITE_SIZE) {
                let word: u64 = u64::from_le_bytes(chunk.try_into().unwrap());
                write_volatile(offset as *mut u64, word);
                offset += Self::WRITE_SIZE as u32;
                Self::wait_busy();
                Self::check_and_clear_errors()?;
            }

            flash.cr().modify(|w| w.set_pg(false));

            Self::lock();
        }

        Ok(())
    }
}

impl ReadNorFlash for Stm32g4Flash {
    const READ_SIZE: usize = 4;

    fn read(&mut self, mut offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        if offset % Self::READ_SIZE as u32 != 0 || bytes.len() % Self::READ_SIZE != 0 {
            return Err(FlashError::Unaligned);
        }

        for chunk in bytes.chunks_exact_mut(Self::READ_SIZE) {
            let word = unsafe { read_volatile(offset as *const u32) };
            chunk.copy_from_slice(&word.to_le_bytes());
            offset += Self::READ_SIZE as u32;
        }

        Ok(())
    }

    fn capacity(&self) -> usize {
        128 * 1024
    }
}
