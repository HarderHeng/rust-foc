//! OTA flag operations: shared trait + flash-backed implementation.

use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

use crate::{OTA_FLAG_NONE, OTA_FLAG_PENDING};

/// Current state of the OTA flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtaState {
    /// App requested OTA — bootloader enters y-modem mode.
    Pending,
    /// Normal: bootloader should jump to app.
    None,
}

/// Errors from flag operations. Wrapper over the underlying flash error.
#[derive(Debug)]
pub enum FlagError<F: NorFlash> {
    /// Underlying flash error.
    Flash(F::Error),
}

impl<F: NorFlash> core::fmt::Display for FlagError<F> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Flash(_) => write!(f, "flash error reading/writing OTA flag"),
        }
    }
}

/// Abstraction over OTA flag operations. App and bootloader both depend on this.
pub trait OtaFlag {
    /// Associated error type for fallible operations.
    type Error;

    /// Read the current flag state.
    /// Takes `&mut self` because the underlying `ReadNorFlash::read` requires it.
    fn read(&mut self) -> OtaState;

    /// Set the flag to Pending (bootloader will enter y-modem mode on next boot).
    fn set_pending(&mut self) -> Result<(), Self::Error>;

    /// Clear the flag to None (bootloader will jump to app on next boot).
    fn clear(&mut self) -> Result<(), Self::Error>;
}

/// Flash-backed `OtaFlag` implementation.
/// `F` is any `NorFlash` + `ReadNorFlash` (the latter is required to read the flag).
pub struct FlashOtaFlag<F: NorFlash + ReadNorFlash> {
    storage: F,
    addr: u32,
}

impl<F: NorFlash + ReadNorFlash> FlashOtaFlag<F> {
    /// Create a new flag accessor. Caller must own the flash.
    pub fn new(storage: F, addr: u32) -> Self {
        Self { storage, addr }
    }

    /// Read the raw flag byte (1 byte, may be 0x00 or 0xAA).
    fn read_byte(&mut self) -> u8 {
        let mut buf = [0u8; 1];
        // Ignore error — a failed read returns 0x00 which is the safe default (None).
        let _ = self.storage.read(self.addr, &mut buf);
        buf[0]
    }
}

impl<F: NorFlash + ReadNorFlash> OtaFlag for FlashOtaFlag<F> {
    type Error = FlagError<F>;

    fn read(&mut self) -> OtaState {
        match self.read_byte() {
            OTA_FLAG_PENDING => OtaState::Pending,
            _ => OtaState::None,
        }
    }

    fn set_pending(&mut self) -> Result<(), Self::Error> {
        // STM32G4 flash pages are 2 KB and erases must be page-aligned.
        // The flag byte is the only state on this page, so we erase the whole page.
        const PAGE_SIZE: u32 = 2048;
        let page_start = self.addr & !(PAGE_SIZE - 1);
        self.storage
            .erase(page_start, page_start + PAGE_SIZE)
            .map_err(FlagError::Flash)?;
        self.storage
            .write(self.addr, &[OTA_FLAG_PENDING])
            .map_err(FlagError::Flash)
    }

    fn clear(&mut self) -> Result<(), Self::Error> {
        // Same page-erase as set_pending: STM32G4 requires page-aligned erases.
        const PAGE_SIZE: u32 = 2048;
        let page_start = self.addr & !(PAGE_SIZE - 1);
        self.storage
            .erase(page_start, page_start + PAGE_SIZE)
            .map_err(FlagError::Flash)?;
        self.storage
            .write(self.addr, &[OTA_FLAG_NONE])
            .map_err(FlagError::Flash)
    }
}
