//! STM32G4 flash driver implementing `embedded_storage::NorFlash`.
//!
//! This is a minimal scaffold for Task 3. Task 4 wires up the page-erase and
//! write operations properly, and adds the half-word programming sequence the
//! STM32G4 flash controller requires (unlock, erase, write, lock).

use embedded_storage::nor_flash::{ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash};

/// STM32G4 flash error type (we use the PAC error directly for now).
#[derive(Debug)]
pub struct FlashError;

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}

/// Minimal placeholder. The real implementation lives in Task 4.
pub struct Stm32g4Flash;

impl Stm32g4Flash {
    pub fn new() -> Self {
        Self
    }
}

impl ErrorType for Stm32g4Flash {
    type Error = FlashError;
}

impl NorFlash for Stm32g4Flash {
    const WRITE_SIZE: usize = 8;
    const ERASE_SIZE: usize = 2048;

    fn erase(&mut self, _from: u32, _to: u32) -> Result<(), Self::Error> {
        Err(FlashError)
    }

    fn write(&mut self, _offset: u32, _bytes: &[u8]) -> Result<(), Self::Error> {
        Err(FlashError)
    }
}

impl ReadNorFlash for Stm32g4Flash {
    const READ_SIZE: usize = 4;
    fn read(&mut self, _offset: u32, _bytes: &mut [u8]) -> Result<(), Self::Error> {
        Err(FlashError)
    }
    fn capacity(&self) -> usize {
        // STM32G431CB has 128 KB flash. The bootloader only owns the first
        // 16 KB, but we report the full capacity so the address space matches
        // the absolute flash addresses used by the OtaFlag (0x0800_3F00 etc).
        128 * 1024
    }
}
