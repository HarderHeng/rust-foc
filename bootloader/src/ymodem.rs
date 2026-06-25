//! y-modem receive protocol. Full implementation in Task 5.
//!
//! Task 4 leaves this as a stub that returns an error so the rest of the
//! state machine compiles and we can verify the flash + flag + jump path.

use crate::flash::Stm32g4Flash;

pub fn receive_image(_flash: &mut Stm32g4Flash) -> Result<(), YmodemError> {
    Err(YmodemError::NotImplemented)
}

#[derive(Debug)]
pub enum YmodemError {
    /// Stub error so Task 4 compiles. Task 5 replaces this with the real set.
    NotImplemented,
    // Real error variants go here in Task 5:
    // - Timeout
    // - Aborted (CAN received)
    // - InvalidPacket
    // - CrcMismatch
    // - FlashError(FlashError)
}
