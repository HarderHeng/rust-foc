//! 0x11 ECUReset handler.

use core::sync::atomic::{AtomicBool, Ordering};
use defmt::info;

use super::nrc::Nrc;
use super::state::{store_response, UdsState};

/// Reset requested by 0x11 HardReset. Polled by the canopen task;
/// resets the chip via NVIC after the response has gone out.
pub static RESET_REQUESTED: AtomicBool = AtomicBool::new(false);

pub fn take_reset_request() -> bool {
    RESET_REQUESTED.swap(false, Ordering::Relaxed)
}

pub fn handle(_state: &mut UdsState, req: &[u8]) {
    if req.len() != 2 {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x11));
        return;
    }
    let subfunc = req[1];
    match subfunc {
        0x01 => {
            // HardReset: stash positive response, arm reset.
            RESET_REQUESTED.store(true, Ordering::Relaxed);
            info!("UDS: ECUReset(Hard) requested");
            store_response(&[0x51, 0x01]);
        }
        // SoftReset (0x03) not implemented in v1.
        _ => {
            store_response(&Nrc::SubFunctionNotSupported.negative_response(0x11));
        }
    }
}
