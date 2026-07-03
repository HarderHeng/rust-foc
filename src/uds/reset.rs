//! 0x11 ECUReset handler.
//!
//! Subfuncs per ISO 14229-1:
//!   0x01 = HardReset  — power-cycle the ECU (NVIC system reset).
//!   0x03 = SoftReset  — restart the application without power
//!                        cycle (NVIC application reset if
//!                        available; we use the same NVIC path
//!                        but the canopen task checks the
//!                        subfunc and can branch if needed).
//!
//! Both stash a positive response and arm the reset flag. The
//! canopen task polls `take_reset_request()` after the response
//! has gone out and triggers the actual NVIC reset.

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use defmt::info;

use super::nrc::Nrc;
use super::state::{store_response, UdsState};

/// Reset requested by 0x11. Polled by the canopen task;
/// resets the chip via NVIC after the response has gone out.
pub static RESET_REQUESTED: AtomicBool = AtomicBool::new(false);

/// Subfunc of the last armed reset (1 = Hard, 3 = Soft).
/// Lets the canopen task branch behaviour (e.g. log differently).
pub static RESET_SUBFUNC: AtomicU8 = AtomicU8::new(0);

pub fn take_reset_request() -> bool {
    RESET_REQUESTED.swap(false, Ordering::Relaxed)
}

/// Subfunc of the last reset (`Some(1)` Hard, `Some(3)` Soft).
/// Cleared on read. Returns `None` if no reset pending.
pub fn take_reset_subfunc() -> Option<u8> {
    let s = RESET_SUBFUNC.swap(0, Ordering::Relaxed);
    if s == 0 { None } else { Some(s) }
}

pub fn handle(_state: &mut UdsState, req: &[u8]) {
    if req.len() != 2 {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x11));
        return;
    }
    let subfunc = req[1];
    match subfunc {
        0x01 | 0x03 => {
            RESET_REQUESTED.store(true, Ordering::Relaxed);
            RESET_SUBFUNC.store(subfunc, Ordering::Relaxed);
            info!("UDS: ECUReset({}) requested",
                  if subfunc == 0x01 { "Hard" } else { "Soft" });
            store_response(&[0x51, subfunc]);
        }
        _ => {
            store_response(&Nrc::SubFunctionNotSupported.negative_response(0x11));
        }
    }
}
