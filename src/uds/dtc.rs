//! 0x14 ClearDiagnosticInformation and 0x19 ReadDTCInformation
//! handlers.

use defmt::info;

use super::nrc::Nrc;
use super::state::{store_response, UdsState};

/// 0x14 ClearDiagnosticInformation. We have no DTCs to clear (no
/// OBD-II stack). Always succeed.
pub fn handle_clear(_state: &mut UdsState, req: &[u8]) {
    // [0x14, group(3)] → [0x54]
    if req.len() != 4 {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x14));
        return;
    }
    store_response(&[0x54]);
}

/// 0x19 ReadDTCInformation. Phase 5a implements subfunc 0x02
/// (reportDTCByStatusMask) only — returns "0 DTCs".
pub fn handle_read(_state: &mut UdsState, req: &[u8]) {
    // [0x19, subfunc, status_mask] → [0x59, subfunc, avail, fmt]
    if req.len() < 3 {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x19));
        return;
    }
    let subfunc = req[1];
    if subfunc != 0x02 {
        store_response(&Nrc::SubFunctionNotSupported.negative_response(0x19));
        return;
    }
    info!("UDS: ReadDTC(subfunc=0x02) → 0 DTCs");
    store_response(&[0x59, 0x02, 0x00, 0x00]);
}
