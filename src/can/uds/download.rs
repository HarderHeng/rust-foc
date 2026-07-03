//! 0x34/0x36/0x37 RequestDownload/TransferData/TransferExit
//! glue. The actual flash work lives in `super::super::ota`;
//! this module just routes the request and dispatches the
//! response.
//!
//! Phase 5a: synchronous (the existing OTA path). Phase 5c will
//! add the pending queue + 0x78 path for long operations.

use super::nrc::Nrc;
use super::state::{store_response, UdsState};
use crate::can::ota;

pub fn handle_request(state: &mut UdsState, req: &[u8]) {
    // req: [0x34, dataFormat, addr(4), size(4)] — but Phase 4 v1
    // uses [0x34, 0x00, size_lo, size_hi, size_hi2, size_hi3]
    // (no address; flash writes start at APP_START). Match
    // that wire format for back-compat.
    if req.len() != 6 || req[1] != 0x00 {
        store_response(&Nrc::RequestOutOfRange.negative_response(0x34));
        return;
    }
    let _ = ota::handle_request_download(&req[1..]);
    // OTA writes the response directly via store_uds_positive/
    // negative; nothing to do here.
    let _ = state; // suppress unused
}

pub fn handle_transfer(state: &mut UdsState, req: &[u8]) {
    if req.len() != 4 {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x36));
        return;
    }
    let _ = ota::handle_transfer_data(&req[1..]);
    let _ = state;
}

pub fn handle_exit(state: &mut UdsState, req: &[u8]) {
    if !req.is_empty() {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x37));
        return;
    }
    let _ = ota::handle_transfer_exit(&[]);
    let _ = state;
}
