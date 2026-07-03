//! 0x3E TesterPresent handler.

use super::nrc::Nrc;
use super::state::{store_response, UdsState};

pub fn handle(_state: &mut UdsState, req: &[u8]) {
    // [0x3E, subfunc] → [0x7E, subfunc]
    if req.len() != 2 {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x3E));
        return;
    }
    let subfunc = req[1];
    if subfunc & 0x7F == 0x00 {
        if subfunc & 0x80 != 0 {
            // Suppress positive response (bit 7 set): server
            // stays silent. We store an empty response; the
            // caller (sdo::dispatch) treats `response_len == 0`
            // as "no payload to upload".
            store_response(&[]);
        } else {
            store_response(&[0x7E, 0x00]);
        }
    } else {
        store_response(&Nrc::SubFunctionNotSupported.negative_response(0x3E));
    }
}
