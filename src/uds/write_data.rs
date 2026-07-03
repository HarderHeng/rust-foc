//! 0x2E WriteDataByIdentifier handler.
//!
//! Phase 5a: no DIDs are writable. Always returns 0x31. Phase 5b
//! will add DIDs (e.g. 0xF186 echo, vendor-specific entries).

use super::config::UdsConfig;
use super::nrc::Nrc;
use super::state::{store_response, UdsState};

pub fn handle(state: &mut UdsState, config: &UdsConfig, req: &[u8]) {
    // [0x2E, did_lo, did_hi, data...] → [0x6E, did_lo, did_hi] or NRC
    if req.len() < 3 {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x2E));
        return;
    }
    let did = u16::from_le_bytes([req[1], req[2]]);

    let entry = match config.write_dids.iter().find(|e| e.did == did) {
        Some(e) => e,
        None => {
            store_response(&Nrc::RequestOutOfRange.negative_response(0x2E));
            return;
        }
    };

    if !super::config::session_allowed(state.session, entry.session_access) {
        store_response(&Nrc::ServiceNotSupportedInActiveSession
            .negative_response(0x2E));
        return;
    }
    if !super::config::security_allowed(state.security, entry.security_level) {
        store_response(&Nrc::SecurityAccessDenied.negative_response(0x2E));
        return;
    }

    match (entry.func)(&req[3..]) {
        Ok(()) => {
            store_response(&[0x6E, req[1], req[2]]);
        }
        Err(nrc) => {
            store_response(&nrc.negative_response(0x2E));
        }
    }
}

// `state` is currently unused; the `&mut` keeps the call-site
// shape uniform with the other handlers (and is needed for
// future DIDs that mutate state).
#[allow(dead_code)]
fn _state_unused(_s: &UdsState) {}
