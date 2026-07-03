//! 0x22 ReadDataByIdentifier handler.
//!
//! Phase 5a: one DID — 0xF186 (ActiveDiagSession) — reading the
//! current session byte. Adding more DIDs is a single
//! `DidReadEntry` in `src/can/uds_config.rs`; no dispatcher change.

use super::config::UdsConfig;
use super::nrc::Nrc;
use super::state::{store_response, UdsState};

pub fn handle(state: &mut UdsState, config: &UdsConfig, req: &[u8]) {
    // [0x22, did_lo, did_hi] → [0x62, did_lo, did_hi, ...data]
    if req.len() != 3 {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x22));
        return;
    }
    let did = u16::from_le_bytes([req[1], req[2]]);

    let entry = match config.read_dids.iter().find(|e| e.did == did) {
        Some(e) => e,
        None => {
            store_response(&Nrc::RequestOutOfRange.negative_response(0x22));
            return;
        }
    };

    if !super::config::session_allowed(state.session, entry.session_access) {
        store_response(&Nrc::ServiceNotSupportedInActiveSession
            .negative_response(0x22));
        return;
    }
    if !super::config::security_allowed(state.security, entry.security_level) {
        store_response(&Nrc::SecurityAccessDenied.negative_response(0x22));
        return;
    }

    let mut payload = [0u8; 7];
    match (entry.func)(&mut payload) {
        Ok(n) => {
            let mut out = [0u8; 7];
            out[0] = 0x62;
            out[1] = req[1];
            out[2] = req[2];
            for i in 0..n.min(5) {
                out[3 + i] = payload[i];
            }
            store_response(&out[..3 + n.min(5)]);
        }
        Err(nrc) => {
            store_response(&nrc.negative_response(0x22));
        }
    }
}
