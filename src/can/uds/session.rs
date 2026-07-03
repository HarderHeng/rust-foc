//! 0x10 DiagnosticSessionControl handler.
//!
//! Wire format:
//!   [0x10, subfunc] → [0x50, subfunc]
//!   subfunc 0x01 = DefaultSession
//!   subfunc 0x02 = ProgrammingSession (requires SAL1 unlocked)
//!   subfunc 0x03 = ExtendedSession
//!
//! Session transitions clear the SecurityAccess state per
//! ISO 14229 (every session change invalidates prior security
//! state). On entry, the configured `on_*_session_enter`
//! callback fires (Phase 5b: noop stubs; real logging or
//! lock-out can be added in `src/can/uds_config.rs`).

use defmt::info;

use super::config::UdsConfig;
use super::nrc::Nrc;
use super::state::{
    SecurityLevel, Session, UdsState, store_response,
};

pub fn handle(state: &mut UdsState, config: &UdsConfig, req: &[u8]) {
    if req.len() != 2 {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x10));
        return;
    }
    let subfunc = req[1];
    let new_session = match Session::from_u8(subfunc) {
        Some(s) => s,
        None => {
            store_response(&Nrc::SubFunctionNotSupported.negative_response(0x10));
            return;
        }
    };

    // ProgrammingSession is privileged: requires SAL1.
    // ExtendedSession: also requires SAL1 (kept strict to
    // mirror Phase 4 behaviour; ISO 14229 leaves the gate
    // up to the implementer).
    if matches!(new_session, Session::Programming | Session::Extended)
        && state.security < SecurityLevel::Sal1
    {
        store_response(&Nrc::SecurityAccessDenied.negative_response(0x10));
        return;
    }

    state.session = new_session;
    // ISO 14229: session change invalidates security.
    state.security = SecurityLevel::Locked;
    state.seed_sent = [false; 3];

    // Fire session-enter callback.
    let cb = match new_session {
        Session::Default => config.on_default_session_enter,
        Session::Programming => config.on_programming_session_enter,
        Session::Extended => config.on_extended_session_enter,
    };
    if let Some(cb) = cb { cb(); }

    info!("UDS: session → 0x{:02x}", subfunc);
    store_response(&[0x50, subfunc]);
}
