//! UDS (ISO 14229) — application layer protocol adapter.
//!
//! **Layering**: this module is the glue between the platform-
//! independent `uds-core` crate and the rest of the firmware.
//! It owns the static `UDS_STATE` instance, the static
//! `UDS_CONFIG` instance (defined in `static_config.rs`), and
//! the top-level `dispatch()` entry point.
//!
//! The `uds-core` crate holds all pure protocol logic (types,
//! dispatch tables, crypto, pending queue, DTC storage). See
//! `uds-core/` or `docs/uds-crate.md` for its module map.

pub mod static_config;

use defmt::info;
use uds_core::pending;
pub use uds_core::table::{take_reset_request, take_reset_subfunc};
use uds_core::table::ServiceHandler;
use uds_core::types::Nrc;
use uds_core::{store_response, UdsConfig, UdsState};

/// Top-level UDS state. `static` so it's a single instance.
pub static UDS_STATE: UdsState = UdsState::zeroed();

/// Public helper: returns true if the canopen task should
/// suppress its proactive frames (heartbeat, NMT ACK,
/// UDS responses) because the master issued
/// 0x28 0x03 disableNormalCommunication.
pub fn tx_disabled() -> bool {
    // Safety: single-threaded executor.
    unsafe { (&*(&raw const UDS_STATE)).tx_disabled }
}

/// Public helper: drive the pending queue. Called by
/// canopen_task every tick. `now_ms` is the current
/// millisecond clock.
pub fn tick(now_ms: u32) {
    let state = unsafe { &mut *(&raw const UDS_STATE as *mut UdsState) };
    let config = unsafe { &mut *(&raw const static_config::UDS_CONFIG as *mut UdsConfig) };
    pending::tick(state, config, now_ms);
}

/// Dispatch a UDS request. `request[0]` is the SID. The
/// response is stored in the shared buffer; the caller reads
/// it via `uds_core::load_response()` after the dispatch returns.
///
/// This is the sole public entry point into the UDS engine.
/// It is platform-independent and transport-agnostic: it takes
/// a `&[u8]` request and produces a response in the internal
/// buffer. The transport adapter (e.g. `src/can/uds_bridge.rs`)
/// calls this with the decoded payload.
pub fn dispatch(request: &[u8]) {
    let state = unsafe { &mut *(&raw const UDS_STATE as *mut UdsState) };

    // Single `&mut` reference to UDS_CONFIG — Rust reborrows as
    // `&` for `&self` dispatch methods and as `&mut` for the OTA
    // methods that push to the pending queue. Single-threaded
    // executor is the sole owner.
    let config: &mut UdsConfig = unsafe {
        &mut *(&raw const static_config::UDS_CONFIG as *mut UdsConfig)
    };

    if request.is_empty() {
        return;
    }
    let sid = request[0];
    let entry = match config.services.iter().find(|e| e.sid == sid) {
        Some(e) => e,
        None => {
            store_response(&Nrc::ServiceNotSupported.negative_response(sid));
            return;
        }
    };

    if !uds_core::table::session_allowed(state.session, entry.session_access) {
        store_response(&Nrc::ServiceNotSupportedInActiveSession
            .negative_response(sid));
        return;
    }
    if !uds_core::table::security_allowed(state.security, entry.security_level) {
        store_response(&Nrc::SecurityAccessDenied.negative_response(sid));
        return;
    }

    // Copy the request bytes into the in-flight buffer so
    // pending-queue closures can read them via `UdsContext`.
    state.request_len = request.len();
    state.request_buf[..request.len()].copy_from_slice(request);

    match entry.handler {
        ServiceHandler::Session        => config.dispatch_0x10(state, request),
        ServiceHandler::EcuReset       => config.dispatch_0x11(state, request),
        ServiceHandler::ClearDtc       => config.dispatch_0x14(state, request),
        ServiceHandler::ReadDtc        => config.dispatch_0x19(state, request),
        ServiceHandler::ReadDataById   => config.dispatch_0x22(state, request),
        ServiceHandler::WriteDataById  => config.dispatch_0x2e(state, request),
        ServiceHandler::CommControl    => config.dispatch_0x28(state, request),
        ServiceHandler::SecurityAccess => config.dispatch_0x27(state, request),
        ServiceHandler::RoutineStart   => config.dispatch_0x31(state, request, uds_core::table::RoutineSub::Start),
        ServiceHandler::RoutineStop    => config.dispatch_0x31(state, request, uds_core::table::RoutineSub::Stop),
        ServiceHandler::RoutineResult  => config.dispatch_0x31(state, request, uds_core::table::RoutineSub::Result),
        ServiceHandler::RequestDownload => {
            let _queued = config.dispatch_0x34(state);
        }
        ServiceHandler::TransferData => {
            let _queued = config.dispatch_0x36(state);
        }
        ServiceHandler::TransferExit => {
            let _queued = config.dispatch_0x37(state);
        }
        ServiceHandler::TesterPresent   => config.dispatch_0x3e(state, request),
    }
    info!("UDS: dispatched SID 0x{:02x}", sid);
}
