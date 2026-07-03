//! UDS (ISO 14229) — application layer protocol.
//!
//! **Layering**: UDS is transport-agnostic. It lives in `src/uds/`
//! at the top level (NOT under `src/can/`) because the
//! application layer should not depend on the physical / data
//! link layer that happens to carry it. The CAN-specific
//! transport adapter is in `src/uds/transport.rs` — the only
//! place in the UDS module that imports embassy-stm32's
//! FDCAN frame types.
//!
//! ## Module map
//!
//! - `types.rs`        — protocol enums (Session, SecurityLevel, SrvState, Nrc)
//! - `state.rs`        — engine runtime state + shared response buffer
//! - `table.rs`        — `UdsConfig` schema + `impl UdsConfig { dispatch_0xNN }`
//!                       (the *only* place that knows per-SID wire format)
//! - `static_config.rs`— the static `UDS_CONFIG` instance + callback fns
//! - `crypto.rs`       — LFSR + bit reversal (pure functions, unit-tested)
//! - `pending.rs`      — pending queue + 0x78 ResponsePending
//! - `transport.rs`    — CAN-specific transport adapter
//!
//! ## Adding a new SID
//!
//! 1. Add `dispatch_0xNN` method in `table.rs::impl UdsConfig`.
//! 2. Add `ServiceHandler::YourSid` variant in `table.rs`.
//! 3. Add a `ServiceEntry` row in `static_config.rs::SERVICES`.
//!    Add the dispatch arm in the `route` function in this file.
//!
//! ## Adding a new DID callback
//!
//! 1. Write the callback fn in `static_config.rs`.
//! 2. Add a `DidReadEntry` row in `static_config.rs::READ_DIDS`.
//!    Nothing else moves.

use defmt::info;

pub mod crypto;
pub mod pending;
pub mod state;
pub mod static_config;
pub mod table;
pub mod transport;
pub mod types;

use state::{store_response, UdsState};
pub use table::{take_reset_request, take_reset_subfunc, UdsConfig};
use types::Nrc;

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

/// True iff a UDS response is ready in the shared buffer.
pub fn response_ready() -> bool {
    // Safety: single-threaded.
    unsafe { (*(&raw const UDS_STATE)).response_pending }
}

/// Dispatch a UDS request. `request[0]` is the SID. The
/// response is stored in the shared buffer; the canopen task
/// reads it after the dispatch returns (sync case) or after
/// the pending queue's `tick` (async OTA case).
///
/// **Lives in `.data` (RAM).** Called from
/// `transport::handle_rx_frame` on every UDS request. Keeping
/// the dispatch chain in RAM means the entire UDS path stays
/// off the OTA write path.
#[inline(never)]
#[link_section = ".data"]
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

    if !table::session_allowed(state.session, entry.session_access) {
        store_response(&Nrc::ServiceNotSupportedInActiveSession
            .negative_response(sid));
        return;
    }
    if !table::security_allowed(state.security, entry.security_level) {
        store_response(&Nrc::SecurityAccessDenied.negative_response(sid));
        return;
    }

    // Copy the request bytes into the in-flight buffer so
    // pending-queue closures can read them via `UdsContext`.
    state.request_len = request.len();
    state.request_buf[..request.len()].copy_from_slice(request);

    match entry.handler {
        table::ServiceHandler::Session        => config.dispatch_0x10(state, request),
        table::ServiceHandler::EcuReset       => config.dispatch_0x11(state, request),
        table::ServiceHandler::ClearDtc       => config.dispatch_0x14(state, request),
        table::ServiceHandler::ReadDtc        => config.dispatch_0x19(state, request),
        table::ServiceHandler::ReadDataById   => config.dispatch_0x22(state, request),
        table::ServiceHandler::WriteDataById  => config.dispatch_0x2e(state, request),
        table::ServiceHandler::CommControl    => config.dispatch_0x28(state, request),
        table::ServiceHandler::SecurityAccess => config.dispatch_0x27(state, request),
        table::ServiceHandler::RoutineStart   => config.dispatch_0x31(state, request, table::RoutineSub::Start),
        table::ServiceHandler::RoutineStop    => config.dispatch_0x31(state, request, table::RoutineSub::Stop),
        table::ServiceHandler::RoutineResult  => config.dispatch_0x31(state, request, table::RoutineSub::Result),
        // OTA path (Phase 7): push to pending queue. The
        // closure runs in `pending::tick`. While the work is
        // in-flight, `state.state == SrvState::Pending` and
        // subsequent dispatches return `DispatchResult::Pending`
        // (master should back off and poll later).
        table::ServiceHandler::RequestDownload => {
            let _queued = config.dispatch_0x34(state);
        }
        table::ServiceHandler::TransferData => {
            let _queued = config.dispatch_0x36(state);
        }
        table::ServiceHandler::TransferExit => {
            let _queued = config.dispatch_0x37(state);
        }
        table::ServiceHandler::TesterPresent   => config.dispatch_0x3e(state, request),
    }
    info!("UDS: dispatched SID 0x{:02x}", sid);
}
