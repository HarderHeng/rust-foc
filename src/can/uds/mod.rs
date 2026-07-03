//! UDS (ISO 14229) module — Phase 5a.
//!
//! Architecture (see `docs/superpowers/specs/2026-07-03-uds-rewrite-design.md`):
//!
//! - Table-driven dispatch: `config.services` maps SID → handler
//!   with declarative session/SAL gates
//! - Multi-SAL state machine (SAL1/2/3) with handshake tracking
//! - LFSR key derivation (SAL-specific masks)
//! - 23 NRCs (full ISO 14229-1 set we care about)
//!
//! Phase 5a status: synchronous dispatch (no pending queue /
//! 0x78 yet — those land in Phase 5c when OTA is rewired).
//! All 20 existing smoke-test scenarios must continue to pass.

use defmt::info;

pub mod comm_control;
pub mod config;
pub mod download;
pub mod dtc;
pub mod nrc;
pub mod pending;
pub mod read_data;
pub mod reset;
pub mod routine;
pub mod security;
pub mod session;
pub mod state;
pub mod tester_present;
pub mod write_data;

use config::{ServiceHandler, UdsConfig};
use nrc::Nrc;
use state::{store_response, UdsState};

pub use pending::{DispatchResult, take_response, tick as tick_pending};
pub use state::SrvState;

/// Backwards-compat shim for `crate::can::ota::take_reset_request`.
/// Phase 4 used to call `uds::take_reset_request()`; now we re-export
/// the per-module flag from `reset::take_reset_request`.
pub use reset::take_reset_request;

/// Public helper: returns true if the canopen task should
/// suppress its proactive frames (heartbeat, NMT ACK) because
/// the master has issued 0x28 0x03 disableNormalCommunication.
pub fn tx_disabled() -> bool {
    // Safety: single-threaded executor.
    unsafe { (&*(&raw const UDS_STATE)).tx_disabled }
}

/// Public helper: drive the pending queue. Called by canopen_task
/// every tick. `now_ms` is the current millisecond clock.
pub fn tick(now_ms: u32) {
    let state = unsafe { &mut *(&raw const UDS_STATE as *mut UdsState) };
    let config = unsafe { &mut *(&raw mut crate::can::uds_config::UDS_CONFIG) };
    pending::tick(state, config, now_ms);
}

/// Top-level UDS state. `static` so it's a single instance.
pub static UDS_STATE: UdsState = UdsState::zeroed();

/// True iff a UDS response is ready in the shared buffer.
pub fn response_ready() -> bool {
    // Safety: single-threaded.
    unsafe { (*(&raw const UDS_STATE)).response_pending }
}

/// Dispatch a UDS request. `request[0]` is the SID. The response
/// is stored in the shared buffer; the canopen task reads it
/// after the dispatch returns (sync case) or after the
/// pending queue's `tick` (async OTA case).
///
/// **Lives in `.data` (RAM).** Called from `uds_transport::
/// handle_rx_frame` on every UDS request. Keeping the
/// dispatch chain in RAM means the entire UDS path stays
/// off the OTA write path.
///
/// **Lives in `.data` (RAM).** Called from `od::write` on every
/// SDO write to 0x2F00.0. The entire UDS dispatch chain stays
/// off the OTA write path.
#[inline(never)]
#[link_section = ".data"]
pub fn dispatch(request: &[u8]) {
    // Safety: single-threaded executor; we get `&mut UDS_STATE`
    // by taking the address of a `static`. This is OK because
    // the canopen task is the sole owner of UDS calls.
    let state = unsafe { &mut *(&raw const UDS_STATE as *mut UdsState) };
    let config: &'static UdsConfig = unsafe { &*(&raw const crate::can::uds_config::UDS_CONFIG) };

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

    if !config::session_allowed(state.session, entry.session_access) {
        store_response(&Nrc::ServiceNotSupportedInActiveSession
            .negative_response(sid));
        return;
    }
    if !config::security_allowed(state.security, entry.security_level) {
        store_response(&Nrc::SecurityAccessDenied.negative_response(sid));
        return;
    }

    // Copy the request bytes into the in-flight buffer so
    // pending-queue closures can read them via `UdsContext`.
    state.request_len = request.len();
    state.request_buf[..request.len()].copy_from_slice(request);

    // OTA path needs `&mut UdsConfig` (to push onto the
    // pending queue). Take a mut pointer through unsafe —
    // single-threaded executor, canopen task is the sole
    // owner. Other handlers use the shared `config`.
    let config_mut: &mut UdsConfig = unsafe {
        &mut *(&raw const crate::can::uds_config::UDS_CONFIG as *mut UdsConfig)
    };

    match entry.handler {
        ServiceHandler::Session        => session::handle(state, config, request),
        ServiceHandler::EcuReset       => reset::handle(state, request),
        ServiceHandler::ClearDtc       => dtc::handle_clear(state, request),
        ServiceHandler::ReadDtc        => dtc::handle_read(state, request),
        ServiceHandler::ReadDataById   => read_data::handle(state, config, request),
        ServiceHandler::WriteDataById  => write_data::handle(state, config, request),
        ServiceHandler::CommControl    => comm_control::handle(state, request),
        ServiceHandler::SecurityAccess => security::handle(state, config, request),
        ServiceHandler::RoutineStart   => routine::handle(state, config, request, routine::RoutineSub::Start),
        // OTA path (Phase 7): push to pending queue. The
        // closure runs in `pending::tick`. While the work is
        // in-flight, `state.state == SrvState::Pending` and
        // subsequent dispatches return `DispatchResult::Pending`
        // (master should back off and poll later).
        ServiceHandler::RequestDownload => {
            let _queued = download::try_queue_request(state, config_mut);
        }
        ServiceHandler::TransferData => {
            let _queued = download::try_queue_transfer(state, config_mut);
        }
        ServiceHandler::TransferExit => {
            let _queued = download::try_queue_exit(state, config_mut);
        }
        ServiceHandler::TesterPresent   => tester_present::handle(state, request),
    }
}
