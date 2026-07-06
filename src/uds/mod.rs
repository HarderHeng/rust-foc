//! UDS (ISO 14229) — application layer protocol adapter.
//!
//! **Layering**: this module is the glue between the platform-
//! independent `uds-core` crate and the rest of the firmware.
//! It owns the two statics (`UDS_STATE`, `UDS_CONFIG`) and
//! the three public entry points (`dispatch`, `tick`, `tx_disabled`).
//!
//! The `uds-core` crate holds all pure protocol logic (types,
//! dispatch tables, crypto, pending queue, DTC storage, and the
//! `dispatch_sid` router). See `uds-core/` for its module map.

pub mod static_config;

use uds_core::pending;
pub use uds_core::table::{take_reset_request, take_reset_subfunc};
use uds_core::{UdsConfig, UdsState};

/// Top-level UDS state. Single-threaded executor is the sole owner.
pub static UDS_STATE: UdsState = UdsState::zeroed();

/// Public helper: returns true if the canopen task should
/// suppress its proactive frames (heartbeat, NMT ACK,
/// UDS responses) because the master issued
/// 0x28 0x03 disableNormalCommunication.
pub fn tx_disabled() -> bool {
    // Safety: single-threaded executor; immutable read.
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
/// The transport adapter (e.g. `src/can/uds_bridge.rs`) calls
/// this with the decoded payload.
///
/// The actual routing logic lives in `UdsConfig::dispatch_sid()`
/// inside the `uds-core` crate — this function only acquires
/// the two static references and delegates.
pub fn dispatch(request: &[u8]) {
    let state = unsafe { &mut *(&raw const UDS_STATE as *mut UdsState) };
    let config = unsafe { &mut *(&raw const static_config::UDS_CONFIG as *mut UdsConfig) };
    config.dispatch_sid(state, request);
}
