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
//!
//! C11: both statics are wrapped in
//! `critical_section::Mutex<RefCell<T>>` (matches the
//! `store_response` pattern in `uds-core/src/state.rs`).
//! The `RefCell` provides `&mut T` for the duration of the
//! critical section, so we can hold both `&mut UdsState` and
//! `&mut UdsConfig` simultaneously inside `dispatch` / `tick`.
//! The previous `static mut` + `&mut *(&raw const … as *mut T)`
//! pattern was Stacked-Borrows UB whenever `write_key_masks`
//! re-entered the dispatcher.

pub mod static_config;

use core::cell::RefCell;
use critical_section::Mutex;
use uds_core::pending;
pub use uds_core::table::{take_reset_request, take_reset_subfunc};
use uds_core::UdsState;

/// Top-level UDS state. C11: wrapped in
/// `critical_section::Mutex<RefCell<UdsState>>` so access
/// goes through `borrow_ref` / `borrow_ref_mut` with a CS
/// token.
pub static UDS_STATE: Mutex<RefCell<UdsState>> =
    Mutex::new(RefCell::new(UdsState::zeroed()));

/// Public helper: returns true if the canopen task should
/// suppress its proactive frames (heartbeat, NMT ACK,
/// UDS responses) because the master issued
/// 0x28 0x03 disableNormalCommunication.
pub fn tx_disabled() -> bool {
    critical_section::with(|cs| UDS_STATE.borrow_ref(cs).tx_disabled)
}

/// Public helper: check and clear the `response_pending` flag.
/// Returns `true` if a pending-queue job completed and stored
/// a response that should be transmitted.
pub fn take_response_pending() -> bool {
    critical_section::with(|cs| {
        let state = &mut *UDS_STATE.borrow_ref_mut(cs);
        if state.response_pending {
            state.response_pending = false;
            true
        } else {
            false
        }
    })
}

/// Public helper: drive the pending queue. Called by
/// canopen_task every tick. `now_ms` is the current
/// millisecond clock.
pub fn tick(now_ms: u32) {
    // Acquire both locks inside one CS. The RefCells give us
    // simultaneous `&mut` access because they wrap different
    // allocations (separate statics). The CS prevents
    // interrupt-level re-entry between the two `borrow_ref_mut`
    // calls.
    critical_section::with(|cs| {
        let state = &mut *UDS_STATE.borrow_ref_mut(cs);
        let config = &mut *static_config::UDS_CONFIG.borrow_ref_mut(cs);
        pending::tick(state, config, now_ms);
    });
}

/// Dispatch a UDS request. `request[0]` is the SID. The
/// response is stored in the shared buffer; the caller reads
/// it via `uds_core::load_response()` after the dispatch returns.
///
/// `now_ms` is the current timestamp in milliseconds, used to
/// stamp `request_tick_ms` for P2/P2* timeout tracking.
///
/// This is the sole public entry point into the UDS engine.
/// The transport adapter (e.g. `src/can/uds_bridge.rs`) calls
/// this with the decoded payload.
///
/// The actual routing logic lives in `UdsConfig::dispatch_sid()`
/// inside the `uds-core` crate — this function only acquires
/// the two static references and delegates.
pub fn dispatch(request: &[u8], now_ms: u32) {
    critical_section::with(|cs| {
        let state = &mut *UDS_STATE.borrow_ref_mut(cs);
        let config = &mut *static_config::UDS_CONFIG.borrow_ref_mut(cs);
        config.dispatch_sid(state, request, now_ms);
    });
}
