//! UDS engine runtime state (RAM-resident) + the shared
//! response buffer used by the dispatcher and the transport
//! adapter.
//!
//! Protocol enums (`Session`, `SecurityLevel`, `SrvState`,
//! `Nrc`) live in `types.rs`; this module holds only the
//! engine's mutable state and its I/O helpers.

use core::cell::RefCell;
use core::sync::atomic::Ordering;
use critical_section::Mutex;

use super::types::{Session, SecurityLevel, SrvState};

/// UDS engine runtime state. Single-threaded owner (canopen
/// task). `request_buf` is set at the start of `dispatch`;
/// pending-queue closures read it back via `UdsContext`.
/// `response_pending = true` ⇒ `RESPONSE_BUF[0..response_len]`
/// holds a response; the canopen task reads + clears.
pub struct UdsState {
    pub session: Session,
    pub security: SecurityLevel,
    /// `true` after a RequestSeed, cleared after the matching
    /// SendKey. Prevents out-of-order SendKey.
    pub seed_sent: bool,
    pub current_seed: [u8; 16],

    pub state: SrvState,
    pub request_buf: [u8; 64],
    pub request_len: usize,
    pub response_pending: bool,
    pub request_tick_ms: u32,

    /// 0x28 CommControl: `true` ⇒ canopen task skips proactive
    /// frames (heartbeat, NMT ACK, UDS responses).
    pub tx_disabled: bool,
}

impl UdsState {
    pub const fn zeroed() -> Self {
        Self {
            session: Session::Default,
            security: SecurityLevel::Locked,
            seed_sent: false,
            current_seed: [0u8; 16],
            state: SrvState::Idle,
            request_buf: [0; 64],
            request_len: 0,
            response_pending: false,
            request_tick_ms: 0,
            tx_disabled: false,
        }
    }
}

/// Shared response buffer. 64 bytes matches CAN-FD max payload
/// (ISO 14229-3 §7 — UDS over CAN-FD single frame).
pub type ResponseBuf = Mutex<RefCell<[u8; 64]>>;

pub static RESPONSE_BUF: ResponseBuf = Mutex::new(RefCell::new([0; 64]));
pub static RESPONSE_LEN: core::sync::atomic::AtomicU8 =
    core::sync::atomic::AtomicU8::new(0);

/// Write a UDS response into the shared buffer. Also sets
/// `UDS_STATE.response_pending = true` so the canopen task
/// picks it up. Returns the byte count written (clipped to 64).
///
/// **Truncation**: the internal buffer is 64 bytes. Payloads
/// >64 bytes will be silently truncated. None of the built-in
/// SIDs exceed this — the longest response is an AES-128
/// seed (18 bytes).
pub fn store_response(payload: &[u8]) -> usize {
    debug_assert!(payload.len() <= 64,
                  "UDS response {} bytes exceeds 64-byte buffer",
                  payload.len());
    let len = payload.len().min(64);
    critical_section::with(|cs| {
        let buf = &mut *RESPONSE_BUF.borrow_ref_mut(cs);
        buf[..len].copy_from_slice(&payload[..len]);
    });
    RESPONSE_LEN.store(len as u8, Ordering::Relaxed);
    // Safety: single-threaded executor.
    unsafe {
        let p = &raw const crate::uds::UDS_STATE
               as *mut crate::uds::UdsState;
        (*p).response_pending = true;
    }
    len
}

/// Read the last stored UDS response. Returns `(bytes, len)`.
pub fn load_response() -> ([u8; 64], u8) {
    let mut bytes = [0u8; 64];
    let len = RESPONSE_LEN.load(Ordering::Relaxed);
    critical_section::with(|cs| {
        let buf = RESPONSE_BUF.borrow_ref(cs);
        for i in 0..(len as usize).min(64) {
            bytes[i] = buf[i];
        }
    });
    (bytes, len)
}
