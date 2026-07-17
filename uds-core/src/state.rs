//! UDS engine runtime state (RAM-resident) + the shared
//! response buffer used by the dispatcher and the transport
//! adapter.
//!
//! Protocol enums (`Session`, `SecurityLevel`, `SrvState`,
//! `Nrc`) live in `types.rs`; this module holds only the
//! engine's mutable state and its I/O helpers.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, Ordering};
use critical_section::Mutex;

use crate::types::{Session, SecurityLevel, SrvState};

/// UDS engine runtime state. Single-threaded owner (canopen
/// task). `request_buf` is set at the start of `dispatch`;
/// pending-queue closures read it back via `UdsContext`.
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
    /// Set by `store_response` or `tick` when a response is ready.
    /// Polled by the transport layer.
    pub response_pending: bool,
    pub request_tick_ms: u32,

    /// 0x28 CommControl: `true` ⇒ transport skips proactive frames.
    pub tx_disabled: bool,

    /// SecurityAccess (0x27) consecutive failed SendKey attempts.
    /// Resets on successful unlock, session change, or power cycle.
    /// When ≥ `config.sa_max_attempts`, returns 0x36 ExceededNumberOfAttempts.
    pub sa_fail_count: u8,

    /// Snapshotted SID of the pending operation. Set by `push_pending`
    /// when transitioning to `SrvState::Pending`; used by the tick
    /// loop to emit correct 0x78 ResponsePending frames even when a
    /// new request has overwritten `request_buf`. 0 means no pending
    /// operation.
    pub pending_sid: u8,
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
            sa_fail_count: 0,
            pending_sid: 0,
        }
    }
}

/// Shared response buffer. 64 bytes matches CAN-FD max payload
/// (ISO 14229-3 §7 — UDS over CAN-FD single frame).
const RESPONSE_BUF_SIZE: usize = 64;

static RESPONSE_BUF: Mutex<RefCell<[u8; RESPONSE_BUF_SIZE]>> =
    Mutex::new(RefCell::new([0; RESPONSE_BUF_SIZE]));
static RESPONSE_LEN: AtomicU8 = AtomicU8::new(0);

/// Write a UDS response into the shared buffer. Returns the byte
/// count written (clipped to 64).
///
/// **Truncation**: the internal buffer is 64 bytes. Payloads
/// >64 bytes will be silently truncated.
pub fn store_response(payload: &[u8]) -> usize {
    debug_assert!(payload.len() <= RESPONSE_BUF_SIZE,
                  "UDS response {} bytes exceeds {} byte buffer",
                  payload.len(), RESPONSE_BUF_SIZE);
    let len = payload.len().min(RESPONSE_BUF_SIZE);
    critical_section::with(|cs| {
        let buf = &mut *RESPONSE_BUF.borrow_ref_mut(cs);
        buf[..len].copy_from_slice(&payload[..len]);
    });
    RESPONSE_LEN.store(len as u8, Ordering::Relaxed);
    len
}

/// Read the last stored UDS response. Returns `(bytes, len)`.
pub fn load_response() -> ([u8; 64], u8) {
    let mut bytes = [0u8; 64];
    let len = RESPONSE_LEN.load(Ordering::Relaxed);
    critical_section::with(|cs| {
        let buf = RESPONSE_BUF.borrow_ref(cs);
        let n = (len as usize).min(64);
        for i in 0..n {
            bytes[i] = buf[i];
        }
    });
    (bytes, len)
}
