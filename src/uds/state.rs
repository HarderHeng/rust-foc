//! UDS runtime state — RAM-resident so the OTA write path
//! stays off the flash-resident call chain (see `super`).

use core::cell::RefCell;
use core::sync::atomic::Ordering;
use critical_section::Mutex;

use super::nrc::Nrc;

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Session {
    Default = 0x01,
    Programming = 0x02,
    Extended = 0x03,
}

impl Session {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x01 => Some(Self::Default),
            0x02 => Some(Self::Programming),
            0x03 => Some(Self::Extended),
            _ => None,
        }
    }
    pub const fn as_u8(self) -> u8 { self as u8 }
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum SecurityLevel {
    Locked = 0,
    Sal1 = 1,
    Sal2 = 2,
    Sal3 = 3,
}

impl SecurityLevel {
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Locked,
            1 => Self::Sal1,
            2 => Self::Sal2,
            _ => Self::Sal3,
        }
    }
    pub const fn as_u8(self) -> u8 { self as u8 }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SrvState {
    Idle,
    Pending,
}

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
    pub current_seed: u32,

    pub state: SrvState,
    pub request_buf: [u8; 64],
    pub request_len: usize,
    pub response_len: usize,
    pub response_pending: bool,
    pub request_tick_ms: u32,

    /// 0x28 CommControl: `true` ⇒ dispatcher rejects new
    /// requests with 0x22 (except 0x28 0x00 enable, the
    /// unlock path).
    pub tx_disabled: bool,
}

impl UdsState {
    pub const fn zeroed() -> Self {
        Self {
            session: Session::Default,
            security: SecurityLevel::Locked,
            seed_sent: false,
            current_seed: 0,
            state: SrvState::Idle,
            request_buf: [0; 64],
            request_len: 0,
            response_len: 0,
            response_pending: false,
            request_tick_ms: 0,
            tx_disabled: false,
        }
    }
}

/// Shared response buffer. Bounded to 7 bytes (SDO upload
/// ceiling; CAN-FD allows 64 but UDS responses are almost
/// always ≤7 anyway).
pub type ResponseBuf = Mutex<RefCell<[u8; 7]>>;

pub static RESPONSE_BUF: ResponseBuf = Mutex::new(RefCell::new([0; 7]));
pub static RESPONSE_LEN: core::sync::atomic::AtomicU8 =
    core::sync::atomic::AtomicU8::new(0);

/// Write a UDS response into the shared buffer. Also sets
/// `UDS_STATE.response_pending = true` so the canopen task
/// picks it up. Returns the byte count written (clipped to 7).
pub fn store_response(payload: &[u8]) -> usize {
    let len = payload.len().min(7);
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
pub fn load_response() -> ([u8; 7], u8) {
    let mut bytes = [0u8; 7];
    let len = RESPONSE_LEN.load(Ordering::Relaxed);
    critical_section::with(|cs| {
        let buf = RESPONSE_BUF.borrow_ref(cs);
        for i in 0..(len as usize).min(7) {
            bytes[i] = buf[i];
        }
    });
    (bytes, len)
}

/// Convenience: store a standard negative response.
pub fn store_negative(sid: u8, nrc: Nrc) -> usize {
    store_response(&nrc.negative_response(sid))
}
