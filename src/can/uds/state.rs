//! UDS runtime state. RAM-resident (`#[link_section = ".data"]`)
//! so the OTA write path stays off the flash-resident call chain
//! (see module-level docs in `super`).

use core::cell::RefCell;
use core::sync::atomic::Ordering;
use critical_section::Mutex;

use super::nrc::Nrc;

/// Diagnostic session types. Values match UDS spec.
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

/// Security Access Level. Locked = no key verified yet.
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
            1 => Self::Sal1,
            2 => Self::Sal2,
            3 => Self::Sal3,
            _ => Self::Locked,
        }
    }
    pub const fn as_u8(self) -> u8 { self as u8 }
}

/// UDS dispatcher state. `Idle` accepts new requests; `Parsing` is
/// inside the dispatch (sync only); `Pending` means a continuation
/// function is queued in the pending queue and `tick` will run it.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SrvState {
    Idle,
    Parsing,
    Pending,
}

/// UDS engine runtime state. RAM-resident.
///
/// All fields are plain data (no `Cell` / `Atomic`) because the
/// canopen task is the only owner and the executor is
/// single-threaded. Sync handlers run to completion before any
/// other UDS code touches `state`.
#[allow(dead_code)] // several fields are wired in Phase 5c/5d
#[link_section = ".data"]
pub struct UdsState {
    pub session: Session,
    pub security: SecurityLevel,
    /// Per-SAL seed tracking. Index 0/1/2 = SAL1/2/3.
    /// `seed_sent[i] = true` means a RequestSeed for SAL `i+1` was
    /// issued and the next SendKey for that SAL must match.
    pub seed_sent: [bool; 3],
    /// Most recent seed per SAL (kept across the request → key
    /// sequence).
    pub current_seed: [u32; 3],

    pub state: SrvState,
    /// The in-flight UDS request (set at the start of `dispatch`).
    /// `request_len` bytes are valid; the rest is undefined.
    /// Pending-queue closures read from here to recover their
    /// input — no env capture needed.
    pub request_buf: [u8; 64],
    pub request_len: usize,
    pub response_len: usize,
    /// `response_pending = true` means `response_buf[0..response_len]`
    /// holds a response to send. `take_response` reads + clears.
    pub response_pending: bool,
    pub request_tick_ms: u32,

    /// OTA state. Mirrored from `super::super::ota` static
    /// `AtomicU8` to keep dispatch in one place; OTA still owns
    /// the authoritative state for the write pointer, CRC, etc.
    pub download_active: bool,
    pub transfer_sn: u8,

    /// CommunicationControl state. `tx_disabled = true` ⇒ 0x28
    /// disableNormalCommunication is active and dispatcher
    /// rejects new requests with 0x22 (except 0x28 0x00 enable
    /// which is the unlock path).
    pub tx_disabled: bool,
    pub rx_disabled: bool,
}

impl UdsState {
    pub const fn zeroed() -> Self {
        Self {
            session: Session::Default,
            security: SecurityLevel::Locked,
            seed_sent: [false; 3],
            current_seed: [0; 3],
            state: SrvState::Idle,
            request_buf: [0; 64],
            request_len: 0,
            response_len: 0,
            response_pending: false,
            request_tick_ms: 0,
            download_active: false,
            transfer_sn: 0,
            tx_disabled: false,
            rx_disabled: false,
        }
    }
}

/// UDS request / response buffers, RAM-resident. Both are
/// `pub` so the dispatcher in `mod.rs` can mutate them; the
/// `Mutex<RefCell>` indirection lets multiple call sites (SDO
/// write trigger, SDO read fetch, canopen_task tick) coexist
/// without aliasing.
///
/// Phase 5c keeps the existing `RESPONSE_BUF` / `RESPONSE_LEN`
/// pair as the wire path. `take_response` in `pending.rs` reads
/// from the same buffer. A future phase may move the response
/// buffer into `UdsConfig::response_buf` to avoid the static.
#[allow(dead_code)]
pub static REQUEST_BUF: critical_section::Mutex<RefCell<[u8; 64]>> =
    critical_section::Mutex::new(RefCell::new([0; 64]));

/// Storage for the last UDS response. Up to 7 bytes (SDO upload
/// ceiling). Single-threaded owner, but a `Mutex<RefCell>` keeps
/// the API consistent with the rest of the CAN stack and lets
/// `sdo::dispatch` read it from a different task context if
/// refactor moves the SDO read path to a future OTA-only task.
pub type ResponseBuf = Mutex<RefCell<[u8; 7]>>;

pub static RESPONSE_BUF: ResponseBuf = Mutex::new(RefCell::new([0; 7]));
pub static RESPONSE_LEN: core::sync::atomic::AtomicU8 =
    core::sync::atomic::AtomicU8::new(0);

/// Write a UDS response into the shared buffer. `payload[0]` should
/// be the SID (positive or negative); for negatives the second
/// byte is the original SID. Returns the byte count written
/// (clipped to 7 — anything longer is a programming bug).
///
/// Also sets `UDS_STATE.response_pending = true` so
/// `pending::take_response` will find it. (Phase 5c.)
pub fn store_response(payload: &[u8]) -> usize {
    let len = payload.len().min(7);
    critical_section::with(|cs| {
        let buf = &mut *RESPONSE_BUF.borrow_ref_mut(cs);
        buf[..len].copy_from_slice(&payload[..len]);
    });
    RESPONSE_LEN.store(len as u8, Ordering::Relaxed);
    // Safety: single-threaded executor; UDS_STATE is a `static`
    // (not `static mut`) and the only writes go through this
    // function and `pending::take_response` (also unsafe).
    unsafe {
        let p = &raw const crate::can::uds::UDS_STATE
               as *mut crate::can::uds::UdsState;
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

/// Read the last UDS request (used by 0x78 frame to look up the
/// in-flight SID). Returns `(bytes, len)`.
pub fn load_request() -> ([u8; 64], u8) {
    let mut bytes = [0u8; 64];
    let len = REQUEST_LEN.load(Ordering::Relaxed);
    critical_section::with(|cs| {
        let buf = REQUEST_BUF.borrow_ref(cs);
        for i in 0..(len as usize).min(64) {
            bytes[i] = buf[i];
        }
    });
    (bytes, len)
}

pub static REQUEST_LEN: core::sync::atomic::AtomicU8 =
    core::sync::atomic::AtomicU8::new(0);

/// Convenience: store a standard negative response.
pub fn store_negative(sid: u8, nrc: Nrc) -> usize {
    store_response(&nrc.negative_response(sid))
}

/// Read the current session byte for `0xF186 ActiveDiagSession`.
/// Returns the subfunc value of the active session (0x01/0x02/0x03).
pub fn load_response_session_byte() -> u8 {
    // Forward reference: `super::UDS_STATE` is defined in
    // `super::mod` (the parent module's mod.rs). The body is
    // type-checked after the whole module is parsed, so the
    // reference is valid here. `static`s are `Sync`, so the
    // shared reference below is sound.
    crate::can::uds::UDS_STATE.session.as_u8()
}
