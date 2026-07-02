//! UDS (ISO 14229) over CANopen SDO — Phase 3 minimal.
//!
//! Architecture: all UDS services are reached through the vendor
//! SDO object `0x2F00.0` (the "UDS gateway"). Master does:
//!
//!   1. SDO Initiate Download to `0x2F00.0` with payload
//!      `[SID, params...]` (the UDS request).
//!   2. Read back via SDO Initiate Upload from `0x2F00.0` —
//!      the response is the UDS positive / negative response
//!      that the gateway stored while dispatching the request.
//!
//! Response payloads are bounded to 4 bytes so they fit SDO
//! expedited transfer. The 0x27 SecurityAccess seed is 2 bytes
//! (truncated from the spec's 4-byte placeholder seed) for the
//! same reason. Phase 3+ (or v2 of this module) would grow
//! segmented SDO transfer for the longer services.
//!
//! ## Services implemented (v1)
//!
//! | SID  | Name                  | Notes |
//! |------|-----------------------|-------|
//! | 0x10 | DiagnosticSession     | Default(0x01), Programming(0x02) |
//! | 0x11 | ECUReset              | HardReset(0x01); triggers NVIC reset after 10 ms |
//! | 0x14 | ClearDiagnosticInfo   | ack |
//! | 0x19 | ReadDTCInformation    | subfunc 0x02: 0 DTCs |
//! | 0x22 | ReadDataByIdentifier  | 0xF186 (ActiveDiagSession, 1 byte) only |
//! | 0x2E | WriteDataByIdentifier | ack (no DIDs writable in v1) |
//! | 0x27 | SecurityAccess        | seed=0xA5A5, key=0xA5B7 (2-byte truncated) |
//! | 0x3E | TesterPresent         | subfunc 0x00, 0x80 (suppress positive) |
//!
//! ## Negative response codes (NRCs)
//!
//! 0x12 SubFunctionNotSupported, 0x13 IncorrectMessageLength,
//! 0x14 ResponseTooLong, 0x22 ConditionsNotCorrect,
//! 0x31 RequestOutOfRange, 0x33 SecurityAccessDenied,
//! 0x72 GeneralProgrammingFailure.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use defmt::info;

/// Diagnostic session types. Values match the UDS spec.
const SESSION_DEFAULT: u8 = 0x01;
#[allow(dead_code)] // Phase 4 OTA enters this session
const SESSION_PROGRAMMING: u8 = 0x02;

/// Current UDS diagnostic session. Read by 0xF186 (ActiveDiagSession),
/// written by 0x10 (DiagnosticSessionControl).
static SESSION: AtomicU8 = AtomicU8::new(SESSION_DEFAULT);

/// SecurityAccess state. `0` = Locked (no seed sent yet), `1`
/// = Unlocked (valid key received since last reset).
static SECURITY: AtomicU8 = AtomicU8::new(0);

/// "Reset requested" flag. Set by 0x11 HardReset, polled by
/// the canopen task. We use a flag + polling rather than
/// firing the reset from inside the UDS handler because the
/// caller (SDO dispatch) is still holding the response buffer
/// when the request finishes — resetting there would leave the
/// master with an unsent response.
static RESET_REQUESTED: AtomicBool = AtomicBool::new(false);

/// Storage for the last UDS response. 7 bytes max (SDO
/// expedited payload ceiling for a 1-byte SDO response, or
/// 7 data bytes + 1 cmd if we used the non-expedited encoding).
/// `cortex-m`'s critical section is the cheapest synchronization
/// here — single-threaded executor, never held across an await.
///
/// Wrapped in `RefCell` because `critical-section`'s `Mutex` only
/// exposes `borrow_ref_mut` (which returns a `RefMut<T>`) on
/// `Mutex<RefCell<T>>`. The plain `Mutex<T>::borrow` returns a
/// shared `&T`, so we can't assign through it.
static LAST_RESPONSE: critical_section::Mutex<RefCell<[u8; 7]>> =
    critical_section::Mutex::new(RefCell::new([0; 7]));
static LAST_RESPONSE_LEN: core::sync::atomic::AtomicU8 =
    core::sync::atomic::AtomicU8::new(0);

/// Process a UDS request and store the response.
///
/// `request` is the UDS payload (`[SID, ...]`). Returns the
/// response length (always 1..=4 in v1; services that would
/// need a longer reply return NRC 0x14 ResponseTooLong).
///
/// The response is also stashed in `LAST_RESPONSE` for the
/// SDO read on `0x2F00.0` to fetch.
pub fn dispatch(request: &[u8]) -> usize {
    if request.is_empty() {
        return store_negative(0, NRC::IncorrectMessageLength);
    }
    let sid = request[0];
    let payload = &request[1..];
    match sid {
        SID_DIAGNOSTIC_SESSION_CONTROL => handle_session_control(payload),
        SID_ECU_RESET => handle_ecu_reset(payload),
        SID_CLEAR_DIAGNOSTIC_INFORMATION => handle_clear_dtc(payload),
        SID_READ_DTC_INFORMATION => handle_read_dtc(payload),
        SID_READ_DATA_BY_IDENTIFIER => handle_read_did(payload),
        SID_WRITE_DATA_BY_IDENTIFIER => handle_write_did(payload),
        SID_SECURITY_ACCESS => handle_security_access(payload),
        SID_TESTER_PRESENT => handle_tester_present(payload),
        // Phase 4: OTA via UDS TransferData. All three SIDs
        // require the controller to be in ProgrammingSession
        // (0x02); otherwise we return NRC 0x22
        // ConditionsNotCorrect.
        SID_REQUEST_DOWNLOAD => {
            if SESSION.load(Ordering::Relaxed) != 0x02 {
                return store_negative(sid, NRC::ConditionsNotCorrect);
            }
            super::ota::handle_request_download(payload)
        }
        SID_TRANSFER_DATA => {
            if SESSION.load(Ordering::Relaxed) != 0x02 {
                return store_negative(sid, NRC::ConditionsNotCorrect);
            }
            super::ota::handle_transfer_data(payload)
        }
        SID_REQUEST_TRANSFER_EXIT => {
            if SESSION.load(Ordering::Relaxed) != 0x02 {
                return store_negative(sid, NRC::ConditionsNotCorrect);
            }
            super::ota::handle_transfer_exit(payload)
        }
        _ => store_negative(sid, NRC::SubFunctionNotSupported),
    }
}

/// Read back the last response, in `OdValue` form. The
/// canopen OD layer calls this for SDO reads of `0x2F00.0`.
pub fn load_response() -> super::od::OdValue {
    let len = LAST_RESPONSE_LEN.load(Ordering::Relaxed) as usize;
    let mut bytes = [0u8; 8];
    let mut bits: u32 = 0;
    let n = len.min(4);
    critical_section::with(|cs| {
        let buf = LAST_RESPONSE.borrow_ref(cs);
        for i in 0..n {
            bytes[i] = buf[i];
            bits |= (buf[i] as u32) << (8 * i);
        }
    });
    super::od::OdValue { size: n as u8, bits }
}

/// True iff the canopen task should perform an NVIC system
/// reset on the next tick. Cleared once observed (so the task
/// can keep polling without re-entering the reset path).
pub fn take_reset_request() -> bool {
    RESET_REQUESTED.swap(false, Ordering::Relaxed)
}

// ---- Service IDs (UDS spec) -------------------------------------------

const SID_DIAGNOSTIC_SESSION_CONTROL: u8 = 0x10;
const SID_ECU_RESET: u8 = 0x11;
const SID_CLEAR_DIAGNOSTIC_INFORMATION: u8 = 0x14;
const SID_READ_DTC_INFORMATION: u8 = 0x19;
const SID_READ_DATA_BY_IDENTIFIER: u8 = 0x22;
const SID_WRITE_DATA_BY_IDENTIFIER: u8 = 0x2E;
const SID_SECURITY_ACCESS: u8 = 0x27;
const SID_TESTER_PRESENT: u8 = 0x3E;

// ---- Data Identifiers (DIDs) ----------------------------------------

/// 0xF186 = ActiveDiagnosticSession. Read-only 1-byte value.
const DID_ACTIVE_DIAG_SESSION: [u8; 2] = [0x86, 0xF1];

// ---- Re-export OTA service IDs (Phase 4) -----------------------------

pub use super::ota::SID_REQUEST_DOWNLOAD;
pub use super::ota::SID_TRANSFER_DATA;
pub use super::ota::SID_REQUEST_TRANSFER_EXIT;

// ---- NRCs (UDS spec) -------------------------------------------------

#[allow(dead_code)]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum NRC {
    SubFunctionNotSupported       = 0x12,
    IncorrectMessageLength        = 0x13,
    ResponseTooLong               = 0x14,
    ConditionsNotCorrect          = 0x22,
    RequestOutOfRange             = 0x31,
    SecurityAccessDenied          = 0x33,
    GeneralProgrammingFailure     = 0x72,
}

// ---- Per-service handlers -------------------------------------------

fn handle_session_control(payload: &[u8]) -> usize {
    // [0x10, subfunc] → [0x50, subfunc]
    if payload.len() != 1 {
        return store_negative(
            SID_DIAGNOSTIC_SESSION_CONTROL,
            NRC::IncorrectMessageLength,
        );
    }
    let subfunc = payload[0];
    match subfunc {
        0x01 | 0x02 => {
            SESSION.store(subfunc, Ordering::Relaxed);
            // Switching session also locks SecurityAccess — the
            // seed is invalidated by definition.
            SECURITY.store(0, Ordering::Relaxed);
            info!("UDS: session → 0x{:02x}", subfunc);
            store_positive(&[SID_DIAGNOSTIC_SESSION_CONTROL + 0x40, subfunc])
        }
        _ => store_negative(
            SID_DIAGNOSTIC_SESSION_CONTROL,
            NRC::SubFunctionNotSupported,
        ),
    }
}

fn handle_ecu_reset(payload: &[u8]) -> usize {
    // [0x11, subfunc] → [0x51, subfunc]; then trigger reset
    if payload.len() != 1 {
        return store_negative(SID_ECU_RESET, NRC::IncorrectMessageLength);
    }
    let subfunc = payload[0];
    match subfunc {
        0x01 => {
            // HardReset: stash the positive response, mark the
            // flag, and let the canopen task fire the NVIC reset
            // on the next tick.
            RESET_REQUESTED.store(true, Ordering::Relaxed);
            info!("UDS: ECUReset(Hard) requested");
            store_positive(&[SID_ECU_RESET + 0x40, subfunc])
        }
        _ => store_negative(SID_ECU_RESET, NRC::SubFunctionNotSupported),
    }
}

fn handle_clear_dtc(payload: &[u8]) -> usize {
    // [0x14, group(3)] → [0x54]
    if payload.len() != 3 {
        return store_negative(
            SID_CLEAR_DIAGNOSTIC_INFORMATION,
            NRC::IncorrectMessageLength,
        );
    }
    // We have no DTCs to clear (Phase 3 v1). Always succeed.
    store_positive(&[SID_CLEAR_DIAGNOSTIC_INFORMATION + 0x40])
}

fn handle_read_dtc(payload: &[u8]) -> usize {
    // [0x19, subfunc, status_mask] → [0x59, subfunc, avail, fmt]
    // where avail = 0 (no DTCs match the mask), fmt = 0x00
    // (ISO15031-6 format). Phase 3 v1 only handles subfunc 0x02
    // (reportDTCByStatusMask); other subfuncs return
    // SubFunctionNotSupported.
    if payload.len() < 2 {
        return store_negative(
            SID_READ_DTC_INFORMATION,
            NRC::IncorrectMessageLength,
        );
    }
    let subfunc = payload[0];
    if subfunc != 0x02 {
        return store_negative(
            SID_READ_DTC_INFORMATION,
            NRC::SubFunctionNotSupported,
        );
    }
    info!("UDS: ReadDTC(subfunc=0x02) → 0 DTCs");
    store_positive(&[
        SID_READ_DTC_INFORMATION + 0x40,
        0x02, // subfunc echo
        0x00, // availability mask (no DTCs)
        0x00, // format = ISO15031-6
    ])
}

fn handle_read_did(payload: &[u8]) -> usize {
    // [0x22, did_lo, did_hi] → [0x62, did_lo, did_hi, ...data]
    if payload.len() != 2 {
        return store_negative(
            SID_READ_DATA_BY_IDENTIFIER,
            NRC::IncorrectMessageLength,
        );
    }
    let did = [payload[0], payload[1]];
    if did == DID_ACTIVE_DIAG_SESSION {
        let session = SESSION.load(Ordering::Relaxed);
        store_positive(&[
            SID_READ_DATA_BY_IDENTIFIER + 0x40,
            did[0], did[1],
            session,
        ])
    } else {
        store_negative(SID_READ_DATA_BY_IDENTIFIER, NRC::RequestOutOfRange)
    }
}

fn handle_write_did(payload: &[u8]) -> usize {
    // [0x2E, did_lo, did_hi, data...] → [0x6E, did_lo, did_hi]
    if payload.len() < 2 {
        return store_negative(
            SID_WRITE_DATA_BY_IDENTIFIER,
            NRC::IncorrectMessageLength,
        );
    }
    // v1: no DIDs are writable. Always return RequestOutOfRange.
    let did = [payload[0], payload[1]];
    info!("UDS: WriteDID(0x{:02x}{:02x}) rejected (no writable DIDs in v1)", did[0], did[1]);
    store_negative(SID_WRITE_DATA_BY_IDENTIFIER, NRC::RequestOutOfRange)
}

fn handle_security_access(payload: &[u8]) -> usize {
    // [0x27, subfunc, ...] → varies
    if payload.is_empty() {
        return store_negative(SID_SECURITY_ACCESS, NRC::IncorrectMessageLength);
    }
    let subfunc = payload[0];
    match subfunc {
        // requestSeed: respond with a 2-byte seed (the spec
        // example seed is 4 bytes; we truncate to fit SDO
        // expedited 4-byte response). Subfunc 0x01 is "request
        // seed level 1".
        0x01 => {
            if SECURITY.load(Ordering::Relaxed) != 0 {
                // Already unlocked: spec says "requestSeed when
                // already unlocked" is an error.
                return store_negative(
                    SID_SECURITY_ACCESS,
                    NRC::SecurityAccessDenied,
                );
            }
            // Seed = 0xA5A5 (2 bytes; the high 2 bytes of the
            // spec's 4-byte seed are dropped to fit SDO).
            store_positive(&[
                SID_SECURITY_ACCESS + 0x40,
                0x01,
                0xA5, 0xA5,
            ])
        }
        // sendKey: payload = [0x02, key_lo, key_hi, ...]
        0x02 => {
            if payload.len() != 3 {
                return store_negative(
                    SID_SECURITY_ACCESS,
                    NRC::IncorrectMessageLength,
                );
            }
            // Expected key: seed + 0x12 = 0xA5A5 + 0x12 = 0xA5B7.
            let key = u16::from_le_bytes([payload[1], payload[2]]);
            if key == 0xA5B7 {
                SECURITY.store(1, Ordering::Relaxed);
                info!("UDS: SecurityAccess unlocked");
                store_positive(&[SID_SECURITY_ACCESS + 0x40, 0x02])
            } else {
                info!("UDS: SecurityAccess wrong key 0x{:04x}", key);
                store_negative(
                    SID_SECURITY_ACCESS,
                    NRC::SecurityAccessDenied,
                )
            }
        }
        _ => store_negative(SID_SECURITY_ACCESS, NRC::SubFunctionNotSupported),
    }
}

fn handle_tester_present(payload: &[u8]) -> usize {
    // [0x3E, subfunc] → [0x7E, subfunc]
    if payload.len() != 1 {
        return store_negative(
            SID_TESTER_PRESENT,
            NRC::IncorrectMessageLength,
        );
    }
    let subfunc = payload[0];
    if subfunc & 0x7F == 0x00 {
        if subfunc & 0x80 != 0 {
            // Suppress positive response (bit 7 set). Per the
            // spec, the server stays silent in this case. We
            // store an empty response and return length 0; the
            // SDO layer will return 0x60 (download success)
            // and the master will see no payload on the read.
            store_positive(&[])
        } else {
            store_positive(&[SID_TESTER_PRESENT + 0x40, 0x00])
        }
    } else {
        store_negative(SID_TESTER_PRESENT, NRC::SubFunctionNotSupported)
    }
}

// ---- Response storage -----------------------------------------------

/// Store a positive response (`[SID+0x40, ...]`) and return its
/// length. The response is also left in `LAST_RESPONSE` for the
/// SDO read on `0x2F00.0` to fetch.
fn store_positive(payload: &[u8]) -> usize {
    let len = payload.len();
    if len > 4 {
        // Shouldn't happen for v1, but defensively: any service
        // that would produce a >4-byte response is misdesigned.
        return store_negative(payload[0].saturating_sub(0x40), NRC::ResponseTooLong);
    }
    let mut buf = [0u8; 7];
    buf[..len].copy_from_slice(payload);
    critical_section::with(|cs| {
        *LAST_RESPONSE.borrow_ref_mut(cs) = buf;
    });
    LAST_RESPONSE_LEN.store(len as u8, Ordering::Relaxed);
    len
}

/// Store a negative response (`[0x7F, SID, NRC]`) and return 3.
fn store_negative(sid: u8, nrc: NRC) -> usize {
    store_external_response(&[0x7F, sid, nrc as u8])
}

/// External entry point for storing a UDS response without
/// going through the local `store_positive` / `store_negative`
/// helpers. Used by `super::ota` to push a response that
/// originates from a different code path (still inside the
/// single-threaded canopen task, so no race).
pub fn store_external_response(payload: &[u8]) -> usize {
    let len = payload.len();
    let mut buf = [0u8; 7];
    buf[..len.min(7)].copy_from_slice(&payload[..len.min(7)]);
    critical_section::with(|cs| {
        *LAST_RESPONSE.borrow_ref_mut(cs) = buf;
    });
    LAST_RESPONSE_LEN.store(len.min(7) as u8, Ordering::Relaxed);
    len.min(7)
}
