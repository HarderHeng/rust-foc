//! UDS configuration: service table + DID/Routine tables + callbacks.
//!
//! Phase 5a scope:
//! - 1 service table (SID → handler + gate)
//! - 1 read DID table (currently only 0xF186 ActiveDiagSession)
//! - 0 write DIDs
//! - 0 routines (Phase 5b)
//! - SAL1 LFSR key derivation with a hardcoded mask
//!
//! The actual instance of `UdsConfig` lives in
//! `src/can/uds_config.rs` (top-level under the `can` module).

use super::nrc::Nrc;
use super::state::{Session, SecurityLevel, SrvState};

/// Handler enum: each variant points at the per-SID handler module.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ServiceHandler {
    Session,
    EcuReset,
    ClearDtc,
    ReadDtc,
    ReadDataById,
    WriteDataById,
    CommControl,
    SecurityAccess,
    RoutineStart,
    RequestDownload,
    TransferData,
    TransferExit,
    TesterPresent,
}

#[derive(Copy, Clone, Debug)]
pub struct ServiceEntry {
    pub sid: u8,
    /// Bitmask of allowed sessions: bit 0 = Default, bit 1 = Programming,
    /// bit 2 = Extended. 0 means "no restriction".
    pub session_access: u8,
    /// Minimum required SAL. 0 = no SAL required.
    pub security_level: u8,
    pub handler: ServiceHandler,
}

impl ServiceEntry {
    pub const fn new(
        sid: u8,
        session_access: u8,
        security_level: u8,
        handler: ServiceHandler,
    ) -> Self {
        Self { sid, session_access, security_level, handler }
    }
}

/// ReadDataByIdentifier (0x22) DID entry.
pub struct DidReadEntry {
    pub did: u16,
    pub session_access: u8,
    pub security_level: u8,
    /// Caller writes the response payload to `out` and returns the
    /// number of bytes written. Returning `Err(nrc)` produces a
    /// negative response with the given NRC.
    pub func: fn(out: &mut [u8; 7]) -> Result<usize, Nrc>,
}

/// WriteDataByIdentifier (0x2E) DID entry. Phase 5a has none; declared
/// so the type can be re-used when Phase 5b adds DIDs.
pub struct DidWriteEntry {
    pub did: u16,
    pub session_access: u8,
    pub security_level: u8,
    pub func: fn(data: &[u8]) -> Result<(), Nrc>,
}

/// RoutineControl (0x31) RID entry. Each of the three subfuncs
/// (start / stop / result) has its own table; the same RID can
/// appear in multiple tables with different callbacks.
pub struct RoutineEntry {
    pub rid: u16,
    pub session_access: u8,
    pub security_level: u8,
    /// Input: payload bytes after the RID. Output: response bytes
    /// (length returned via `Ok(n)`). Errors map to negative
    /// responses.
    pub func: fn(req: &[u8], resp: &mut [u8]) -> Result<usize, Nrc>,
}

/// Helper for the empty write-DIDs table — keeps `static`
/// initializers type-inferable without a workaround.
pub fn __empty_write_dids() -> &'static [DidWriteEntry] { &[] }

/// UDS configuration. One instance per program; `static`.
pub struct UdsConfig {
    pub services: &'static [ServiceEntry],

    pub read_dids: &'static [DidReadEntry],
    #[allow(dead_code)] // wired in Phase 5b
    pub write_dids: &'static [DidWriteEntry],

    /// RoutineControl (0x31) tables, one per subfunc.
    pub routines_start: &'static [RoutineEntry],
    pub routines_stop: &'static [RoutineEntry],
    pub routines_result: &'static [RoutineEntry],

    /// Pending queue (Phase 5c). 4 slots covers TransferData +
    /// TransferExit + 2 waiting. Stored as `&'static mut`
    /// because `dispatch` and `tick` need to mutate the slots
    /// and the `Option<PendingJob>` payload contains a
    /// `Box<dyn FnMut>` which isn't `Sync`.
    pub pending_queue: &'static mut [Option<crate::uds::pending::PendingJob>],

    /// P2 server timer (ms). When a request stays in
    /// `SrvState::Pending` for longer than this, the
    /// dispatcher pushes a 0x78 response. ISO 14229 standard:
    /// 50 ms.
    pub p2_server_ms: u32,

    /// Per-SAL LFSR mask. Index 0/1/2 = SAL1/2/3.
    pub key_masks: [u32; 3],

    /// Session-change callbacks. Phase 5b registers noop stubs;
    /// real logging / lock-out can be added by editing
    /// `src/can/uds_config.rs`.
    pub on_default_session_enter: Option<fn()>,
    pub on_programming_session_enter: Option<fn()>,
    pub on_extended_session_enter: Option<fn()>,
}

/// Helper: build the bitmask for a given session.
pub const fn session_bit(s: Session) -> u8 { 1 << (s.as_u8() - 1) }

/// Check whether `state.session` is in the `access` bitmask.
pub fn session_allowed(state_session: Session, access: u8) -> bool {
    if access == 0 { return true; }  // 0 = allpass
    (access & session_bit(state_session)) != 0
}

/// Check whether `state.security >= required`.
pub fn security_allowed(state_security: SecurityLevel, required: u8) -> bool {
    (state_security as u8) >= required
}

/// Helper: unused but documents the relationship between the
/// state machine and the dispatcher return type.
#[allow(dead_code)]
fn _state_unused(_s: SrvState) {}
