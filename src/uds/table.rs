//! UDS service table schema + the `UdsConfig` impl block that
//! contains every built-in SID's dispatch logic.
//!
//! ## Architecture
//!
//! The MiniUds pattern: ALL `dispatch_0xNN` methods live on
//! `UdsConfig`. Adding a new SID = add a method here + a
//! `ServiceEntry` in `static_config.rs`. Adding a new
//! callback (DID reader, routine handler) = add an entry in
//! `static_config.rs`. Nothing else moves.
//!
//! Each `dispatch_0xNN` is short: parse the wire format, find
//! the table entry, gate on session+security, call the
//! callback. State mutations are explicit (e.g. 0x10 advances
//! `state.session`, 0x27 advances `state.security`, 0x28 sets
//! `state.tx_disabled`).

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use defmt::info;

use super::crypto::{generate_key, AesBlock};
use super::pending::{push_pending, PendingFn, UdsContext};
use super::state::{store_response, UdsState};
use super::types::{Nrc, SecurityLevel, Session};

// ============================================================================
// Schema types — the *shape* of the tables
// ============================================================================

/// Tag for the built-in SID dispatchers. Used by `mod.rs` to
/// route a request to the right `dispatch_0xNN` method. There
/// is one variant per built-in SID.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ServiceHandler {
    Session,        // 0x10
    EcuReset,       // 0x11
    ClearDtc,       // 0x14
    ReadDtc,        // 0x19
    ReadDataById,   // 0x22
    WriteDataById,  // 0x2E
    CommControl,    // 0x28
    SecurityAccess, // 0x27
    #[allow(dead_code)]
    RoutineStart,   // 0x31 (start)
    #[allow(dead_code)]
    RoutineStop,    // 0x31 (stop)
    #[allow(dead_code)]
    RoutineResult,  // 0x31 (result)
    RequestDownload,// 0x34
    TransferData,   // 0x36
    TransferExit,   // 0x37
    TesterPresent,  // 0x3E
}

/// One row in the service table. The dispatcher in
/// `super::mod` looks up SID, gates on session+security, and
/// then calls the matching `ServiceHandler::dispatch_*` method
/// (which lives in this file as `UdsConfig::dispatch_0xNN`).
#[derive(Copy, Clone, Debug)]
pub struct ServiceEntry {
    pub sid: u8,
    /// Bitmask of allowed sessions: bit 0 = Default, bit 1 =
    /// Programming, bit 2 = Extended. 0 = allpass.
    pub session_access: u8,
    /// Minimum required SAL. 0 = no SAL required.
    pub security_level: u8,
    pub handler: ServiceHandler,
}

impl ServiceEntry {
    pub const fn new(
        sid: u8, session_access: u8, security_level: u8, handler: ServiceHandler,
    ) -> Self {
        Self { sid, session_access, security_level, handler }
    }
}

/// ReadDataByIdentifier (0x22) DID entry. The `func` writes
/// the response payload to `out` and returns the byte count.
pub struct DidReadEntry {
    pub did: u16,
    pub session_access: u8,
    pub security_level: u8,
    pub func: fn(out: &mut [u8; 64]) -> Result<usize, Nrc>,
}

/// WriteDataByIdentifier (0x2E) DID entry. Empty by default.
pub struct DidWriteEntry {
    pub did: u16,
    pub session_access: u8,
    pub security_level: u8,
    pub func: fn(data: &[u8]) -> Result<(), Nrc>,
}

/// RoutineControl (0x31) RID entry. Each of the three
/// subfuncs (start/stop/result) has its own table.
pub struct RoutineEntry {
    pub rid: u16,
    pub session_access: u8,
    pub security_level: u8,
    pub func: fn(req: &[u8], resp: &mut [u8]) -> Result<usize, Nrc>,
}

/// Helper: build the bitmask for a given session.
pub const fn session_bit(s: Session) -> u8 { 1 << (s.as_u8() - 1) }

/// Check whether `state.session` is in the `access` bitmask.
pub fn session_allowed(state_session: Session, access: u8) -> bool {
    if access == 0 { return true; }
    (access & session_bit(state_session)) != 0
}

/// Check whether `state.security >= required`.
pub fn security_allowed(state_security: SecurityLevel, required: u8) -> bool {
    (state_security as u8) >= required
}

// ============================================================================
// The big UdsConfig struct
// ============================================================================

/// The single source of truth for the UDS protocol stack.
/// All tables + all callbacks + all built-in dispatch logic.
pub struct UdsConfig {
    pub services: &'static [ServiceEntry],

    pub read_dids: &'static [DidReadEntry],
    #[allow(dead_code)]
    pub write_dids: &'static [DidWriteEntry],

    pub routines_start: &'static [RoutineEntry],
    pub routines_stop: &'static [RoutineEntry],
    pub routines_result: &'static [RoutineEntry],

    /// Pending queue (Phase 5c). 4 slots covers TransferData +
    /// TransferExit + 2 waiting. `&'static mut` because
    /// `dispatch` and `tick` need to mutate the slots and
    /// `Option<PendingJob>` contains a closure which isn't Sync.
    pub pending_queue: &'static mut [Option<super::pending::PendingJob>],

    /// P2 server timer (ms). When a request stays in
    /// `SrvState::Pending` for longer than this, the dispatcher
    /// pushes a 0x78 response. ISO 14229 standard: 50 ms.
    pub p2_server_ms: u32,

    /// Per-SAL AES-128 key material. Index 0/1/2 = SAL1/2/3.
    /// Writably at runtime via DID 0xF180.
    pub key_masks: [AesBlock; 3],

    /// Session-change callbacks. The runtime registers these
    /// in `static_config.rs`; `fn` pointer (no capture) is fine
    /// for no-alloc no_std.
    pub on_default_session_enter: Option<fn()>,
    pub on_programming_session_enter: Option<fn()>,
    pub on_extended_session_enter: Option<fn()>,

    /// Seed-generation callback for SecurityAccess (0x27).
    ///
    /// `None` — use the built-in SysTick jitter (good enough
    /// for most industrial UDS deployments).
    ///
    /// `Some(fn)` — call `f()` every RequestSeed. The function
    /// must return 16 cryptographically random bytes. Example
    /// using a platform register (e.g., hardware RNG, SysTick
    /// jitter, or TRNG):
    ///
    /// ```ignore
    /// fn my_seed() -> AesBlock {
    ///     let mut seed = [0u8; 16];
    ///     // Fill `seed` from platform entropy source ...
    ///     AesBlock(seed)
    /// }
    /// ```
    pub seed_fn: Option<fn() -> AesBlock>,

    /// Key-derivation callback for SecurityAccess (0x27).
    ///
    /// `None` — use the built-in AES-128-ECB.
    ///
    /// `Some(fn)` — called every SendKey with `(seed, key_material)`,
    /// must return the derived 16-byte key. Enables custom algorithms
    /// (proprietary cipher, custom LFSR, XOR, etc.) without touching
    /// the UDS core.
    pub key_fn: Option<fn(&AesBlock, &AesBlock) -> AesBlock>,

    /// OTA callbacks for 0x34/0x36/0x37 (pending queue).
    ///
    /// `None` — the corresponding SID returns 0x22 (not accepted).
    ///
    /// `Some(fn)` — called from the pending queue closure. The
    /// function reads request bytes from `ctx.state.request_buf`,
    /// performs the flash operation, and sets `ctx.complete = true`
    /// when done.
    ///
    /// Separate from `on_*_session_enter` / `seed_fn` / `key_fn`
    /// because OTA is an optional feature — not every UDS user
    /// has an OTA module or uses flash-based download.
    pub request_download_fn: Option<fn(&mut UdsContext)>,
    pub transfer_data_fn: Option<fn(&mut UdsContext)>,
    pub transfer_exit_fn: Option<fn(&mut UdsContext)>,
}

// ============================================================================
// Reset request flag (consumed by canopen_task)
// ============================================================================

/// Reset requested by 0x11. Polled by the canopen task after
/// the response has gone out.
pub static RESET_REQUESTED: AtomicBool = AtomicBool::new(false);
/// Subfunc of the last armed reset (1 = Hard, 3 = Soft).
pub static RESET_SUBFUNC: AtomicU8 = AtomicU8::new(0);

pub fn take_reset_request() -> bool {
    RESET_REQUESTED.swap(false, Ordering::Relaxed)
}

/// `Some(1)` Hard, `Some(3)` Soft. `None` if no reset pending.
pub fn take_reset_subfunc() -> Option<u8> {
    let s = RESET_SUBFUNC.swap(0, Ordering::Relaxed);
    if s == 0 { None } else { Some(s) }
}

// ============================================================================
// Built-in SID dispatchers
// ============================================================================

impl UdsConfig {
    // -- 0x10 DiagnosticSessionControl --------------------------------

    pub fn dispatch_0x10(&self, state: &mut UdsState, req: &[u8]) {
        // [0x10, subfunc] → [0x50, subfunc]
        if req.len() != 2 {
            store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
                .negative_response(0x10));
            return;
        }
        let subfunc = req[1];
        let new_session = match Session::from_u8(subfunc) {
            Some(s) => s,
            None => {
                store_response(&Nrc::SubFunctionNotSupported.negative_response(0x10));
                return;
            }
        };
        // Programming/Extended both require SAL1 unlocked.
        if matches!(new_session, Session::Programming | Session::Extended)
            && state.security < SecurityLevel::Sal1
        {
            store_response(&Nrc::SecurityAccessDenied.negative_response(0x10));
            return;
        }
        state.session = new_session;
        // ISO 14229: session change invalidates security.
        state.security = SecurityLevel::Locked;
        state.seed_sent = false;

        // Fire session-enter callback.
        let cb = match new_session {
            Session::Default => self.on_default_session_enter,
            Session::Programming => self.on_programming_session_enter,
            Session::Extended => self.on_extended_session_enter,
        };
        if let Some(cb) = cb { cb(); }

        info!("UDS: session → 0x{:02x}", subfunc);
        store_response(&[0x50, subfunc]);
    }

    // -- 0x11 ECUReset -----------------------------------------------

    pub fn dispatch_0x11(&self, _state: &mut UdsState, req: &[u8]) {
        // [0x11, subfunc] → [0x51, subfunc]; subfunc 0x01=Hard, 0x03=Soft.
        if req.len() != 2 {
            store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
                .negative_response(0x11));
            return;
        }
        let subfunc = req[1];
        match subfunc {
            0x01 | 0x03 => {
                RESET_REQUESTED.store(true, Ordering::Relaxed);
                RESET_SUBFUNC.store(subfunc, Ordering::Relaxed);
                info!("UDS: ECUReset({}) requested",
                      if subfunc == 0x01 { "Hard" } else { "Soft" });
                store_response(&[0x51, subfunc]);
            }
            _ => {
                store_response(&Nrc::SubFunctionNotSupported.negative_response(0x11));
            }
        }
    }

    // -- 0x14 ClearDiagnosticInformation ------------------------------

    pub fn dispatch_0x14(&self, _state: &mut UdsState, req: &[u8]) {
        // [0x14, group(3)] → [0x54]. No DTCs in v1.
        if req.len() != 4 {
            store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
                .negative_response(0x14));
            return;
        }
        store_response(&[0x54]);
    }

    // -- 0x19 ReadDTCInformation -------------------------------------

    pub fn dispatch_0x19(&self, _state: &mut UdsState, req: &[u8]) {
        // [0x19, subfunc, status_mask] → [0x59, subfunc, avail, fmt]
        if req.len() < 3 {
            store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
                .negative_response(0x19));
            return;
        }
        let subfunc = req[1];
        if subfunc != 0x02 {
            store_response(&Nrc::SubFunctionNotSupported.negative_response(0x19));
            return;
        }
        info!("UDS: ReadDTC(subfunc=0x02) → 0 DTCs");
        store_response(&[0x59, 0x02, 0x00, 0x00]);
    }

    // -- 0x22 ReadDataByIdentifier -----------------------------------

    pub fn dispatch_0x22(&self, state: &mut UdsState, req: &[u8]) {
        // [0x22, did_lo, did_hi] → [0x62, did_lo, did_hi, ...data]
        if req.len() != 3 {
            store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
                .negative_response(0x22));
            return;
        }
        let did = u16::from_le_bytes([req[1], req[2]]);
        let entry = match self.read_dids.iter().find(|e| e.did == did) {
            Some(e) => e,
            None => {
                store_response(&Nrc::RequestOutOfRange.negative_response(0x22));
                return;
            }
        };
        if !session_allowed(state.session, entry.session_access) {
            store_response(&Nrc::ServiceNotSupportedInActiveSession
                .negative_response(0x22));
            return;
        }
        if !security_allowed(state.security, entry.security_level) {
            store_response(&Nrc::SecurityAccessDenied.negative_response(0x22));
            return;
        }
        let mut payload = [0u8; 64];
        match (entry.func)(&mut payload) {
            Ok(n) => {
                let mut out = [0u8; 64];
                out[0] = 0x62;
                out[1] = req[1];
                out[2] = req[2];
                let n = n.min(61);
                out[3..3 + n].copy_from_slice(&payload[..n]);
                store_response(&out[..3 + n]);
            }
            Err(nrc) => { store_response(&nrc.negative_response(0x22)); }
        }
    }

    // -- 0x2E WriteDataByIdentifier ----------------------------------

    pub fn dispatch_0x2e(&self, state: &mut UdsState, req: &[u8]) {
        // [0x2E, did_lo, did_hi, data...] → [0x6E, did_lo, did_hi] or NRC.
        if req.len() < 3 {
            store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
                .negative_response(0x2E));
            return;
        }
        let did = u16::from_le_bytes([req[1], req[2]]);
        let entry = match self.write_dids.iter().find(|e| e.did == did) {
            Some(e) => e,
            None => {
                store_response(&Nrc::RequestOutOfRange.negative_response(0x2E));
                return;
            }
        };
        if !session_allowed(state.session, entry.session_access) {
            store_response(&Nrc::ServiceNotSupportedInActiveSession
                .negative_response(0x2E));
            return;
        }
        if !security_allowed(state.security, entry.security_level) {
            store_response(&Nrc::SecurityAccessDenied.negative_response(0x2E));
            return;
        }
        match (entry.func)(&req[3..]) {
            Ok(()) => { store_response(&[0x6E, req[1], req[2]]); }
            Err(nrc) => { store_response(&nrc.negative_response(0x2E)); }
        }
    }

    // -- 0x27 SecurityAccess -----------------------------------------

    pub fn dispatch_0x27(&self, state: &mut UdsState, req: &[u8]) {
        if req.len() < 2 {
            store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
                .negative_response(0x27));
            return;
        }
        let (sal, is_request_seed) = match parse_sa_subfunc(req[1]) {
            Some(v) => v,
            None => {
                store_response(&Nrc::SubFunctionNotSupported.negative_response(0x27));
                return;
            }
        };
        if !sal_session_allowed(sal, state.session) {
            store_response(&Nrc::SubFunctionNotSupportedInActiveSession
                .negative_response(0x27));
            return;
        }
        if is_request_seed {
            sa_request_seed(state, self, sal, req[1]);
        } else {
            sa_send_key(state, self, sal, req[1], req);
        }
    }

    // -- 0x28 CommunicationControl ------------------------------------

    pub fn dispatch_0x28(&self, state: &mut UdsState, req: &[u8]) {
        // [0x28, subfunc, network_type] → [0x68, subfunc]
        if req.len() != 3 {
            store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
                .negative_response(0x28));
            return;
        }
        let subfunc = req[1];
        let _network_type = req[2];
        match subfunc {
            0x00 => {
                state.tx_disabled = false;
                info!("UDS: CommControl enable (TX ON)");
                store_response(&[0x68, 0x00]);
            }
            0x01 => {
                state.tx_disabled = true;
                info!("UDS: CommControl enableRxDisableTx (TX OFF)");
                store_response(&[0x68, 0x01]);
            }
            0x02 => {
                // enableTxDisableRx: advisory only. Dispatcher
                // always accepts incoming SDO/UDS frames; future
                // phase may wire `state.rx_disabled` into
                // canopen_task.
                info!("UDS: CommControl enableTxDisableRx (advisory)");
                store_response(&[0x68, 0x02]);
            }
            0x03 => {
                state.tx_disabled = true;
                info!("UDS: CommControl disable (TX OFF)");
                store_response(&[0x68, 0x03]);
            }
            _ => {
                store_response(&Nrc::SubFunctionNotSupported.negative_response(0x28));
            }
        }
    }

    // -- 0x31 RoutineControl (start / stop / result) -----------------

    /// `sub` picks which subfunc table to look in.
    pub fn dispatch_0x31(&self, state: &mut UdsState, req: &[u8], sub: RoutineSub) {
        // [0x31, subfunc, rid_hi, rid_lo, ...payload] →
        //   [0x71, subfunc, rid_hi, rid_lo, ...result]
        if req.len() < 4 {
            store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
                .negative_response(0x31));
            return;
        }
        let subfunc = req[1];
        let rid = u16::from_be_bytes([req[2], req[3]]);
        let table: &[RoutineEntry] = match sub {
            RoutineSub::Start => self.routines_start,
            RoutineSub::Stop => self.routines_stop,
            RoutineSub::Result => self.routines_result,
        };
        let entry = match table.iter().find(|e| e.rid == rid) {
            Some(e) => e,
            None => {
                store_response(&Nrc::RequestOutOfRange.negative_response(0x31));
                return;
            }
        };
        if !session_allowed(state.session, entry.session_access) {
            store_response(&Nrc::SubFunctionNotSupportedInActiveSession
                .negative_response(0x31));
            return;
        }
        if !security_allowed(state.security, entry.security_level) {
            store_response(&Nrc::SecurityAccessDenied.negative_response(0x31));
            return;
        }
        let payload = &req[4..];
        let mut resp_buf = [0u8; 60];
        match (entry.func)(payload, &mut resp_buf) {
            Ok(resp_len) => {
                let mut out = [0u8; 64];
                let n = resp_len.min(60);
                let total = 4 + n;
                out[0] = 0x71;
                out[1] = subfunc;
                out[2] = req[2];
                out[3] = req[3];
                out[4..4 + n].copy_from_slice(&resp_buf[..n]);
                info!("UDS: Routine 0x{:04x} sub=0x{:02x} OK ({} bytes result)",
                      rid, subfunc, resp_len);
                store_response(&out[..total]);
            }
            Err(nrc) => { store_response(&nrc.negative_response(0x31)); }
        }
    }

    // -- 0x34 / 0x36 / 0x37 OTA path (push to pending queue) --------

    /// 0x34 RequestDownload (OTA). Pushes the registered
    /// `request_download_fn` onto the pending queue.
    /// Returns `false` if validation fails or no callback set.
    pub fn dispatch_0x34(&mut self, state: &mut UdsState) -> bool {
        let req = &state.request_buf[..state.request_len];
        if req.len() != 6 || req[1] != 0x00 {
            store_response(&Nrc::RequestOutOfRange.negative_response(0x34));
            return false;
        }
        match self.request_download_fn {
            Some(f) => push_pending(state, self, f as PendingFn),
            None => {
                store_response(&Nrc::RequestOutOfRange.negative_response(0x34));
                false
            }
        }
    }

    /// 0x36 TransferData (OTA).
    pub fn dispatch_0x36(&mut self, state: &mut UdsState) -> bool {
        let req = &state.request_buf[..state.request_len];
        if req.len() != 4 {
            store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
                .negative_response(0x36));
            return false;
        }
        match self.transfer_data_fn {
            Some(f) => push_pending(state, self, f as PendingFn),
            None => {
                store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
                    .negative_response(0x36));
                false
            }
        }
    }

    /// 0x37 TransferExit (OTA).
    pub fn dispatch_0x37(&mut self, state: &mut UdsState) -> bool {
        let req = &state.request_buf[..state.request_len];
        if !req.is_empty() {
            store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
                .negative_response(0x37));
            return false;
        }
        match self.transfer_exit_fn {
            Some(f) => push_pending(state, self, f as PendingFn),
            None => {
                store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
                    .negative_response(0x37));
                false
            }
        }
    }

    // -- 0x3E TesterPresent ------------------------------------------

    pub fn dispatch_0x3e(&self, _state: &mut UdsState, req: &[u8]) {
        // [0x3E, subfunc] → [0x7E, subfunc]
        if req.len() != 2 {
            store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
                .negative_response(0x3E));
            return;
        }
        let subfunc = req[1];
        if subfunc & 0x7F == 0x00 {
            if subfunc & 0x80 != 0 {
                // Suppress positive response: stay silent.
                store_response(&[]);
            } else {
                store_response(&[0x7E, 0x00]);
            }
        } else {
            store_response(&Nrc::SubFunctionNotSupported.negative_response(0x3E));
        }
    }
}

// ============================================================================
// Helpers shared by the 0x27 dispatcher
// ============================================================================

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RoutineSub {
    Start,
    Stop,
    Result,
}

/// Generate a 16-byte seed for SecurityAccess (0x27).
///
/// Uses the configured `seed_fn` callback if set (`UdsConfig::seed_fn`);
/// otherwise falls back to a deterministic-but-changing seed from a
/// static counter. The counter ensures every RequestSeed in the same
/// session returns different bytes; it does NOT provide cryptographic
/// randomness. **Production deployments must configure `seed_fn`** to
/// point at a true entropy source (hardware RNG or system jitter).
/// Platform-independent counter-based seed. Used as fallback when
/// no `seed_fn` or RNG is available. Every call returns different
/// bytes but the sequence is deterministic — fine for smoke tests.
pub fn fallback_seed() -> AesBlock {
    static COUNTER: core::sync::atomic::AtomicU32 =
        core::sync::atomic::AtomicU32::new(0);
    let c = COUNTER.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    let mut seed = [0u8; 16];
    for i in 0usize..4 {
        let v = c.wrapping_mul(0x9E37_79B9u32.wrapping_add(i as u32));
        seed[i * 4..(i + 1) * 4].copy_from_slice(&v.to_le_bytes());
    }
    AesBlock(seed)
}

fn generate_seed(config: &UdsConfig) -> AesBlock {
    if let Some(f) = config.seed_fn {
        return f();
    }
    fallback_seed()
}

/// 0x27 subfunc → (SAL number, RequestSeed?).
fn parse_sa_subfunc(subfunc: u8) -> Option<(u8, bool)> {
    match subfunc {
        0x01 => Some((1, true)),
        0x02 => Some((1, false)),
        0x03 => Some((2, true)),
        0x04 => Some((2, false)),
        0x05 => Some((3, true)),
        0x06 => Some((3, false)),
        _ => None,
    }
}

/// Per-SAL session gate.
fn sal_session_allowed(sal: u8, session: Session) -> bool {
    match sal {
        1 => matches!(session, Session::Default | Session::Programming),
        2 => matches!(session, Session::Programming | Session::Extended),
        3 => matches!(session, Session::Extended),
        _ => false,
    }
}

fn sa_request_seed(state: &mut UdsState, config: &UdsConfig, sal: u8, subfunc: u8) {
    if (state.security as u8) >= sal {
        // Already unlocked at this SAL (or higher): ISO 14229
        // says positive response with a zero seed.
        let mut zero = [0u8; 18];
        zero[0] = subfunc + 0x40;
        zero[1] = subfunc;
        store_response(&zero);
        return;
    }
    let seed = generate_seed(config);
    state.current_seed = seed.0;
    state.seed_sent = true;
    let mut resp = [0u8; 18];
    resp[0] = subfunc + 0x40;
    resp[1] = subfunc;
    resp[2..18].copy_from_slice(&seed.0);
    info!("UDS: SecurityAccess RequestSeed(SAL{}) → {:02x}..{:02x}",
          sal, seed.0[0], seed.0[1]);
    store_response(&resp);
}

fn sa_send_key(state: &mut UdsState, config: &UdsConfig, sal: u8,
               subfunc: u8, req: &[u8]) {
    if !state.seed_sent {
        store_response(&Nrc::RequestSequenceError.negative_response(0x27));
        return;
    }
    if req.len() != 18 {  // 0x27 + subfunc + 16-byte key
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x27));
        return;
    }
    state.seed_sent = false;
    let mut rx_key = [0u8; 16];
    rx_key.copy_from_slice(&req[2..18]);
    let expected = if let Some(f) = config.key_fn {
        f(&AesBlock(state.current_seed), &config.key_masks[(sal - 1) as usize])
    } else {
        generate_key(&AesBlock(state.current_seed), &config.key_masks[(sal - 1) as usize])
    };
    if rx_key != expected.0 {
        info!("UDS: SecurityAccess SAL{} wrong key", sal);
        store_response(&Nrc::InvalidKey.negative_response(0x27));
        return;
    }
    state.security = SecurityLevel::from_u8(sal);
    info!("UDS: SecurityAccess unlocked to SAL{}", sal);
    store_response(&[subfunc + 0x40, subfunc]);
}

// OTA pending closures live in `static_config.rs`, registered via
// `UdsConfig::request_download_fn / transfer_data_fn / transfer_exit_fn`.
// When `None`, the SID returns 0x22 — zero OTA coupling.
