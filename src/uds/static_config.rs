//! Static `UdsConfig` instance + the callback functions
//! referenced by the table entries.
//!
//! To add a new DID: add a callback fn here + a `DidReadEntry`
//! row in `READ_DIDS`. To add a new routine: same pattern in
//! `ROUTINES_START` / `_STOP` / `_RESULT`. To add a new SID:
//! add a `ServiceEntry` row in `SERVICES` (and a
//! `dispatch_0xNN` method in `table.rs`).
//!
//! The pending queue is `static mut` because `Option<PendingJob>`
//! holds a closure (not `Sync`); the `unsafe` is required at
//! the static initializer.

use crate::uds::table::{DidReadEntry, DidWriteEntry, RoutineEntry,
                          ServiceEntry, ServiceHandler, UdsConfig};
use crate::uds::types::{Nrc, Session};

// Service table. Order is irrelevant (linear search); we group
// related services for readability.
static SERVICES: &[ServiceEntry] = &[
    ServiceEntry::new(0x10, 0b111, 0, ServiceHandler::Session),
    ServiceEntry::new(0x11, 0b111, 0, ServiceHandler::EcuReset),
    ServiceEntry::new(0x14, 0b111, 0, ServiceHandler::ClearDtc),
    ServiceEntry::new(0x19, 0b111, 0, ServiceHandler::ReadDtc),
    ServiceEntry::new(0x22, 0b111, 0, ServiceHandler::ReadDataById),
    ServiceEntry::new(0x27, 0b111, 0, ServiceHandler::SecurityAccess),
    ServiceEntry::new(0x28, 0b111, 0, ServiceHandler::CommControl),
    ServiceEntry::new(0x2E, 0b111, 0, ServiceHandler::WriteDataById),
    ServiceEntry::new(0x31, 0b111, 0, ServiceHandler::RoutineStart),
    ServiceEntry::new(0x34, 0b010, 1, ServiceHandler::RequestDownload),
    ServiceEntry::new(0x36, 0b010, 1, ServiceHandler::TransferData),
    ServiceEntry::new(0x37, 0b010, 1, ServiceHandler::TransferExit),
    ServiceEntry::new(0x3E, 0b111, 0, ServiceHandler::TesterPresent),
];

// ---- DID read callbacks --------------------------------------------------

/// 0xF186 = ActiveDiagSession. Read-only 1-byte value.
fn read_active_session(out: &mut [u8; 7]) -> Result<usize, Nrc> {
    out[0] = crate::uds::UDS_STATE.session.as_u8();
    Ok(1)
}

static READ_DIDS: &[DidReadEntry] = &[
    DidReadEntry {
        did: 0xF186,
        session_access: 0b111,  // any session
        security_level: 0,       // no SAL required
        func: read_active_session,
    },
];

// Write DIDs: none in v1.
static WRITE_DIDS: &[DidWriteEntry] = &[];

// ---- Pending queue -------------------------------------------------------

// 4 slots covers TransferData + TransferExit + 2 waiting.
static mut PENDING_QUEUE: [Option<crate::uds::pending::PendingJob>; 4]
    = [None, None, None, None];

// ---- Routine callbacks ---------------------------------------------------

fn routine_noop(_req: &[u8], _resp: &mut [u8]) -> Result<usize, Nrc> { Ok(0) }

fn routine_check_pre(_req: &[u8], resp: &mut [u8]) -> Result<usize, Nrc> {
    // 1-byte response: 0x00 = "pre-conditions met".
    resp[0] = 0x00;
    Ok(1)
}

// 0xFF00 = checkProgrammingDependencies (per ISO 14229 / UDS
//          on CAN, used at the start of every flash session).
// 0xF001 = checkProgrammingPreConditions (vendor check; the
//          default callback in our test rig returns 0x00).
static ROUTINES_START: &[RoutineEntry] = &[
    RoutineEntry {
        rid: 0xFF00,
        session_access: 0b011,  // Programming | Extended
        security_level: 1,
        func: routine_noop,
    },
];
static ROUTINES_STOP: &[RoutineEntry] = &[
    RoutineEntry {
        rid: 0xFF00,
        session_access: 0b011,
        security_level: 1,
        func: routine_noop,
    },
];
static ROUTINES_RESULT: &[RoutineEntry] = &[
    RoutineEntry {
        rid: 0xF001,
        session_access: 0b111,
        security_level: 0,
        func: routine_check_pre,
    },
];

// ---- The single static instance -----------------------------------------

/// LFSR masks per SAL. Index 0/1/2 = SAL1/2/3. Pick any
/// non-zero 32-bit value; the LFSR cycle is then unique to
/// that mask. The smoke tests expect the existing seed/key
/// pairs derived from these masks — see
/// `scripts/smoke_test.py::_lfsr_key`.
pub static mut UDS_CONFIG: UdsConfig = UdsConfig {
    services: SERVICES,
    read_dids: READ_DIDS,
    write_dids: WRITE_DIDS,
    routines_start: ROUTINES_START,
    routines_stop: ROUTINES_STOP,
    routines_result: ROUTINES_RESULT,
    pending_queue: unsafe { &mut PENDING_QUEUE },
    p2_server_ms: 50,
    key_masks: [0x3000_2212, 0x524C_5E63, 0xA5C3_F11B],
    on_default_session_enter: None,
    on_programming_session_enter: None,
    on_extended_session_enter: None,
};

#[allow(dead_code)]
fn _silence_unused(_s: &Session) {}
