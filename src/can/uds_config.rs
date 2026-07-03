//! Static UDS configuration. Single instance lives here.
//!
//! To add a new DID or routine, edit this file. No dispatcher
//! change needed (table-driven dispatch).

use crate::can::uds::config::{
    DidReadEntry, RoutineEntry, ServiceEntry, ServiceHandler, UdsConfig,
};
use crate::can::uds::nrc::Nrc;

// Service table. Order is irrelevant (linear search); we group
// related services for readability.
static SERVICES: &[ServiceEntry] = &[
    ServiceEntry::new(0x10, 0b111, 0, ServiceHandler::Session),
    ServiceEntry::new(0x11, 0b011, 0, ServiceHandler::EcuReset),
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

// DID tables. Adding a DID = adding a single entry here; the
// dispatcher's `find` call picks it up.

/// 0xF186 = ActiveDiagSession. Read-only 1-byte value.
fn read_active_session(out: &mut [u8; 7]) -> Result<usize, Nrc> {
    out[0] = crate::can::uds::state::load_response_session_byte();
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

// Write DIDs: none in v1. Phase 5b will add at least one.
static WRITE_DIDS: &[crate::can::uds::config::DidWriteEntry] = &[];

// Pending queue (Phase 5c). 4 slots covers TransferData +
// TransferExit + 2 waiting.
static mut PENDING_QUEUE: [Option<crate::can::uds::pending::PendingJob>; 4]
    = [None, None, None, None];

// RoutineControl (0x31) tables. Phase 5b registers two example
// routines with stub callbacks (real OTA wiring is Phase 5c).
//
// 0xFF00 = checkProgrammingDependencies (per ISO 14229 / UDS on
//          CAN, used at the start of every flash session).
// 0xF001 = checkProgrammingPreConditions (an OBC-specific
//          vendor check we can add later).

fn routine_noop(_req: &[u8], _resp: &mut [u8]) -> Result<usize, Nrc> { Ok(0) }
fn routine_check_pre(_req: &[u8], resp: &mut [u8]) -> Result<usize, Nrc> {
    // 1-byte response: 0x00 = "pre-conditions met".
    resp[0] = 0x00;
    Ok(1)
}

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

// LFSR masks per SAL. Phase 5a uses a single hardcoded mask; the
// other slots are placeholders. To compute: pick any non-zero
// 32-bit value; the LFSR cycle is then unique to that mask.
// The Phase 5a smoke test expects the existing seed/key pair
// (seed 0xA5A5A5A5 → key 0xA5A5B7D9); the test was updated in 5d
// to derive the key from this mask.
pub static mut UDS_CONFIG: UdsConfig = UdsConfig {
    services: SERVICES,
    read_dids: READ_DIDS,
    write_dids: WRITE_DIDS,
    routines_start: ROUTINES_START,
    routines_stop: ROUTINES_STOP,
    routines_result: ROUTINES_RESULT,
    pending_queue: unsafe { &mut PENDING_QUEUE },
    p2_server_ms: 50,
    p2_star_ms: 5000,
    key_masks: [0x3000_2212, 0x524C_5E63, 0xA5C3_F11B],
    on_default_session_enter: None,
    on_programming_session_enter: None,
    on_extended_session_enter: None,
};
