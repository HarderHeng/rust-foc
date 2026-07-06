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

use crate::uds::crypto::AesBlock;
use crate::uds::table::{DidReadEntry, DidWriteEntry, RoutineEntry,
                          ServiceEntry, ServiceHandler, UdsConfig};
use crate::uds::types::Nrc;

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
fn read_active_session(out: &mut [u8; 64]) -> Result<usize, Nrc> {
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

// ---- DID write callbacks ---------------------------------------------------

/// 0xF180 = KeyDataMasks (vendor-specific, writable). Accepts 48 bytes
/// = 3 × AES-128 key (16 bytes each, LE):
///   `[mask_sal1(16), mask_sal2(16), mask_sal3(16)]`.
/// All-zero masks are rejected.
fn write_key_masks(data: &[u8]) -> Result<(), Nrc> {
    if data.len() != 48 {
        return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
    }
    let mut raw = [[0u8; 16]; 3];
    for i in 0..3 {
        raw[i].copy_from_slice(&data[i * 16..(i + 1) * 16]);
        if raw[i].iter().all(|&b| b == 0) {
            return Err(Nrc::RequestOutOfRange);
        }
    }
    let masks = [
        AesBlock(raw[0]),
        AesBlock(raw[1]),
        AesBlock(raw[2]),
    ];
    // Safety: single-threaded executor; called from dispatch which
    // is the sole owner of the UDS_CONFIG static.
    unsafe {
        let cfg = &mut *(&raw mut crate::uds::static_config::UDS_CONFIG
                         as *mut crate::uds::table::UdsConfig);
        cfg.key_masks = masks;
    }
    defmt::info!("UDS: key_masks updated via DID 0xF180");
    Ok(())
}

static WRITE_DIDS: &[DidWriteEntry] = &[
    DidWriteEntry {
        did: 0xF180,
        session_access: 0b011,  // Programming | Extended
        security_level: 2,       // SAL2+
        func: write_key_masks,
    },
];

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
    key_masks: [
        AesBlock::from_bytes([
            0x30, 0x00, 0x22, 0x12, 0xAB, 0xCD, 0xEF, 0x01,
            0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x45, 0x67,
        ]),
        AesBlock::from_bytes([
            0x52, 0x4C, 0x5E, 0x63, 0xDE, 0xAD, 0xBE, 0xEF,
            0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
        ]),
        AesBlock::from_bytes([
            0xA5, 0xC3, 0xF1, 0x1B, 0xCA, 0xFE, 0xBA, 0xBE,
            0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12,
        ]),
    ],
    on_default_session_enter: None,
    on_programming_session_enter: None,
    on_extended_session_enter: None,
    seed_fn: None,
};
