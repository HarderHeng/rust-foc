//! DTC (Diagnostic Trouble Code) storage — ISO 14229-1 0x14/0x19.
//!
//! ## Layering
//!
//! This module owns the persistent DTC record array and exposes a
//! public API that any fault-detection code can call:
//!
//! - `set_dtc(code, status)` — record or update a fault
//! - `clear_all()` — wipe all records (called by 0x14)
//! - `report_by_status_mask(mask) -> [u8]` — build a 0x19 0x02 response
//!
//! ## Decoupling
//!
//! Fault detection lives OUTSIDE this crate (e.g. in a CANopen
//! task). It imports `uds_core::dtc::set_dtc` — a simple function
//! call, no callback registration needed.

use core::cell::RefCell;
use critical_section::Mutex;

use crate::state::store_response;
use crate::types::Nrc;

// ---- Constants ------------------------------------------------------------

/// Maximum number of DTC records stored simultaneously.
const MAX_DTCS: usize = 8;

// ---- Public types ---------------------------------------------------------

/// 3-byte DTC code in ISO 15031-6 format.
///
/// High nibble of byte 0 encodes the group:
///   0x0 = P (Powertrain)
///   0x1 = C (Chassis)
///   0x2 = B (Body)
///   0x3 = U (Network)
///
/// The remaining bits + bytes 1-2 encode the specific fault.
/// Stored as a u32 for convenience (top byte unused).
pub type DtcCode = u32;

/// Status bitmask per ISO 14229-1 Table 55.
#[allow(dead_code)]
pub mod status {
    pub const TEST_FAILED: u8                 = 0x01;
    pub const TEST_FAILED_CURRENT: u8          = 0x02;
    pub const CONFIRMED: u8                    = 0x04;
    pub const PENDING: u8                      = 0x08;
    pub const PREVIOUSLY_CONFIRMED: u8         = 0x10;
    pub const WARNING_LAMP_REQUESTED: u8       = 0x20;
    pub const WARNING_LAMP_ON: u8              = 0x40;
    pub const NOT_AVAILABLE: u8                = 0x80;
}

// ---- Internal storage -----------------------------------------------------

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct DtcRecord {
    code: DtcCode,
    status: u8,
}

static DTCS: Mutex<RefCell<[Option<DtcRecord>; MAX_DTCS]>> =
    Mutex::new(RefCell::new([None; MAX_DTCS]));

// ---- Public API -----------------------------------------------------------

/// Record a DTC. If the code already exists, OR the new status bits in.
/// If not, find a free slot (or replace the oldest entry).
pub fn set_dtc(code: DtcCode, bits: u8) {
    critical_section::with(|cs| {
        let mut storage = DTCS.borrow_ref_mut(cs);
        // Try to update existing record.
        for slot in storage.iter_mut() {
            if let Some(ref mut rec) = slot {
                if rec.code == code {
                    rec.status |= bits;
                    return;
                }
            }
        }
        // Find first free slot.
        for slot in storage.iter_mut() {
            if slot.is_none() {
                *slot = Some(DtcRecord { code, status: bits });
                return;
            }
        }
        // All full — overwrite oldest (first slot).
        storage[0] = Some(DtcRecord { code, status: bits });
    });
}

/// Clear a specific DTC (removes it from storage).
pub fn clear_dtc(code: DtcCode) {
    critical_section::with(|cs| {
        let mut storage = DTCS.borrow_ref_mut(cs);
        for slot in storage.iter_mut() {
            if let Some(rec) = slot {
                if rec.code == code {
                    *slot = None;
                    return;
                }
            }
        }
    });
}

/// Clear all DTCs matching the given group mask.
/// 0xFFFFFF = clear everything.
pub fn clear_group(group: u32) {
    let group_hi = (group >> 16) as u8;
    let group_mid = (group >> 8) as u8;
    let group_lo = group as u8;
    critical_section::with(|cs| {
        let mut storage = DTCS.borrow_ref_mut(cs);
        if group_hi == 0xFF && group_mid == 0xFF && group_lo == 0xFF {
            // Clear all.
            for slot in storage.iter_mut() {
                *slot = None;
            }
            return;
        }
        // Clear matching group (by high nibble of most-significant byte).
        let group_high_nibble = (group >> 16) as u8 & 0xF0;
        for slot in storage.iter_mut() {
            if let Some(rec) = slot {
                if (rec.code >> 16) as u8 & 0xF0 == group_high_nibble {
                    *slot = None;
                }
            }
        }
    });
}

/// Build a `reportDTCByStatusMask` (0x19 0x02) response payload.
///
/// Wire format:
///   [0x59, 0x02, status_avail, count_hi, count_lo,
///    dtc1_hi, dtc1_mid, dtc1_lo, dtc1_status,
///    ...]
fn report_by_status_mask(status_mask: u8) -> [u8; 64] {
    let mut out = [0u8; 64];
    out[0] = 0x59;
    out[1] = 0x02;
    // statusAvailability byte: bits 0-6 indicate which status bits
    // the ECU supports. We support all standard bits.
    out[2] = 0xFE;
    let mut count = 0u16;
    let mut offset = 5; // header = 5 bytes
    critical_section::with(|cs| {
        let storage = DTCS.borrow_ref(cs);
        for slot in storage.iter() {
            if let Some(rec) = slot {
                if rec.status & status_mask != 0 {
                    // 3-byte DTC code (big-endian in wire format).
                    let code_bytes = rec.code.to_be_bytes();
                    out[offset]     = code_bytes[1]; // high byte
                    out[offset + 1] = code_bytes[2]; // mid byte
                    out[offset + 2] = code_bytes[3]; // low byte
                    out[offset + 3] = rec.status;
                    offset += 4;
                    count += 1;
                }
            }
        }
    });
    out[3] = (count >> 8) as u8;
    out[4] = count as u8;
    out
}

// ---- 0x14 / 0x19 handlers (called from table.rs) -------------------------

pub fn handle_clear(req: &[u8]) {
    // [0x14, group(3)] → [0x54]
    if req.len() != 4 {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x14));
        return;
    }
    let group = u32::from_be_bytes([0, req[1], req[2], req[3]]);
    clear_group(group);
    store_response(&[0x54]);
}

pub fn handle_read(req: &[u8]) {
    // [0x19, subfunc, ...] → see dispatch_0x19 for wire format.
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
    let status_mask = req[2];
    let out = report_by_status_mask(status_mask);
    let count = u16::from_be_bytes([out[3], out[4]]);
    let len = 5 + count as usize * 4;
    store_response(&out[..len]);
}
