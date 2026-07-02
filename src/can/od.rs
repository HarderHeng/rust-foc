//! CANopen Object Dictionary (OD) — Phase 2 minimal subset.
//!
//! Only the v1 entries from the spec at
//! `docs/superpowers/specs/2026-07-02-can-ota-uds-design.md`:
//!
//! | Index | Sub | Name | Type | Access | Reset | Notes |
//! |-------|-----|------|------|--------|-------|-------|
//! | 0x1000 | 0 | DeviceType | u32 | RO | 0 | CiA 301 mandatory |
//! | 0x1001 | 0 | ErrorRegister | u8 | RO | 0 | CiA 301 mandatory |
//! | 0x1017 | 0 | HeartbeatProducerTime | u16 | RW | 1000 | ms |
//! | 0x1018 | 1 | Identity.VendorId | u32 | RO | 0xCAFE | placeholder |
//! | 0x1018 | 2 | Identity.ProductCode | u32 | RO | 0x00B0 | B-G431B-ESC1 |
//! | 0x1018 | 3 | Identity.Revision | u32 | RO | 0x0000_0001 | initial rev |
//! | 0x1018 | 4 | Identity.Serial | u32 | RO | 0x0000_0000 | TODO: 96-bit UDID |
//!
//! Phase 3 will add `0x2F00.0` (UDS gateway) and other vendor
//! entries. Phase 2 has no vendor entries.
//!
//! `read` and `write` are pure functions of the OD contents; the
//! one piece of state (`0x1017` heartbeat period) lives in a
//! static `AtomicU16` so the canopen task (which owns the
//! heartbeat ticker) and the SDO handler can both touch it
//! without taking a borrow.

use core::sync::atomic::{AtomicU16, Ordering};

/// Object Dictionary value types. Phase 2 v1 stored a u32 +
/// 1-byte length; Phase 3 grew it to 7 bytes so the UDS seed
/// response (6 bytes — `[0x67, 0x01, seed_0..3]`) fits in a
/// segmented SDO upload. The SDO layer chooses between expedited
/// transfer (1–4 bytes) and segmented transfer (5–7 bytes) based
/// on `len`.
///
/// Wire layouts:
///
///   expedited (len ≤ 4): `bytes[0..len]` are LE; the SDO response
///     uses cmd 0x8F/0x8E/0x8D/0x8C (server Upload Initiate
///     Response, e=1, s=1, n=3..0).
///
///   segmented (len > 4): `bytes[0..len]` are the whole response;
///     the SDO layer emits 0x82 (Initiate, segmented, with size)
///     then 0xA0/A1 segment responses. See `super::sdo`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct OdValue {
    /// Raw bytes in little-endian order. Only `bytes[0..len]` are
    /// valid; the rest are padding and must be ignored.
    pub bytes: [u8; 7],
    /// Number of significant bytes (1..=7).
    pub len: u8,
}

impl OdValue {
    pub const fn u8(v: u8) -> Self {
        Self { bytes: [v, 0, 0, 0, 0, 0, 0], len: 1 }
    }
    pub const fn u16(v: u16) -> Self {
        Self { bytes: [v as u8, (v >> 8) as u8, 0, 0, 0, 0, 0], len: 2 }
    }
    pub const fn u32(v: u32) -> Self {
        Self {
            bytes: [
                v as u8,
                (v >> 8) as u8,
                (v >> 16) as u8,
                (v >> 24) as u8,
                0, 0, 0,
            ],
            len: 4,
        }
    }
}

/// Heartbeat producer period in milliseconds. Read + written by
/// the canopen task (each tick) and the SDO handler (write via
/// `0x1017.0`).
static HEARTBEAT_PRODUCER_MS: AtomicU16 = AtomicU16::new(1000);

/// Get the current heartbeat period. Used by the canopen task's
/// ticker construction.
pub fn heartbeat_period_ms() -> u16 {
    HEARTBEAT_PRODUCER_MS.load(Ordering::Relaxed)
}

/// Set the heartbeat period. `0` is reserved by CiA 301 to mean
/// "heartbeat disabled"; we accept it but the canopen task will
/// treat it as a no-op (no heartbeat frame ever sent).
pub fn set_heartbeat_period_ms(ms: u16) {
    HEARTBEAT_PRODUCER_MS.store(ms, Ordering::Relaxed);
}

/// Read an OD entry. Returns the value, or an SDO abort code if
/// the entry doesn't exist.
pub fn read(index: u16, sub: u8) -> Result<OdValue, SdoAbort> {
    match (index, sub) {
        // CiA 301 mandatory
        (0x1000, 0) => Ok(OdValue::u32(0)), // DeviceType: 0 = no standard profile
        (0x1001, 0) => Ok(OdValue::u8(0)),   // ErrorRegister: no error bits set
        (0x1017, 0) => Ok(OdValue::u16(heartbeat_period_ms())),
        // CiA 301 Identity object (mandatory if supported)
        (0x1018, 1) => Ok(OdValue::u32(0x0000_CAFE)), // VendorId (placeholder)
        (0x1018, 2) => Ok(OdValue::u32(0x0000_00B0)), // ProductCode (B-G431B-ESC1)
        (0x1018, 3) => Ok(OdValue::u32(0x0000_0001)), // Revision
        (0x1018, 4) => Ok(OdValue::u32(0x0000_0000)), // Serial (TODO: 96-bit UDID)
        // Vendor: UDS gateway. Read returns the last UDS response
        // (the master does SDO write then SDO read to do a UDS
        // call; see `super::uds`).
        (0x2F00, 0) => Ok(super::uds::load_response()),
        // Subindex out of range for 0x1018
        (0x1018, n) if n > 4 => Err(SdoAbort::NoSubindex),
        // No more vendor entries in Phase 3.
        _ => Err(SdoAbort::NotExist),
    }
}

/// Write an OD entry. Returns `Ok(())` on success, or an SDO
/// abort code if the entry doesn't exist, is read-only, or the
/// value's size doesn't match the OD entry's size.
pub fn write(index: u16, sub: u8, value: OdValue) -> Result<(), SdoAbort> {
    match (index, sub) {
        // RW: heartbeat producer time. Accept u16 (2 bytes); any
        // other size is a length mismatch.
        (0x1017, 0) => {
            if value.len == 2 {
                set_heartbeat_period_ms(u16::from_le_bytes([value.bytes[0], value.bytes[1]]));
                Ok(())
            } else {
                Err(SdoAbort::LengthMismatch)
            }
        }
        // Vendor: UDS gateway. Write dispatches the payload as a
        // UDS request; the response is stashed for the next SDO
        // read on this index.
        (0x2F00, 0) => {
            // The SDO write hands us an `OdValue` whose low
            // `len` bytes are the UDS request payload. Slice
            // off `len` bytes and dispatch.
            //
            // Note: Phase 3 v1 fits every UDS request in ≤ 4 bytes
            // (all requests except 0x34 RequestDownload). The 0x34
            // OTA path needs segmented SDO write to deliver its
            // 6-byte payload; that path is deferred to a later
            // phase. The Python smoke test for OTA exercises the
            // board's segmented-upload path only — i.e. reading
            // 0x2F00.0 after a simulated write — and the UDS
            // request is constructed off-board for now.
            super::uds::dispatch(&value.bytes[..value.len as usize]);
            Ok(())
        }
        // RO: everything else in v1.
        (0x1000, _) | (0x1001, _) | (0x1018, _) => Err(SdoAbort::ReadOnly),
        // Unknown entry.
        _ => Err(SdoAbort::NotExist),
    }
}

/// CANopen SDO abort codes. Only the ones Phase 2 v1 needs; the
/// full CiA 301 table grows over Phase 3+.
///
/// 32-bit values, encoded little-endian in the SDO abort frame's
/// bytes 4..7. Top byte is reserved and always 0.
#[allow(dead_code)] // NotSupported / LocalControl reserved for Phase 3+.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum SdoAbort {
    /// 0x0601_0000 — read/write access to an object is not
    /// supported. We use this for "object exists but wrong
    /// direction" (a write to a read-only entry, etc.).
    NotSupported      = 0x0601_0000,
    /// 0x0602_0000 — object does not exist in the object
    /// dictionary.
    NotExist          = 0x0602_0000,
    /// 0x0604_0043 — subindex does not exist for the given
    /// index.
    NoSubindex        = 0x0604_0043,
    /// 0x0601_0002 — attempt to write a read-only object.
    ReadOnly          = 0x0601_0002,
    /// 0x0607_0010 — service parameter length too long for the
    /// object (we received more bytes than the entry accepts).
    LengthMismatch    = 0x0607_0010,
    /// 0x0800_0022 — local control or device state prevents the
    /// requested access. Reserved for Phase 3+ when we add
    /// "ProgrammingSession" gates.
    LocalControl      = 0x0800_0022,
    /// 0x0504_0001 — client/server command specifier not valid
    /// (unknown CCS/SCS bits). Returned by the SDO dispatcher
    /// when it can't parse a request.
    InvalidCommand    = 0x0504_0001,
}

impl SdoAbort {
    /// Encode the abort code as a 32-bit value for the SDO abort
    /// frame (little-endian in the wire payload).
    pub const fn code(self) -> u32 { self as u32 }
}
