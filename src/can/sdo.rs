//! CANopen SDO server protocol — Phase 2 minimal subset.
//!
//! Wire-level handling only. The Object Dictionary side of things
//! lives in `super::od`. This module owns:
//!   - Parsing incoming SDO request frames (Initiate Download /
//!     Initiate Upload) into a `SdoRequest` enum
//!   - Encoding SDO response frames (success or abort) into a
//!     fresh 8-byte `Frame` ready to be sent on the SDO
//!     server-transmit COB-ID (`0x580 + NodeId`)
//!   - The "Phase 2 v1" simplification: expedited transfer only,
//!     no segmented transfer. A segmented request gets an
//!     InvalidCommand abort.
//!
//! ## Frame layout reminder
//!
//! Initiate Download Request (client → server, 8 bytes):
//!   byte 0: 0x20 | e | s | (n2 | n1 | n0)   (e=1, s=1 for
//!                                             expedited; n
//!                                             encodes (4 - num
//!                                             data bytes))
//!   bytes 1..3: index (LE u16) | subindex
//!   bytes 4..7: data (LE, up to 4 bytes for expedited)
//!
//! Initiate Download Response (server → client):
//!   byte 0: 0x60 (success, no data)
//!
//! Initiate Upload Request (client → server):
//!   byte 0: 0x40
//!   bytes 1..3: index | subindex
//!
//! Initiate Upload Response (server → client):
//!   byte 0: 0x40 | e | s | n         (e=1, s=1 for expedited)
//!   bytes 1..3: index | subindex
//!   bytes 4..7: data (LE, up to 4 bytes)
//!
//! Abort Transfer:
//!   byte 0: 0x80
//!   bytes 1..3: index | subindex (or 0 if not applicable)
//!   bytes 4..7: abort code (LE u32)

use defmt::warn;
use embassy_stm32::can::Frame;
use embedded_can::Id;

use super::canopen::NODE_ID;
use super::od::{read as od_read, write as od_write, OdValue, SdoAbort};

/// SDO server receive (master → slave) COB-ID.
pub const SDO_RX_COB_ID: u16 = 0x600 + NODE_ID as u16;
/// SDO server transmit (slave → master) COB-ID.
pub const SDO_TX_COB_ID: u16 = 0x580 + NODE_ID as u16;

/// Top-3-bit CCS/SCS field (byte 0, bits 7-5). These are the
/// only "valid" request / response types in Phase 2 v1.
const SDO_CMD_MASK: u8 = 0xE0;
const SDO_CMD_DOWNLOAD: u8 = 0x20; // Initiate Download Request
const SDO_CMD_UPLOAD: u8   = 0x40; // Initiate Upload Request
const SDO_CMD_ABORT: u8    = 0x80; // Abort Transfer

/// e (expedited) and s (size indicator) flag bits in byte 0.
const SDO_FLAG_E: u8 = 0x02;
const SDO_FLAG_S: u8 = 0x01;

/// n field mask (bits 2..3 of byte 0, 2 bits).
///
/// Per CiA 301 § 7.2.4.3.2 the SDO command specifier layout for
/// Initiate Download is:
///
///   bits 7..5 = CCS (0b001 for download initiate)
///   bit  4   = reserved (0 for download initiate)
///   bits 3..2 = n (number of data bytes NOT used in the 4-byte
///                payload — so num_bytes = 4 - n)
///   bit  1   = e (expedited)
///   bit  0   = s (size indicator)
///
/// An earlier version of this file used `0x1C` here, which
/// included bit 4 (the reserved bit) and caused 1-byte writes
/// (cmd = 0x2F, where bit 4 is set in the CANopen spec) to
/// compute `n = 7` and be rejected with InvalidCommand.
const SDO_N_MASK: u8 = 0x0C;

/// Parsed SDO request from a received frame's first 8 bytes.
pub enum SdoRequest {
    /// Client asks us to write a value into the OD.
    Download { index: u16, sub: u8, value: OdValue },
    /// Client asks us to read a value from the OD.
    Upload   { index: u16, sub: u8 },
}

/// Parse a received SDO request frame. Returns `None` for
/// non-SDO COB-IDs, segmented / abort transfers (Phase 2 v1
/// only handles expedited), and unknown command specifiers.
///
/// `data` is the 8-byte payload of the frame (passed in as a
/// slice so callers can decide how to copy; we never modify it).
pub fn parse_request(data: &[u8]) -> Option<Result<SdoRequest, SdoAbort>> {
    if data.len() < 8 {
        // A malformed frame on the SDO COB-ID is suspect; just
        // skip (don't abort — there's no index to attach).
        return None;
    }
    let cmd = data[0];
    let index = u16::from_le_bytes([data[1], data[2]]);
    let sub = data[3];
    match cmd & SDO_CMD_MASK {
        SDO_CMD_DOWNLOAD => {
            let e = cmd & SDO_FLAG_E;
            let s = cmd & SDO_FLAG_S;
            if e == 0 || s == 0 {
                // Segmented or no-size. Not supported in Phase 2.
                return Some(Err(SdoAbort::InvalidCommand));
            }
            let n = (cmd & SDO_N_MASK) >> 2;
            // n = 4 - num_data_bytes, so num_data_bytes = 4 - n.
            // Valid range: n in 0..=3 (1..=4 bytes).
            if n > 3 {
                return Some(Err(SdoAbort::InvalidCommand));
            }
            let num_bytes = 4 - n;
            // Pack the relevant `num_bytes` of payload (LE) into
            // a u32. Bytes beyond `num_bytes` are ignored.
            let mut bits: u32 = 0;
            for i in 0..(num_bytes as usize) {
                bits |= (data[4 + i] as u32) << (8 * i);
            }
            Some(Ok(SdoRequest::Download {
                index,
                sub,
                value: OdValue { size: num_bytes as u8, bits },
            }))
        }
        SDO_CMD_UPLOAD => {
            // Initiate Upload Request. e and s are not used.
            Some(Ok(SdoRequest::Upload { index, sub }))
        }
        SDO_CMD_ABORT => {
            // A client-initiated abort. We don't have any state to
            // tear down in Phase 2 (no segmented transfer), so
            // just ignore.
            None
        }
        _ => Some(Err(SdoAbort::InvalidCommand)),
    }
}

/// Build a 0x60 success response for an Initiate Download Request.
pub fn build_download_ok_response() -> Frame {
    // unwrap: 8-byte frame with arbitrary payload.
    Frame::new_standard(SDO_TX_COB_ID, &[0x60, 0, 0, 0, 0, 0, 0, 0])
        .expect("SDO response is 8 bytes, always valid")
}

/// Build a 0x4_ Initiate Upload Response carrying `value`.
///
/// Picks the right e/s/n bits based on `value.size`:
///   size=1: 0x4F
///   size=2: 0x4B
///   size=3: 0x47
///   size=4: 0x43
/// (or any other value: 0x40 with s=0 — server has no
/// segmented-up data, so this only happens for malformed OD
/// values; we use it defensively.)
pub fn build_upload_response(index: u16, sub: u8, value: OdValue) -> Frame {
    let cmd = match value.size {
        1 => 0x4F, // e=1, s=1, n=3 → 1 byte
        2 => 0x4B, // e=1, s=1, n=2 → 2 bytes
        3 => 0x47, // e=1, s=1, n=1 → 3 bytes
        4 => 0x43, // e=1, s=1, n=0 → 4 bytes
        _ => 0x40, // no expedited data; Phase 2 v1 doesn't expect this
    };
    let index_bytes = index.to_le_bytes();
    let mut payload = [0u8; 8];
    payload[0] = cmd;
    payload[1] = index_bytes[0];
    payload[2] = index_bytes[1];
    payload[3] = sub;
    for i in 0..(value.size as usize).min(4) {
        payload[4 + i] = ((value.bits >> (8 * i)) & 0xFF) as u8;
    }
    Frame::new_standard(SDO_TX_COB_ID, &payload)
        .expect("SDO response is 8 bytes, always valid")
}

/// Build an abort frame for the given (index, sub).
pub fn build_abort_response(index: u16, sub: u8, code: SdoAbort) -> Frame {
    let index_bytes = index.to_le_bytes();
    let code_bytes = code.code().to_le_bytes();
    let payload = [
        0x80,
        index_bytes[0], index_bytes[1], sub,
        code_bytes[0], code_bytes[1], code_bytes[2], code_bytes[3],
    ];
    Frame::new_standard(SDO_TX_COB_ID, &payload)
        .expect("SDO abort is 8 bytes, always valid")
}

/// Handle one SDO request frame, returning the response frame
/// (or `None` if no response should be sent — e.g. an abort
/// received from the master that needs no reply).
///
/// On parse failure or OD error, returns an abort frame with
/// the appropriate SDO abort code. The caller just needs to
/// `can.write(&response).await`.
pub fn dispatch(data: &[u8]) -> Option<Frame> {
    let parsed = match parse_request(data) {
        Some(Ok(req)) => req,
        Some(Err(abort)) => {
            // No index/sub to echo back in the abort frame (the
            // parse failed before we read them); use 0/0.
            return Some(build_abort_response(0, 0, abort));
        }
        None => return None,
    };

    match parsed {
        SdoRequest::Download { index, sub, value } => {
            match od_write(index, sub, value) {
                Ok(()) => {
                    defmt::info!(
                        "SDO: write 0x{:04x}:{} = {} bytes OK",
                        index, sub, value.size
                    );
                    Some(build_download_ok_response())
                }
                Err(abort) => {
                    warn!("SDO: write 0x{:04x}:{} abort 0x{:08x}", index, sub, abort.code());
                    Some(build_abort_response(index, sub, abort))
                }
            }
        }
        SdoRequest::Upload { index, sub } => {
            match od_read(index, sub) {
                Ok(value) => {
                    defmt::info!(
                        "SDO: read 0x{:04x}:{} = {} bytes",
                        index, sub, value.size
                    );
                    Some(build_upload_response(index, sub, value))
                }
                Err(abort) => {
                    warn!("SDO: read 0x{:04x}:{} abort 0x{:08x}", index, sub, abort.code());
                    Some(build_abort_response(index, sub, abort))
                }
            }
        }
    }
}

/// Check whether a received frame is addressed to our SDO
/// server (the master → slave COB-ID). Used by the canopen
/// task to route frames to `dispatch`.
pub fn is_sdo_request(frame: &Frame) -> bool {
    let id = match frame.header().id() {
        Id::Standard(s) => s.as_raw(),
        Id::Extended(_) => return false,
    };
    id == SDO_RX_COB_ID
}
