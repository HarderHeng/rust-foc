//! CANopen SDO server protocol — Phase 2 minimal subset.
//!
//! Wire-level handling only. The Object Dictionary side of things
//! lives in `super::od`. This module owns:
//!   - Parsing incoming SDO request frames (Initiate Download /
//!     Initiate Upload / Upload Segment) into a `SdoRequest` enum
//!   - Encoding SDO response frames (success or abort) into a
//!     fresh 8-byte `Frame` ready to be sent on the SDO
//!     server-transmit COB-ID (`0x580 + NodeId`)
//!   - The segmented-upload state machine (Phase 3 v2): a static
//!     buffer holds the bytes to deliver across one or more
//!     Upload Segment responses after a segmented Initiate.
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
//!   - expedited (1–4 bytes):
//!     byte 0: 0x80 | (e=1)<<3 | (s=1)<<2 | n         — n is
//!              bits 0–1, equals (4 - size).
//!              size=1 → 0x8F
//!              size=2 → 0x8E
//!              size=3 → 0x8D
//!              size=4 → 0x8C
//!     bytes 1..3: index | subindex
//!     bytes 4..7: data (LE)
//!   - segmented (5–7 bytes — v2 supports up to 7):
//!     byte 0: 0x82                                          (scs=4, s=1, e=0, n=0)
//!     bytes 1..3: index | subindex
//!     bytes 4..5: total size (LE u16)
//!     bytes 6..7: 0 bytes of data (all of the data follows in segments)
//!
//! Upload Segment Request (client → server, after a 0x82):
//!   byte 0: 0x60 | (toggle<<4)                                (toggle alternates 0/1)
//!   bytes 1..7: ignored
//!
//! Upload Segment Response (server → client):
//!   byte 0: 0xA0 | (toggle<<4) | (n<<1) | c                   (n = 7 - num_data_bytes; c=1 on last segment)
//!   bytes 1..7: up to 7 bytes of data
//!
//! Abort Transfer:
//!   byte 0: 0x80
//!   bytes 1..3: index | subindex (or 0 if not applicable)
//!   bytes 4..7: abort code (LE u32)

use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, Ordering};

use defmt::warn;
use embassy_stm32::can::Frame;
use embedded_can::Id;

use super::canopen::NODE_ID;
use super::od::{read as od_read, write as od_write, OdValue, SdoAbort};

/// SDO server receive (master → slave) COB-ID.
pub const SDO_RX_COB_ID: u16 = 0x600 + NODE_ID as u16;
/// SDO server transmit (slave → master) COB-ID.
pub const SDO_TX_COB_ID: u16 = 0x580 + NODE_ID as u16;

/// Top-3-bit CCS/SCS field (byte 0, bits 7-5).
const SDO_CMD_MASK: u8 = 0xE0;
const SDO_CMD_DOWNLOAD:     u8 = 0x20; // CCS=1 — Initiate Download Request
const SDO_CMD_UPLOAD:       u8 = 0x40; // CCS=2 — Initiate Upload Request
const SDO_CMD_UPLOAD_SEG:   u8 = 0x60; // CCS=3 — Upload Segment Request (client asks for next chunk)
const SDO_CMD_ABORT:        u8 = 0x80; // SCS=4 — Abort Transfer

/// e (expedited) and s (size indicator) flag bits in byte 0
/// (for the Initiate Download Request, where they live in the
/// low bits).
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

/// Maximum payload bytes for an expedited SDO response (4 bytes
/// of data + 1 cmd byte leaves 3 bytes for index/sub and 4 for
/// data).
const EXPEDITED_MAX: usize = 4;

/// Maximum payload bytes for a segmented SDO response (one
/// segment is up to 7 bytes; we cap total response at 7 bytes
/// because Phase 3 v2 only needs to fit the SecurityAccess seed
/// response — 6 bytes — and going larger requires more
/// architecture work, e.g. handling multiple segments).
const SEGMENTED_MAX: usize = 7;

/// Parsed SDO request from a received frame's first 8 bytes.
pub enum SdoRequest {
    /// Client asks us to write a value into the OD.
    Download { index: u16, sub: u8, value: OdValue },
    /// Client asks us to read a value from the OD.
    Upload   { index: u16, sub: u8 },
    /// Client asks for the next segment of an in-flight
    /// segmented upload (after a 0x82 Initiate).
    UploadSegment { toggle: u8 },
}

// ---- Segmented upload state machine ---------------------------------

/// State for an in-flight segmented upload. `bytes[0..len]` is
/// the response to deliver; `offset` is the index of the next
/// byte to send on the next Upload Segment request. When
/// `len == 0` no upload is in progress (the next segmented
/// Initiate will overwrite this).
static SDO_UPLOAD_BUF: critical_section::Mutex<RefCell<[u8; SEGMENTED_MAX]>> =
    critical_section::Mutex::new(RefCell::new([0; SEGMENTED_MAX]));
static SDO_UPLOAD_LEN: AtomicU8 = AtomicU8::new(0);
static SDO_UPLOAD_OFFSET: AtomicU8 = AtomicU8::new(0);
/// Toggle bit (0 or 1) for the next segment response. The spec
/// says the toggle alternates with each segment; we initialise to
/// 0 on every Initiate and flip after each segment.
static SDO_UPLOAD_TOGGLE: AtomicU8 = AtomicU8::new(0);

/// Store `bytes` (length `len`) as the segmented-upload payload,
/// replacing any in-progress upload. Called from
/// `build_upload_response` when the response is too big for
/// expedited transfer.
fn segmented_upload_begin(bytes: &[u8]) {
    debug_assert!(bytes.len() <= SEGMENTED_MAX);
    let len = bytes.len() as u8;
    critical_section::with(|cs| {
        let buf = &mut *SDO_UPLOAD_BUF.borrow_ref_mut(cs);
        buf[..bytes.len()].copy_from_slice(bytes);
    });
    SDO_UPLOAD_LEN.store(len, Ordering::Relaxed);
    SDO_UPLOAD_OFFSET.store(0, Ordering::Relaxed);
    SDO_UPLOAD_TOGGLE.store(0, Ordering::Relaxed);
}

/// Pull the next segment from the in-flight upload. Returns
/// `(segment_bytes, last)` where `last` is true iff this segment
/// is the final one for the upload. Returns `None` if no upload
/// is in progress (SDO_UPLOAD_LEN == 0) — the caller should
/// translate that into an abort.
fn segmented_upload_next() -> Option<([u8; 7], usize, bool)> {
    let len = SDO_UPLOAD_LEN.load(Ordering::Relaxed) as usize;
    if len == 0 {
        return None;
    }
    let offset = SDO_UPLOAD_OFFSET.load(Ordering::Relaxed) as usize;
    debug_assert!(offset < len);
    let chunk = (len - offset).min(7);
    let mut seg = [0u8; 7];
    critical_section::with(|cs| {
        let buf = SDO_UPLOAD_BUF.borrow_ref(cs);
        seg[..chunk].copy_from_slice(&buf[offset..offset + chunk]);
    });
    let new_offset = offset + chunk;
    let last = new_offset == len;
    SDO_UPLOAD_OFFSET.store(new_offset as u8, Ordering::Relaxed);
    if last {
        // Clear state so a stray segment request after the last
        // one returns an abort instead of replaying.
        SDO_UPLOAD_LEN.store(0, Ordering::Relaxed);
        SDO_UPLOAD_OFFSET.store(0, Ordering::Relaxed);
    }
    let toggle = SDO_UPLOAD_TOGGLE.load(Ordering::Relaxed);
    SDO_UPLOAD_TOGGLE.store(toggle ^ 1, Ordering::Relaxed);
    Some((seg, chunk, last))
}

// ---- Parsing ---------------------------------------------------------

/// Parse a received SDO request frame. Returns `None` for
/// non-SDO COB-IDs, abort transfers (Phase 2 v1 only handles
/// expedited download + expedited/segmented upload), and
/// unknown command specifiers.
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
                // (OTA's 0x34 RequestDownload is 5 bytes — the
                // 0x2F00 gateway would need segmented download
                // to receive it. Deferred to a later phase.)
                return Some(Err(SdoAbort::InvalidCommand));
            }
            let n = (cmd & SDO_N_MASK) >> 2;
            // n = 4 - num_data_bytes, so num_data_bytes = 4 - n.
            // Valid range: n in 0..=3 (1..=4 bytes).
            if n > 3 {
                return Some(Err(SdoAbort::InvalidCommand));
            }
            let num_bytes = 4 - n;
            let mut bytes = [0u8; 7];
            for i in 0..(num_bytes as usize) {
                bytes[i] = data[4 + i];
            }
            Some(Ok(SdoRequest::Download {
                index,
                sub,
                value: OdValue { bytes, len: num_bytes as u8 },
            }))
        }
        SDO_CMD_UPLOAD => {
            // Initiate Upload Request. e and s are not used.
            Some(Ok(SdoRequest::Upload { index, sub }))
        }
        SDO_CMD_UPLOAD_SEG => {
            // Upload Segment Request (after a 0x82 Initiate).
            // The toggle bit is bit 4 of byte 0.
            let toggle = (cmd >> 4) & 0x01;
            Some(Ok(SdoRequest::UploadSegment { toggle }))
        }
        SDO_CMD_ABORT => {
            // A client-initiated abort. We don't have any state to
            // tear down (no segmented transfer), so just ignore.
            // (If we're mid segmented-upload, the spec says clear
            // state; we do that defensively.)
            SDO_UPLOAD_LEN.store(0, Ordering::Relaxed);
            SDO_UPLOAD_OFFSET.store(0, Ordering::Relaxed);
            None
        }
        _ => Some(Err(SdoAbort::InvalidCommand)),
    }
}

// ---- Response builders ----------------------------------------------

/// Build a 0x60 success response for an Initiate Download Request.
pub fn build_download_ok_response() -> Frame {
    // unwrap: 8-byte frame with arbitrary payload.
    Frame::new_standard(SDO_TX_COB_ID, &[0x60, 0, 0, 0, 0, 0, 0, 0])
        .expect("SDO response is 8 bytes, always valid")
}

/// Build an Initiate Upload Response for `value`. Picks
/// expedited vs segmented transfer based on `value.len`:
///
///   len ≤ 4: expedited (0x8F/0x8E/0x8D/0x8C for 1/2/3/4 bytes)
///   len ≥ 5: segmented — emit 0x82 with size, stash bytes for
///            the subsequent Upload Segment requests.
pub fn build_upload_response(index: u16, sub: u8, value: OdValue) -> Frame {
    let index_bytes = index.to_le_bytes();
    let mut payload = [0u8; 8];
    payload[1] = index_bytes[0];
    payload[2] = index_bytes[1];
    payload[3] = sub;
    if value.len as usize <= EXPEDITED_MAX {
        // Expedited. Per CiA 301 § 7.2.4.3.4, the command
        // specifier for an Initiate Upload *Response* is
        //   byte 0 = scs (0b100 = 0x80 base)
        //           | e (bit 3, must be 1 for expedited)
        //           | s (bit 2, must be 1 for size indicated)
        //           | n (bits 0-1, (4 - len))
        //
        // Earlier versions of this file had the scs bits set to
        // 0b010 (the Initiate Upload *Request* command specifier
        // — 0x4F/0x4B/0x47/0x43), which no master would accept
        // because the spec puts scs=0b010 in the request
        // direction, not the response direction.
        let cmd = match value.len {
            1 => 0x8F, // e=1, s=1, n=3 → 1 byte
            2 => 0x8E, // e=1, s=1, n=2 → 2 bytes
            3 => 0x8D, // e=1, s=1, n=1 → 3 bytes
            4 => 0x8C, // e=1, s=1, n=0 → 4 bytes
            _ => unreachable!("handled above"),
        };
        payload[0] = cmd;
        let n = value.len as usize;
        for i in 0..n {
            payload[4 + i] = value.bytes[i];
        }
    } else {
        // Segmented Initiate. Per CiA 301 § 7.2.4.3.5:
        //   byte 0 = 0x82               (scs=0b100, e=0, s=1, n=0)
        //   bytes 1-3 = index, sub
        //   bytes 4-5 = total size (LE u16)
        //   bytes 6-7 = unused          (data arrives in segments)
        payload[0] = 0x82;
        payload[4] = value.len;
        payload[5] = 0;
        payload[6] = 0;
        payload[7] = 0;
        // Stash for the next Upload Segment requests.
        segmented_upload_begin(&value.bytes[..value.len as usize]);
    }
    Frame::new_standard(SDO_TX_COB_ID, &payload)
        .expect("SDO response is 8 bytes, always valid")
}

/// Build an Upload Segment Response carrying the next 7 bytes
/// of the in-flight segmented upload. `toggle` is the toggle
/// bit from the client's Upload Segment Request (echoed back per
/// spec).
pub fn build_upload_segment_response(toggle: u8) -> Frame {
    let mut payload = [0u8; 8];
    match segmented_upload_next() {
        Some((seg, chunk, last)) => {
            // byte 0 = 0xA0 | (toggle<<4) | (n<<1) | c
            // where n = 7 - chunk (number of bytes NOT used) and
            // c = 1 if last segment, 0 otherwise.
            let n = (7 - chunk) as u8;
            payload[0] = 0xA0 | ((toggle & 0x01) << 4) | (n << 1) | (last as u8);
            for i in 0..chunk {
                payload[1 + i] = seg[i];
            }
        }
        None => {
            // No upload in progress. Spec says abort (0x80 with
            // code 0x0504_0001). We use index/sub = 0/0 because
            // the segment request doesn't carry them.
            return build_abort_response(0, 0, SdoAbort::InvalidCommand);
        }
    }
    Frame::new_standard(SDO_TX_COB_ID, &payload)
        .expect("SDO segment response is 8 bytes, always valid")
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
                        index, sub, value.len
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
                        index, sub, value.len
                    );
                    Some(build_upload_response(index, sub, value))
                }
                Err(abort) => {
                    warn!("SDO: read 0x{:04x}:{} abort 0x{:08x}", index, sub, abort.code());
                    Some(build_abort_response(index, sub, abort))
                }
            }
        }
        SdoRequest::UploadSegment { toggle } => {
            // The toggle bit in the request must match what we
            // were expecting. Per CiA 301, a mismatch aborts the
            // transfer; the spec calls for the abort code
            // 0x0504_0001 (InvalidCommand). v1 just checks and
            // aborts on mismatch — the master has to restart the
            // whole Upload Initiate.
            let expected = SDO_UPLOAD_TOGGLE.load(Ordering::Relaxed);
            if toggle != expected {
                warn!(
                    "SDO: upload-segment toggle {} (expected {})",
                    toggle, expected
                );
                SDO_UPLOAD_LEN.store(0, Ordering::Relaxed);
                SDO_UPLOAD_OFFSET.store(0, Ordering::Relaxed);
                return Some(build_abort_response(0, 0, SdoAbort::InvalidCommand));
            }
            defmt::info!("SDO: upload-segment toggle {}", toggle);
            Some(build_upload_segment_response(toggle))
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