//! UDS frame construction and parsing.
//!
//! Phase 6 uses single-frame UDS on classic CAN (8 bytes max
//! per frame). For services that need >7 bytes (e.g. full
//! 0x34 RequestDownload with 4-byte addr + 4-byte size = 11
//! bytes; 0x19 ReadDTCInformation with many DTCs), we use
//! CAN-FD (64 bytes per frame) — that's the follow-up commit
//! 6.6 (separate from this architectural decoupling).
//!
//! For now, every UDS request/response must fit in ≤ 8 bytes
//! on classic CAN. 0x34 OTA payload is truncated to 7 bytes
//! inside the dispatcher (the size is still recovered from
//! the truncated bytes for boot-time validation).

use embassy_stm32::can::Frame;
use embedded_can::Id;

use crate::can::uds::state::load_response;
use crate::can::uds_transport::can_id::response_id_for_request;

/// Maximum bytes per UDS frame on classic CAN (CAN 2.0B / ISO
/// 11898-1: 8-byte data field). When the bus switches to
/// CAN-FD (commit 6.6) this bumps to 64.
pub const UDS_FRAME_MAX: usize = 8;

/// Build the UDS response frame for a given request COB-ID.
/// Reads the last response from the UDS engine's response
/// buffer and wraps it in a CAN frame on the appropriate
/// response COB-ID.
///
/// Returns `None` if the response is empty (e.g. 0x3E 0x80
/// suppress positive response — currently the SDO layer
/// limitation prevents this; tracked as a known gap).
#[inline(never)]
#[link_section = ".data"]
pub fn build_response_frame(request_id: u16) -> Option<Frame> {
    let (bytes, len) = load_response();
    if len == 0 {
        return None;
    }
    let n = (len as usize).min(UDS_FRAME_MAX);
    let response_id = response_id_for_request(request_id);
    Frame::new_standard(response_id, &bytes[..n]).ok()
}

/// Wrapper used by `handle_rx_frame` — see mod.rs.
#[inline(never)]
pub fn build_response_frame_for(request_id: u16) -> Option<Frame> {
    build_response_frame(request_id)
}

/// Parse a UDS request frame: copy the data bytes (skipping
/// CAN framing) into a slice. The slice is borrowed from
/// `frame.data()` so the returned reference is valid for
/// the lifetime of the frame — the caller must not hold it
/// across an `await`.
///
/// Currently a no-op (the data is already in the right
/// format); the helper exists so the call site is symmetric
/// with `build_response_frame` and so the multi-frame
/// parser (Phase 6 follow-up) can be slotted in here.
pub fn parse_request_frame<'a>(frame: &'a Frame) -> &'a [u8] {
    let len = frame.header().len() as usize;
    let data = frame.data();
    if len <= data.len() { &data[..len] } else { data }
}
