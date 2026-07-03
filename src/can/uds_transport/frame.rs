//! UDS frame construction and parsing.
//!
//! Phase 6 commit 2: UDS frames use CAN-FD (up to 64 bytes
//! per frame) so a single frame covers even the long UDS
//! services (0x34 RequestDownload = 11 bytes; 0x19 ReadDTC
//! with many DTCs can return >8 bytes). NMT + heartbeat
//! stay on classic CAN (1 byte payload, no need to upgrade).

use embassy_stm32::can::frame::FdFrame;
use embedded_can::Id;

use crate::can::uds::state::load_response;
use crate::can::uds_transport::can_id::response_id_for_request;

/// Maximum bytes per UDS frame. CAN-FD is 64 bytes (vs 8 on
/// classic CAN). Phase 6 commit 2 uses CAN-FD exclusively
/// for UDS; this constant documents the bus-mode limit.
pub const UDS_FRAME_MAX: usize = 64;

/// Build the UDS response frame for a given request COB-ID.
/// Reads the last response from the UDS engine's response
/// buffer and wraps it in a CAN-FD frame on the appropriate
/// response COB-ID.
///
/// Returns `None` if the response is empty (e.g. 0x3E 0x80
/// suppress positive response — currently the SDO layer
/// limitation prevents this; tracked as a known gap).
#[inline(never)]
#[link_section = ".data"]
pub fn build_response_frame(request_id: u16) -> Option<FdFrame> {
    let (bytes, len) = load_response();
    if len == 0 {
        return None;
    }
    let n = (len as usize).min(UDS_FRAME_MAX);
    let response_id = response_id_for_request(request_id);
    FdFrame::new_standard(response_id, &bytes[..n]).ok()
}

/// Parse a UDS request frame: copy the data bytes (skipping
/// CAN framing) into a slice. The slice is borrowed from
/// `frame.data()` so the returned reference is valid for
/// the lifetime of the frame — the caller must not hold it
/// across an `await`.
pub fn parse_request_frame<'a>(frame: &'a FdFrame) -> &'a [u8] {
    let len = frame.header().len() as usize;
    let data = frame.data();
    if len <= data.len() { &data[..len] } else { data }
}
