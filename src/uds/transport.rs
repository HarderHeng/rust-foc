//! UDS transport layer — independent of CANopen.
//!
//! Phase 6 (decoupling) replaced the legacy "UDS over CANopen
//! SDO 0x2F00.0" tunnel with a direct UDS frame handler on
//! FDCAN1. This module owns:
//!
//! - CAN-ID routing (functional / physical request, physical
//!   response)
//! - Single-frame UDS dispatch (1 CAN frame per request/response)
//! - Bridge between raw CAN frames and the UDS dispatcher in
//!   `super`
//!
//! ## CAN-ID layout (ISO 14229-3 §7)
//!
//! | Direction  | COB-ID  | Notes |
//! |------------|---------|-------|
//! | Functional request  | 0x7DF | broadcast, all ECUs |
//! | Physical request   | 0x7E0 | our ECU address (1) |
//! | Physical response  | 0x7E8 | 0x7E0 + 8 (per ISO 14229-3) |
//!
//! `transport::handle_rx_frame` is called from `canopen_task`
//! for every received CAN frame whose ID falls in the UDS
//! range. It builds a UDS request slice, dispatches to the
//! UDS engine, and (on Ready) returns the response frame for
//! `canopen_task` to TX.

use embassy_stm32::can::frame::FdFrame;
use embedded_can::Id;

use crate::uds;
use super::state::load_response;

// ============================================================================
// CAN-ID routing
// ============================================================================

/// Functional request COB-ID (broadcast, all ECUs).
pub const COB_ID_FUNCTIONAL_REQUEST: u16 = 0x7DF;

/// Physical request COB-ID (our ECU address = 1).
pub const COB_ID_PHYSICAL_REQUEST: u16 = 0x7E0;

/// Physical response COB-ID (= request + 8, per ISO 14229-3 §7).
pub const COB_ID_PHYSICAL_RESPONSE: u16 = 0x7E8;

/// True iff `id` is a UDS request we should handle (either
/// functional broadcast or our physical address).
fn is_uds_request_id(id: u16) -> bool {
    id == COB_ID_FUNCTIONAL_REQUEST || id == COB_ID_PHYSICAL_REQUEST
}

/// Map a request COB-ID to the response COB-ID the ECU
/// should reply on. Both functional and physical requests
/// get the physical response ID (each responding ECU
/// replies individually).
fn response_id_for_request(_request_id: u16) -> u16 {
    COB_ID_PHYSICAL_RESPONSE
}

/// True iff the given frame is addressed to one of our UDS
/// request COB-IDs (functional 0x7DF or physical 0x7E0).
/// `canopen_task` uses this to route RX frames.
pub fn is_uds_frame(frame: &FdFrame) -> bool {
    let id = match frame.header().id() {
        Id::Standard(s) => s.as_raw(),
        Id::Extended(_) => return false,
    };
    is_uds_request_id(id)
}

// ============================================================================
// Frame construction / parsing
// ============================================================================

/// Maximum bytes per UDS frame. CAN-FD is 64 bytes (vs 8 on
/// classic CAN). Phase 6 commit 2 uses CAN-FD exclusively
/// for UDS.
const UDS_FRAME_MAX: usize = 64;

/// Build the UDS response frame for a given request COB-ID.
/// Reads the last response from the UDS engine's response
/// buffer and wraps it in a CAN-FD frame on the appropriate
/// response COB-ID.
///
/// Returns `None` if the response is empty (e.g. 0x3E 0x80
/// suppress positive response).
#[inline(never)]
#[link_section = ".data"]
fn build_response_frame(request_id: u16) -> Option<FdFrame> {
    let (bytes, len) = load_response();
    if len == 0 {
        return None;
    }
    let n = (len as usize).min(UDS_FRAME_MAX);
    let response_id = response_id_for_request(request_id);
    FdFrame::new_standard(response_id, &bytes[..n]).ok()
}

/// Parse a UDS request frame: borrow the data bytes. The
/// returned slice is valid for the lifetime of the frame.
fn parse_request_frame<'a>(frame: &'a FdFrame) -> &'a [u8] {
    let len = frame.header().len() as usize;
    let data = frame.data();
    if len <= data.len() { &data[..len] } else { data }
}

// ============================================================================
// Main entry point called from canopen_task
// ============================================================================

/// Handle one received UDS request frame (Phase 6 commit 2:
/// CAN-FD, 64-byte max). The frame must pass `is_uds_frame`
/// (checked by the caller). Builds the UDS request slice,
/// runs the UDS dispatcher, and returns the response frame.
///
/// **Lives in `.data` (RAM).** Called from `canopen_task` for
/// every UDS request. Keeping the entire UDS receive path
/// off the OTA write path is the same rationale as Phase 4.
#[inline(never)]
#[link_section = ".data"]
pub fn handle_rx_frame(frame: &FdFrame) -> Option<FdFrame> {
    let id = match frame.header().id() {
        Id::Standard(s) => s.as_raw(),
        Id::Extended(_) => return None,
    };
    if !is_uds_request_id(id) {
        return None;
    }
    let request = parse_request_frame(frame);
    if request.is_empty() {
        return None;
    }
    uds::dispatch(request);
    build_response_frame(id)
}
