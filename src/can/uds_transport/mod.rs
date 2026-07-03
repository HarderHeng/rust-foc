//! UDS transport layer — independent of CANopen.
//!
//! Phase 6 (decoupling) replaces the legacy "UDS over CANopen
//! SDO 0x2F00.0" tunnel with a direct UDS frame handler on
//! FDCAN1. This module owns:
//!
//! - CAN-ID routing (functional / physical request, physical
//!   response)
//! - Single-frame UDS dispatch (1 CAN frame per request/response)
//! - Bridge between raw CAN frames and the UDS dispatcher in
//!   `src/can/uds/`
//!
//! ## CAN-ID layout (ISO 14229-3 §7)
//!
//! | Direction  | COB-ID  | Notes |
//! |------------|---------|-------|
//! | Functional request  | 0x7DF | broadcast, all ECUs |
//! | Physical request   | 0x7E0 | our ECU address (1) |
//! | Physical response  | 0x7E8 | 0x7E0 + 8 (per ISO 14229-3) |
//!
//! `uds_transport::handle_rx_frame` is called from
//! `canopen_task` for every received CAN frame whose ID falls in
//! the UDS range. It builds a UDS request slice, dispatches to
//! the UDS engine, and (on Ready) sends the response frame
//! back on the response COB-ID.

pub mod can_id;
pub mod frame;

use embassy_stm32::can::Frame;
use embedded_can::Id;

use super::uds;
use frame::build_response_frame;

/// Functional request COB-ID (broadcast, all ECUs).
pub const COB_ID_FUNCTIONAL_REQUEST: u16 = 0x7DF;

/// Physical request COB-ID (our ECU address = 1).
pub const COB_ID_PHYSICAL_REQUEST: u16 = 0x7E0;

/// Physical response COB-ID (= request + 8, per ISO 14229-3 §7).
pub const COB_ID_PHYSICAL_RESPONSE: u16 = 0x7E8;

/// True iff the given frame is addressed to one of our UDS
/// request COB-IDs (functional 0x7DF or physical 0x7E0).
/// `canopen_task` uses this to route RX frames.
pub fn is_uds_frame(frame: &Frame) -> bool {
    let id = match frame.header().id() {
        Id::Standard(s) => s.as_raw(),
        Id::Extended(_) => return false,
    };
    can_id::is_uds_request_id(id)
}

/// Handle one received UDS request frame. The frame must pass
/// `is_uds_frame` (checked by the caller). Builds the UDS
/// request slice, runs the UDS dispatcher, and on
/// `DispatchResult::Ready` sends the response frame back.
///
/// **Lives in `.data` (RAM).** Called from `canopen_task` for
/// every UDS request. Keeping the entire UDS receive path
/// off the OTA write path is the same rationale as Phase 4
/// (SDO dispatch was RAM-resident for OTA safety).
#[inline(never)]
#[link_section = ".data"]
pub fn handle_rx_frame(frame: &Frame) -> Option<Frame> {
    let id = match frame.header().id() {
        Id::Standard(s) => s.as_raw(),
        Id::Extended(_) => return None,
    };
    if !can_id::is_uds_request_id(id) {
        return None;
    }
    let request = frame::parse_request_frame(frame);
    if request.is_empty() {
        return None;
    }
    uds::dispatch(request);
    build_response_frame(id)
}
