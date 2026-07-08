//! UDS ↔ CAN-FD bridge — platform-specific transport adapter.
//!
//! This file is the **only** place in the firmware that knows about
//! CAN-FD frame types. It lives in `src/can/` (the bus layer), NOT
//! in `src/uds/` (the application layer), because the UDS library
//! must be a pure application-layer library with zero hardware
//! dependencies.
//!
//! ## CAN-ID layout (ISO 14229-3 §7)
//!
//! | Direction  | COB-ID  | Notes |
//! |------------|---------|-------|
//! | Functional request  | 0x7DF | broadcast, all ECUs |
//! | Physical request   | 0x7E0 | our ECU address (1) |
//! | Physical response  | 0x7E8 | 0x7E0 + 8 (per ISO 14229-3) |
//!
//! ## Porting to a different transport
//!
//! Replace this file entirely. The UDS library exposes a clean API:
//!
//! - `crate::uds::dispatch(request)` — process a request, store response
//! - `crate::uds::load_response()` — read the response bytes
//! - `crate::uds::tick(now_ms)` — drive the pending queue
//! - `crate::uds::tx_disabled()` — check if TX is suppressed

use embassy_stm32::can::frame::FdFrame;
use embedded_can::Id;

use crate::uds;
use uds_core::state::load_response;

// ============================================================================
// CAN-ID routing
// ============================================================================

/// Functional request COB-ID (broadcast, all ECUs).
pub const COB_ID_FUNCTIONAL_REQUEST: u16 = 0x7DF;

/// Physical request COB-ID (our ECU address = 1).
pub const COB_ID_PHYSICAL_REQUEST: u16 = 0x7E0;

/// Physical response COB-ID (= request + 8, per ISO 14229-3 §7).
pub const COB_ID_PHYSICAL_RESPONSE: u16 = 0x7E8;

/// SIDs that must not be accepted via functional (0x7DF) addressing.
/// ISO 14229-1 §6.3: SecurityAccess (0x27) is a point-to-point service;
/// accepting it on a broadcast COB-ID would let any node on the bus
/// trigger seed/key exchanges intended for a single ECU.
const FUNCTIONAL_BLOCKED_SIDS: &[u8] = &[0x27];

/// True iff `id` is a UDS request we should handle (either
/// functional broadcast or our physical address).
fn is_uds_request_id(id: u16) -> bool {
    id == COB_ID_FUNCTIONAL_REQUEST || id == COB_ID_PHYSICAL_REQUEST
}

/// Map a request COB-ID to the response COB-ID the ECU should
/// reply on. Both functional and physical requests respond on
/// the physical response ID.
fn response_id_for_request(_request_id: u16) -> u16 {
    COB_ID_PHYSICAL_RESPONSE
}

/// UDS transport adapter. `DefaultUdsTransport` is the
/// production implementation over real CAN-FD frames; the
/// `UdsTransport` trait lets tests substitute a mock without
/// driving the FDCAN hardware.
pub trait UdsTransport {
    /// True iff the given frame is addressed to one of our UDS
    /// request COB-IDs (functional 0x7DF or physical 0x7E0).
    fn is_uds_frame(&self, frame: &FdFrame) -> bool;

    /// Handle one received UDS request frame. The frame must
    /// pass `is_uds_frame` (checked by the caller). Builds the
    /// UDS request slice, runs the dispatcher, and returns the
    /// response frame.
    fn handle_rx_frame(&self, frame: &FdFrame) -> Option<FdFrame>;
}

pub struct DefaultUdsTransport;

impl UdsTransport for DefaultUdsTransport {
    fn is_uds_frame(&self, frame: &FdFrame) -> bool {
        let id = match frame.header().id() {
            Id::Standard(s) => s.as_raw(),
            Id::Extended(_) => return false,
        };
        is_uds_request_id(id)
    }

    fn handle_rx_frame(&self, frame: &FdFrame) -> Option<FdFrame> {
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
        // ISO 14229-1 §6.3: reject security-sensitive SIDs on
        // functional (0x7DF) addressing. These are point-to-point
        // services that must not be broadcast.
        if id == COB_ID_FUNCTIONAL_REQUEST
            && !request.is_empty()
            && FUNCTIONAL_BLOCKED_SIDS.contains(&request[0])
        {
            return None;
        }
        uds::dispatch(request);
        build_response_frame(id)
    }
}

// ============================================================================
// Frame construction / parsing
// ============================================================================

/// Maximum bytes per UDS frame. CAN-FD is 64 bytes.
const UDS_FRAME_MAX: usize = 64;

/// Build the UDS response frame for a given request COB-ID.
/// Reads the last response from the UDS engine and wraps it in
/// a CAN-FD frame on the appropriate response COB-ID.
///
/// Returns `None` if the response is empty (e.g. 0x3E 0x80
/// suppress positive response).
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
