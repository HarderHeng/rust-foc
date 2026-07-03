//! CAN-ID routing for UDS.
//!
//! Per ISO 14229-3 §7:
//! - 0x7DF = functional request (broadcast, all ECUs)
//! - 0x7E0..0x7E7 = physical request (per ECU address)
//! - 0x7E8..0x7EF = physical response (= request + 8)
//!
//! Phase 6 hardcodes ECU address = 1 (matches Phase 1-5
//! `canopen::NODE_ID`).

/// Functional request COB-ID (broadcast, all ECUs).
pub const COB_ID_FUNCTIONAL_REQUEST: u16 = 0x7DF;

/// Physical request COB-ID (our ECU address = 1).
pub const COB_ID_PHYSICAL_REQUEST: u16 = 0x7E0;

/// Physical response COB-ID (= request + 8, per ISO 14229-3 §7).
pub const COB_ID_PHYSICAL_RESPONSE: u16 = 0x7E8;

/// True iff `id` is a UDS request we should handle (either
/// functional broadcast or our physical address).
pub fn is_uds_request_id(id: u16) -> bool {
    id == COB_ID_FUNCTIONAL_REQUEST || id == COB_ID_PHYSICAL_REQUEST
}

/// Map a request COB-ID to the response COB-ID the ECU should
/// reply on. Functional requests get the physical response
/// ID (each responding ECU replies individually). Physical
/// requests get the matching physical response.
pub fn response_id_for_request(request_id: u16) -> u16 {
    // Per ISO 14229-3 §7: response COB-ID = request + 8.
    // Both 0x7DF and 0x7E0 are 11-bit IDs, so the +8 wrapping
    // is well-defined (0x7E7 / 0x7E8 respectively).
    if request_id == COB_ID_FUNCTIONAL_REQUEST {
        COB_ID_PHYSICAL_RESPONSE
    } else if request_id == COB_ID_PHYSICAL_REQUEST {
        COB_ID_PHYSICAL_RESPONSE
    } else {
        COB_ID_PHYSICAL_RESPONSE  // safe default
    }
}
