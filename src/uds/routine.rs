//! 0x31 RoutineControl handler.
//!
//! Wire format:
//!   [0x31, subfunc, rid_hi, rid_lo, ...payload] →
//!     [0x71, subfunc, rid_hi, rid_lo, ...result]
//!
//! Subfuncs:
//!   0x01 startRoutine
//!   0x02 stopRoutine
//!   0x03 requestRoutineResults
//!
//! Each subfunc looks up the corresponding table in
//! `UdsConfig::routines_{start,stop,result}` and invokes the
//! registered `RoutineEntry::func`. Tables are independent so
//! a single RID can have different start / stop / result
//! behaviour.
//!
//! Phase 5b registers two example routines (stub callbacks;
//! real OTA wiring is Phase 5c):
//!   - 0xFF00: erase (start only; stop / result are noop)
//!   - 0xF001: checkProgrammingDependencies (result only)

use defmt::info;

use super::config::UdsConfig;
use super::nrc::Nrc;
use super::state::{store_response, UdsState};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RoutineSub {
    Start,
    Stop,
    Result,
}

pub fn handle(state: &mut UdsState, config: &UdsConfig, req: &[u8], sub: RoutineSub) {
    if req.len() < 4 {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x31));
        return;
    }
    let subfunc = req[1];
    let rid = u16::from_be_bytes([req[2], req[3]]);

    let table: &[super::config::RoutineEntry] = match sub {
        RoutineSub::Start => config.routines_start,
        RoutineSub::Stop => config.routines_stop,
        RoutineSub::Result => config.routines_result,
    };

    let entry = match table.iter().find(|e| e.rid == rid) {
        Some(e) => e,
        None => {
            store_response(&Nrc::RequestOutOfRange.negative_response(0x31));
            return;
        }
    };

    if !super::config::session_allowed(state.session, entry.session_access) {
        store_response(&Nrc::SubFunctionNotSupportedInActiveSession
            .negative_response(0x31));
        return;
    }
    if !super::config::security_allowed(state.security, entry.security_level) {
        store_response(&Nrc::SecurityAccessDenied.negative_response(0x31));
        return;
    }

    let payload = &req[4..];
    let mut resp_buf = [0u8; 8];
    match (entry.func)(payload, &mut resp_buf) {
        Ok(resp_len) => {
            let mut out = [0u8; 12];
            let total = 4 + resp_len.min(8);
            out[0] = 0x71;
            out[1] = subfunc;
            out[2] = req[2];
            out[3] = req[3];
            for i in 0..resp_len.min(8) {
                out[4 + i] = resp_buf[i];
            }
            info!("UDS: Routine 0x{:04x} sub=0x{:02x} OK ({} bytes result)",
                  rid, subfunc, resp_len);
            store_response(&out[..total]);
        }
        Err(nrc) => {
            store_response(&nrc.negative_response(0x31));
        }
    }
}
