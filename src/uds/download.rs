//! 0x34/0x36/0x37 RequestDownload/TransferData/TransferExit
//! glue. The flash work lives in `crate::ota`; this
//! module validates the request and pushes the work onto the
//! pending queue (Phase 7). The continuation runs in
//! `pending::tick`, and `tick` pushes a 0x78 ResponsePending
//! every `p2_server_ms` until the work completes.
//!
//! Closure model: `fn` pointer (no env capture) to avoid a
//! global allocator. The request bytes are in
//! `state.request_buf[0..state.request_len]` (set at the
//! start of `dispatch`); the closure reads from there.

use super::nrc::Nrc;
use super::pending::{push_pending, PendingFn, UdsContext};
use super::state::{store_response, UdsState};
use crate::ota;

pub fn try_queue_request(state: &mut UdsState, config: &mut super::config::UdsConfig) -> bool {
    let req = &state.request_buf[..state.request_len];
    if req.len() != 6 || req[1] != 0x00 {
        store_response(&Nrc::RequestOutOfRange.negative_response(0x34));
        return false;
    }
    push_pending(state, config, pending_erase as PendingFn)
}

fn pending_erase(ctx: &mut UdsContext) {
    let req = &ctx.state.request_buf[..ctx.state.request_len];
    ota::handle_request_download(&req[1..]);
    ctx.complete = true;
}

pub fn try_queue_transfer(state: &mut UdsState, config: &mut super::config::UdsConfig) -> bool {
    let req = &state.request_buf[..state.request_len];
    if req.len() != 4 {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x36));
        return false;
    }
    push_pending(state, config, pending_transfer as PendingFn)
}

fn pending_transfer(ctx: &mut UdsContext) {
    let req = &ctx.state.request_buf[..ctx.state.request_len];
    ota::handle_transfer_data(&req[1..]);
    ctx.complete = true;
}

pub fn try_queue_exit(state: &mut UdsState, config: &mut super::config::UdsConfig) -> bool {
    let req = &state.request_buf[..state.request_len];
    if !req.is_empty() {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x37));
        return false;
    }
    push_pending(state, config, pending_exit as PendingFn)
}

fn pending_exit(_ctx: &mut UdsContext) {
    ota::handle_transfer_exit(&[]);
    _ctx.complete = true;
}
