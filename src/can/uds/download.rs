//! 0x34/0x36/0x37 RequestDownload/TransferData/TransferExit
//! glue. The actual flash work lives in `super::super::ota`;
//! this module just routes the request and dispatches the
//! response.
//!
//! Phase 7: these handlers push the work onto the pending
//! queue. Each handler returns a closure (a fn pointer) that
//! reads the in-flight request bytes from `UdsContext.config.
//! request_buf` (set at the start of `dispatch`) and calls
//! the synchronous OTA implementation. The continuation
//! runs in `pending::tick`, and `tick` pushes a 0x78
//! ResponsePending every `p2_server_ms` until the work
//! completes.
//!
//! Closure model: `fn` pointer (no env capture) to avoid a
//! global allocator. The request bytes are already in
//! `config.request_buf[0..state.request_len]` — the closure
//! reads from there.

use super::nrc::Nrc;
use super::pending::{push_pending, PendingFn, UdsContext};
use super::state::{store_response, UdsState};
use crate::can::ota;

/// Try to push 0x34 RequestDownload onto the pending queue.
/// Returns `true` if the closure was queued (caller returns
/// `DispatchResult::Pending`), `false` if the request was
/// rejected (response already written to response_buf, caller
/// returns `DispatchResult::Ready`).
pub fn try_queue_request(state: &mut UdsState, config: &mut super::config::UdsConfig) -> bool {
    let req = &state.request_buf[..state.request_len];
    if req.len() != 6 || req[1] != 0x00 {
        store_response(&Nrc::RequestOutOfRange.negative_response(0x34));
        return false;
    }
    // Pre-check OTA state synchronously (cheap) so we can
    // emit a negative response without burning a pending
    // slot. The actual erase runs in the closure.
    push_pending(state, config, pending_erase as PendingFn)
}

fn pending_erase(ctx: &mut UdsContext) {
    // Read the 5 bytes after the 0x34 SID from the shared
    // request buffer (set at the start of `dispatch`).
    let req = &ctx.state.request_buf[..ctx.state.request_len];
    if req.len() != 6 || req[1] != 0x00 {
        store_response(&Nrc::RequestOutOfRange.negative_response(0x34));
        ctx.complete = true;
        return;
    }
    ota::handle_request_download(&req[1..]);
    ctx.complete = true;
}

/// Try to push 0x36 TransferData onto the pending queue.
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
    if req.len() != 4 {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x36));
        ctx.complete = true;
        return;
    }
    ota::handle_transfer_data(&req[1..]);
    ctx.complete = true;
}

/// Try to push 0x37 RequestTransferExit onto the pending queue.
pub fn try_queue_exit(state: &mut UdsState, config: &mut super::config::UdsConfig) -> bool {
    let req = &state.request_buf[..state.request_len];
    if !req.is_empty() {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x37));
        return false;
    }
    push_pending(state, config, pending_exit as PendingFn)
}

fn pending_exit(ctx: &mut UdsContext) {
    ota::handle_transfer_exit(&[]);
    ctx.complete = true;
}
