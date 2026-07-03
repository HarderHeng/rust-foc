//! Pending queue + 0x78 ResponsePending machinery.
//!
//! Phase 5c scope: build the infrastructure (PendingJob, dispatch
//! return value, take_response) but **do not rewire OTA to use
//! it** — that's a separate, risky change to the OTA hot path.
//! Phase 6 (next) will rewire `download::handle_*` to push
//! pending closures.
//!
//! ## Components
//!
//! - [`DispatchResult`]: what `dispatch` returns
//! - [`UdsContext`]: passed to pending closures; `complete` flag
//! - [`push_pending`]: register a continuation function
//! - [`tick`]: drain → process → put-back (avoids the borrow
//!   conflict between `config.pending_queue` mut borrow and the
//!   `config` shared borrow inside `UdsContext`)
//! - [`take_response`]: caller reads the response after `dispatch`
//!   or after `tick` pushes one
//! - [`send_response_pending`]: push a 0x78 frame into the response
//!   buffer without touching state machine
//!
//! ## Phase 5c simplification
//!
//! PendingJob uses `fn` pointers (no `Box<dyn FnMut>`) so the
//! firmware doesn't need a global allocator. The implication:
//! continuations can't capture environment. Phase 6 (OTA rewire)
//! will need captured bytes — the design doc proposes either a
//! heap allocator (risky on the OTA hot path) or a "captured
//! bytes" slot per PendingJob. For now the queue is
//! infrastructure-only; no continuations are pushed.

use super::config::UdsConfig;
use super::nrc::Nrc;
use super::state::{load_request, load_response, store_response, UdsState};

/// Maximum number of pending jobs in the queue. 4 covers the OTA
/// flow (TransferData + TransferExit + 2 waiting).
pub const PENDING_QUEUE_SIZE: usize = 4;

/// Return value of `dispatch`.
///
/// `Ready` — response is ready in `state.response_buf`,
/// caller should call `take_response`.
///
/// `Pending` — long task is queued; response is *not* ready
/// (either a 0x78 was just sent, or the task hasn't finished
/// yet). Caller should not read `last_response` and should
/// wait for the next tick to push 0x78 / final response.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DispatchResult {
    Ready,
    Pending,
}

/// Per-continuation context. The pending closure sets
/// `complete = true` when it's done writing the response.
pub struct UdsContext<'a> {
    pub state: &'a mut UdsState,
    pub config: &'a UdsConfig,
    pub complete: bool,
}

/// A queued continuation. Phase 5c: a `fn` pointer (no
/// environment capture) to avoid needing a global allocator.
/// Phase 6 may swap to a `&mut dyn FnMut` or
/// `Box<dyn FnMut + captured_slot>`.
pub type PendingFn = fn(&mut UdsContext);

pub struct PendingJob {
    pub func: PendingFn,
}

impl PendingJob {
    pub const fn new(f: PendingFn) -> Self {
        Self { func: f }
    }
}

/// Push a continuation onto the pending queue. Returns `false`
/// if the queue is full (caller should respond with 0x22).
///
/// Sets `state.state = SrvState::Pending` so subsequent
/// `dispatch` calls return `Pending` until the queue drains.
pub fn push_pending(state: &mut UdsState, config: &mut UdsConfig, f: PendingFn) -> bool {
    for slot in config.pending_queue.iter_mut() {
        if slot.is_none() {
            *slot = Some(PendingJob::new(f));
            state.state = super::state::SrvState::Pending;
            return true;
        }
    }
    false
}

/// Drain → process → put-back. Avoids the borrow conflict
/// between `config.pending_queue` (mut) and the `UdsContext`
/// (which shared-borrows `config`).
///
/// 1. drain: take all jobs out of `config.pending_queue` into
///    a local array (releasing the mut borrow).
/// 2. process: for each local job, call it with a UdsContext
///    that shared-borrows `config`. If `complete = true`,
///    mark `state.response_pending = true` and (if queue is
///    now empty) return to Idle.
/// 3. put-back: re-insert any non-complete jobs into
///    `config.pending_queue`.
///
/// Also handles P2 timeout: if the request has been Pending
/// for longer than `config.p2_server_ms`, push a 0x78 frame
/// and bump the timestamp so we don't send one every tick.
#[inline(never)]
pub fn tick(state: &mut UdsState, config: &mut UdsConfig, now_ms: u32) {
    if state.state != super::state::SrvState::Pending {
        return;
    }

    // 1. drain
    let mut jobs: [Option<PendingJob>; PENDING_QUEUE_SIZE] =
        core::array::from_fn(|_| None);
    for (i, slot) in config.pending_queue.iter_mut().enumerate() {
        jobs[i] = slot.take();
    }

    // 2. process
    //
    // Phase 5c simplification: every job gets called **once**
    // (we don't keep a non-complete job in the queue). This
    // works for the current "long flash write" model where the
    // work is one shot. Phase 6 will extend the contract to
    // support multi-tick continuations.
    let mut any_complete = false;
    for slot in jobs.iter_mut() {
        if let Some(job) = slot.as_ref() {
            let mut ctx = UdsContext { state, config: &*config, complete: false };
            (job.func)(&mut ctx);
            if ctx.complete {
                any_complete = true;
                *slot = None;
            }
        }
    }

    // 3. put-back
    for (i, dst) in config.pending_queue.iter_mut().enumerate() {
        *dst = jobs[i].take();
    }

    // 4. response_pending + state transition
    if any_complete {
        state.response_pending = true;
        let queue_empty = config.pending_queue.iter().all(|j| j.is_none());
        if queue_empty {
            state.state = super::state::SrvState::Idle;
        }
    }

    // 5. P2 timeout: push 0x78 if still pending and >P2 elapsed
    if state.state == super::state::SrvState::Pending {
        if now_ms.saturating_sub(state.request_tick_ms) >= config.p2_server_ms {
            send_response_pending(state, config);
            state.request_tick_ms = now_ms;
        }
    }
}

/// Push a 0x78 ResponsePending frame into the response buffer.
/// Caller must check `state.state == SrvState::Pending` first.
pub fn send_response_pending(state: &mut UdsState, _config: &UdsConfig) {
    if state.request_len == 0 {
        return;
    }
    // 0x78 frame doesn't need the original SID at byte 1 — we
    // use the SID from the in-flight request (stored in
    // REQUEST_BUF static, see state.rs).
    let (req_bytes, req_len) = load_request();
    if req_len == 0 {
        return;
    }
    let sid = req_bytes[0];
    let bytes = Nrc::RequestCorrectlyReceivedResponsePending
        .negative_response(sid);
    store_response(&bytes);
    state.response_pending = true;
}

/// Caller (canopen_task) checks if a response is ready and
/// consumes the flag. The actual bytes are read separately via
/// `load_response()` (which returns `(bytes, len)`).
///
/// **Why this split**: `RESPONSE_BUF` is a static, and the
/// borrow checker can't tie a `&'a [u8]` returned by
/// `take_response` to the lifetime of a `&UdsState` mut
/// borrow. The canopen_task needs both — but it's already
/// calling `load_response` for the wire bytes, so we just
/// need `take_response` to clear the flag.
///
/// **Caller protocol** (matches design doc §4.13):
/// 1. `dispatch()` or `tick()` sets `response_pending = true`
///    and writes bytes to `RESPONSE_BUF`.
/// 2. Caller (canopen_task) calls `take_response()`. If
///    `Some(len)`, read `RESPONSE_BUF` via `load_response()`
///    and send the SDO read response.
/// 3. If `None`, no response yet — caller should wait for
///    the next tick to push one.
pub fn take_response(state: &mut UdsState) -> Option<u8> {
    if !state.response_pending {
        return None;
    }
    state.response_pending = false;
    let (_bytes, len) = load_response();
    if len == 0 {
        return None;
    }
    Some(len)
}
