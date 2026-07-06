//! Pending queue + 0x78 ResponsePending machinery.
//!
//! ## Components
//!
//! - [`UdsContext`]: passed to pending closures; `complete` flag
//! - [`push_pending`]: register a continuation function
//! - [`tick`]: drain → process → put-back (avoids the borrow
//!   conflict between `config.pending_queue` mut borrow and the
//!   `config` shared borrow inside `UdsContext`)

use crate::state::{store_response, UdsState};
use crate::table::UdsConfig;
use crate::types::Nrc;
use crate::SrvState;

/// Maximum number of pending jobs in the queue. 4 covers the OTA
/// flow (TransferData + TransferExit + 2 waiting).
pub const PENDING_QUEUE_SIZE: usize = 4;

/// Per-continuation context. The pending closure sets
/// `complete = true` when it's done writing the response.
pub struct UdsContext<'a> {
    pub state: &'a mut UdsState,
    #[allow(dead_code)]
    pub config: &'a UdsConfig,
    pub complete: bool,
}

/// A queued continuation. A `fn` pointer (no environment capture)
/// to avoid needing a global allocator.
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
            state.state = SrvState::Pending;
            return true;
        }
    }
    false
}

/// Drain → process → put-back. Avoids the borrow conflict
/// between `config.pending_queue` (mut) and the `UdsContext`
/// (which shared-borrows `config`).
///
/// Also handles P2 timeout: if the request has been Pending
/// for longer than `config.p2_server_ms`, push a 0x78 frame
/// and bump the timestamp so we don't send one every tick.
#[inline(never)]
pub fn tick(state: &mut UdsState, config: &mut UdsConfig, now_ms: u32) {
    if state.state != SrvState::Pending {
        return;
    }

    // 1. drain
    let mut jobs: [Option<PendingJob>; PENDING_QUEUE_SIZE] =
        core::array::from_fn(|_| None);
    for (i, slot) in config.pending_queue.iter_mut().enumerate() {
        jobs[i] = slot.take();
    }

    // 2. process — every job runs once
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
            state.state = SrvState::Idle;
        }
    }

    // 5. P2 timeout: push 0x78 if still pending and >P2 elapsed
    if state.state == SrvState::Pending {
        if now_ms.saturating_sub(state.request_tick_ms) >= config.p2_server_ms {
            send_response_pending(state);
            state.request_tick_ms = now_ms;
        }
    }
}

/// Push a 0x78 ResponsePending frame into the response buffer.
fn send_response_pending(state: &mut UdsState) {
    if state.request_len == 0 {
        return;
    }
    let sid = state.request_buf[0];
    let bytes = Nrc::RequestCorrectlyReceivedResponsePending
        .negative_response(sid);
    store_response(&bytes);
}
