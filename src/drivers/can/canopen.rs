//! Minimal CANopen — NMT state machine + heartbeat producer.
//!
//! ## Fault detection (DTC)
//!
//! This task monitors CAN bus communication health and records DTCs
//! via `crate::uds::dtc::set_dtc`:
//!
//! - `0x030100` — CAN communication timeout (>10 s with no frame).
//!
//! Phase 1 only ships the *bare minimum* for a node to be visible
//! on the bus: it boots up, sends a one-shot boot-up message, then
//! emits a 1 Hz heartbeat so a master can see it in `candump`. The
//! node also listens for NMT commands on COB-ID `0x000` and
//! transitions its state accordingly.
//!
//! Phase 2 will add the SDO server and the object dictionary; this
//! file deliberately doesn't grow past NMT + heartbeat until then.
//!
//! ## COB-ID summary (NodeId = 1)
//!
//! | COB-ID | Direction | Function         | Notes |
//! |--------|-----------|------------------|-------|
//! | 0x000  | master→1  | NMT command      | standard |
//! | 0x701  | 1→master  | Heartbeat / boot | 1 Hz, byte 0 = state |
//!
//! Heartbeat state byte per CiA 301:
//!   0x00 = boot-up
//!   0x04 = stopped
//!   0x05 = operational
//!   0x7F = pre-operational (default after boot-up)
//!
//! NMT command byte (the second byte in a master→1 frame on 0x000):
//!   0x01 = enter operational
//!   0x02 = stop
//!   0x80 = enter pre-operational
//!   0x81 = reset node
//!   0x82 = reset communication
//!
//! Frame format on 0x000: `[cmd, node_id]`. A master that broadcasts
//! to all nodes uses `0x00` as the node_id byte (the spec says "0"
//! for "all", not a wildcard per se — we treat `0x00` and any
//! `node_id != 1` as "not for us" and ignore).

use core::sync::atomic::{AtomicU16, Ordering};

use cortex_m::peripheral::SCB;
use defmt::{info, warn};
use embassy_futures::select::{select, Either};
use embassy_stm32::can::{Can, Frame};
use embassy_time::{Duration, Ticker};

use crate::ota;
use crate::uds;
use uds_core::dtc;
use embassy_time::Instant;
use crate::drivers::can::uds_bridge as uds_transport;

/// Heartbeat producer period in milliseconds. Was previously
/// in `src/can/od.rs` (deleted in Phase 6 along with the
/// CANopen SDO server). 0 = heartbeat disabled (per CiA 301
/// §3.6.4 reserved value).
static HEARTBEAT_PRODUCER_MS: AtomicU16 = AtomicU16::new(1000);

/// Get the current heartbeat period. Used by `canopen_task`'s
/// ticker construction.
pub fn heartbeat_period_ms() -> u16 {
    HEARTBEAT_PRODUCER_MS.load(Ordering::Relaxed)
}

/// Set the heartbeat period. Public for external callers
/// (currently none in the Phase 6+ code base; kept for the
/// legacy 0x1017 OD path if it's restored).
#[allow(dead_code)]
pub fn set_heartbeat_period_ms(ms: u16) {
    HEARTBEAT_PRODUCER_MS.store(ms, Ordering::Relaxed);
}

/// Cache of the last heartbeat period the ticker was re-armed
/// with. Used to avoid re-allocating the `Ticker` on every
/// tick (which is what the naive `Ticker::every(...)` in the
/// `Either::Second` arm would do).
static LAST_HEARTBEAT_PERIOD_MS: AtomicU16 = AtomicU16::new(0);

/// Default NodeId. Hardcoded per Phase 1 spec; LSS service in a
/// later milestone can override at runtime.
pub const NODE_ID: u8 = 1;

/// COB-IDs derived from NodeId. Computed once at compile time
/// would be nicer, but `const fn` for `u16 + u8` in older Rust
/// toolchains is finicky; these are `const` so the cost is zero.
pub const HEARTBEAT_COB_ID: u16 = 0x700 + NODE_ID as u16;
pub const NMT_COB_ID: u16 = 0x000;

/// NMT state. CiA 301 §3.3.2 maps each state to a heartbeat byte.
#[derive(defmt::Format, Copy, Clone, Debug, PartialEq, Eq)]
pub enum NmtState {
    /// Power-on state, immediately followed by a single boot-up
    /// message and a transition to `PreOperational`. We never
    /// report a steady-state value of `Booting` — it's only the
    /// value of the very first heartbeat byte.
    Booting,
    /// Default after boot-up. SDO is up (Phase 2+); PDO is not.
    PreOperational,
    /// PDO active. Phase 1 has no PDO, so Operationally is
    /// functionally identical to PreOperational, but the state
    /// transitions correctly so Phase 2+ can attach SDO
    /// behavior on the Operational entry.
    Operational,
    /// Stopped — minimal comms, only NMT.
    Stopped,
}

impl NmtState {
    /// CiA 301 §3.3.2 heartbeat byte.
    pub fn heartbeat_byte(self) -> u8 {
        match self {
            Self::Booting => 0x00,
            Self::Stopped => 0x04,
            Self::Operational => 0x05,
            Self::PreOperational => 0x7F,
        }
    }
}

/// Apply an NMT command byte to the current state. Returns the
/// new state (or `None` if the command is unknown / unsupported
/// in Phase 1, in which case the state stays unchanged).
///
/// `current` is accepted for symmetry with the call site and future
/// use (Phase 4 reset-flow may depend on it). The result depends
/// only on `cmd` in Phase 1.
pub fn apply_nmt_command(_current: NmtState, cmd: u8) -> Option<NmtState> {
    match cmd {
        0x01 => Some(NmtState::Operational),
        0x02 => Some(NmtState::Stopped),
        0x80 => Some(NmtState::PreOperational),
        0x81 | 0x82 => {
            // Reset Node / Reset Communication. We don't support
            // these in Phase 1 (they would require a NVIC system
            // reset or a peripheral reset that the OTA-side
            // CANopen stack needs to coordinate). Treat as
            // "unhandled, log, ignore".
            warn!("NMT: reset command {:02x} not supported in Phase 1", cmd);
            None
        }
        _ => {
            warn!("NMT: unknown command {:02x}", cmd);
            None
        }
    }
}

/// Build a CAN frame for the heartbeat / boot-up message.
///
/// One-byte payload: the heartbeat state byte per `NmtState::heartbeat_byte`.
pub fn build_heartbeat_frame(state: NmtState) -> Frame {
    // unwrap: heartbeat payload is 1 byte, always valid.
    Frame::new_standard(HEARTBEAT_COB_ID, &[state.heartbeat_byte()])
        .expect("heartbeat payload is 1 byte, in [0, 8]")
}

/// CANopen NMT + heartbeat task.
///
/// On entry: emits one boot-up frame, then a 1 Hz heartbeat.
/// Continuously listens on `0x000` for NMT commands and applies
/// the corresponding state transitions.
///
/// Uses `embassy_futures::select` to race the heartbeat ticker
/// against incoming RX frames: whichever fires first wins, and
/// the loser is dropped. A small bias toward RX (the `select`
/// polls both futures and returns the first to complete) means
/// an NMT command gets serviced within a few hundred microseconds
/// of arrival.
///
/// Heartbeat period = 0 means "heartbeat disabled" (CiA 301 §3.6.4
/// — reserved). We represent it with `heartbeat = None` and
/// block the tick arm on `core::future::pending()` so the loop
/// only services RX. Constructing `Ticker::every(Duration::from_millis(0))`
/// would fire on every poll iteration and flood the bus.
///
/// **Lives in `.data` (RAM).** This is the SDO receive dispatch
/// loop — every SDO frame the master sends (including every
/// TransferData byte during OTA) goes through here. Putting
/// the task in RAM removes the only remaining flash-resident
/// trampoline on the OTA write path: `canopen_task → sdo::dispatch`.
/// Without this, the OTA write pointer would eventually cross
/// that trampoline (somewhere in canopen_task's body) and the
/// master would lose the ability to deliver TransferData frames
/// mid-transfer.
#[embassy_executor::task]
#[link_section = ".data"]
pub async fn canopen_task(can: &'static mut Can<'static>) {
    info!("CANopen: node {} starting", NODE_ID);

    // Boot-up: send a one-shot frame with state byte = 0x00 per
    // CiA 301 §3.3.1, then immediately drop into PreOperational.
    if let Some(_dropped) = can.write(&build_heartbeat_frame(NmtState::Booting)).await {
        warn!("CANopen: boot-up frame replaced a pending frame");
    }
    let mut state = NmtState::PreOperational;
    info!("CANopen: state → Pre-Operational");

    // Initial heartbeat ticker; the period may be changed at
    // runtime by writing 0x1017.0 via SDO. `None` = disabled
    // (period = 0). We re-evaluate this at the top of each
    // loop iteration so period changes take effect on the
    // next tick (or the next frame).
    // DTC: CAN communication timeout threshold (10 s without a frame).
    const CAN_TIMEOUT_MS: u64 = 10_000;
    const DTC_CAN_TIMEOUT: u32 = 0x030100;  // U0100-family proprietary
    let mut last_rx = Instant::now();
    let mut dtc_timeout_recorded = false;

    let initial_period = heartbeat_period_ms();
    LAST_HEARTBEAT_PERIOD_MS.store(initial_period, Ordering::Relaxed);
    let mut heartbeat: Option<Ticker> = if initial_period > 0 {
        Some(Ticker::every(Duration::from_millis(initial_period as u64)))
    } else {
        None
    };

    loop {
        // Re-evaluate the heartbeat period each iteration. If
        // it changed (and especially if it changed to 0), rebuild
        // the ticker (or disable it).
        let current = heartbeat_period_ms();
        if current != LAST_HEARTBEAT_PERIOD_MS.load(Ordering::Relaxed) {
            LAST_HEARTBEAT_PERIOD_MS.store(current, Ordering::Relaxed);
            heartbeat = if current > 0 {
                Some(Ticker::every(Duration::from_millis(current as u64)))
            } else {
                None
            };
        }

        // Phase 5c: drive the UDS pending queue. Advances any
        // continuations and pushes a 0x78 ResponsePending frame
        // if the request has been pending longer than P2.
        // No-op in Phase 5c (no jobs pushed yet).
        crate::uds::tick(embassy_time::Instant::now().elapsed().as_millis() as u32);

        // Race the heartbeat tick against the next received
        // frame. If a frame arrives, process it (NMT or UDS).
        // If the tick fires first, send a heartbeat frame.
        // Phase 6 commit 2: use `read_fd` so we can receive
        // both classic (NMT, heartbeat) and FD (UDS) frames
        // on the same bus.
        let rx_fut = can.read_fd();
        let tick_fut = async {
            match &mut heartbeat {
                Some(t) => t.next().await,
                // Heartbeat disabled — never tick. RX wins the
                // race every time and we just process frames.
                None => core::future::pending::<()>().await,
            }
        };
        match select(rx_fut, tick_fut).await {
            Either::First(Ok(envelope)) => {
                last_rx = Instant::now();
                if dtc_timeout_recorded {
                    dtc_timeout_recorded = false;
                    dtc::clear_dtc(DTC_CAN_TIMEOUT);
                    info!("CANopen: communication restored, DTC cleared");
                }
                let frame = envelope.frame;
                // CANopen uses 11-bit standard IDs exclusively.
                // Extended IDs are silently dropped.
                let id_u16: u16 = match frame.header().id() {
                    embedded_can::Id::Standard(s) => s.as_raw(),
                    embedded_can::Id::Extended(_) => 0xFFFF, // sentinel: never matches
                };
                if id_u16 == NMT_COB_ID {
                    let len = frame.header().len() as usize;
                    let data = frame.data();
                    // NMT frame format: [cmd, node_id]. We honour
                    // a frame addressed to *us* (node_id == NODE_ID)
                    // or to "all" (node_id == 0x00).
                    if len >= 2 && (data[1] == NODE_ID || data[1] == 0x00) {
                        if let Some(next) = apply_nmt_command(state, data[0]) {
                            if next != state {
                                info!("CANopen: state {:?} → {:?}", state, next);
                                state = next;
                            }
                        }
                    }
                } else if uds_transport::is_uds_frame(&frame) {
                    // UDS transport: dispatch the UDS request
                    // directly (no CANopen SDO tunnel in Phase 6+).
                    // Phase 6 commit 2: UDS uses CAN-FD frames
                    // (up to 64 bytes) so a single frame covers
                    // even the long services (0x34 RequestDownload
                    // = 11 bytes, 0x19 ReadDTC with many DTCs).
                    if let Some(response) = uds_transport::handle_rx_frame(&frame) {
                        // Wrap the TX in a timeout. If FDCAN1 enters
                        // bus-off (wiring fault, error storm), the
                        // embassy write future can pend indefinitely;
                        // because the canopen task lives in RAM and
                        // runs at the executor's poll rate, that
                        // would freeze the motor task (10 kHz PWM)
                        // too. A 10 ms timeout is generous for a
                        // 500 kbps bus — even a multi-frame burst
                        // finishes in well under 1 ms.
                        let tx = can.write_fd(&response);
                        match embassy_time::with_timeout(
                            embassy_time::Duration::from_millis(10),
                            tx,
                        ).await {
                            Ok(Some(_dropped)) => {
                                warn!("CANopen: UDS response replaced a pending frame");
                            }
                            Ok(None) => {} // sent cleanly
                            Err(_) => {
                                warn!("CANopen: UDS response TX timed out (bus-off?)");
                            }
                        }
                        // After sending the SDO response, check
                        // whether a UDS HardReset was requested
                        // (0x11 0x01) or an OTA TransferExit
                        // (0x37) finished — both arms the same
                        // reset flag. We fire the NVIC reset
                        // from here — not from inside the
                        // handlers — so the response has time
                        // to make it out before NVIC tears
                        // the chip down.
                        if uds::take_reset_request() || ota::take_reset_request() {
                            let kind = match uds::take_reset_subfunc() {
                                Some(0x01) => "Hard",
                                Some(0x03) => "Soft",
                                _ => "OTA",
                            };
                            info!("UDS/OTA: NVIC {} reset in 10 ms", kind);
                            // 10 ms at 170 MHz; lets the last
                            // TX byte (and any pending CAN
                            // frame) reach the wire.
                            cortex_m::asm::delay(170_000_000 / 100);
                            SCB::sys_reset();
                        }
                    }
                }
                // Other COB-IDs (PDO reserved, SYNC, EMCY, ...) are
                // silently dropped in Phase 2.
            }
            Either::First(Err(_e)) => {
                // Bus error (e.g. controller entered error-passive).
                // The driver has already attempted recovery; we
                // just continue.
            }
            Either::Second(()) => {
                // DTC: check CAN communication timeout.
                if last_rx.elapsed().as_millis() >= CAN_TIMEOUT_MS && !dtc_timeout_recorded {
                    dtc_timeout_recorded = true;
                    dtc::set_dtc(DTC_CAN_TIMEOUT, dtc::status::CONFIRMED
                                 | dtc::status::TEST_FAILED_CURRENT);
                    warn!("CANopen: no frame for {} ms — DTC 0x{:06x} recorded",
                          CAN_TIMEOUT_MS, DTC_CAN_TIMEOUT);
                }
                // Phase 5c: if 0x28 disableNormalCommunication
                // is active, skip the heartbeat (TX is muted).
                if !crate::uds::tx_disabled() {
                    // Same timeout protection as the SDO TX path —
                    // a bus-off heartbeat would otherwise stall the
                    // executor.
                    let hb_frame = build_heartbeat_frame(state);
                    let tx = can.write(&hb_frame);
                    match embassy_time::with_timeout(
                        embassy_time::Duration::from_millis(10),
                        tx,
                    ).await {
                        Ok(Some(_dropped)) => {
                            warn!("CANopen: heartbeat frame replaced a pending frame");
                        }
                        Ok(None) => {}
                        Err(_) => {
                            warn!("CANopen: heartbeat TX timed out (bus-off?)");
                        }
                    }
                    // Reflect a runtime change of 0x1017.0: if the
                    // heartbeat period was updated via SDO, restart
                    // the ticker so the next tick honours the new
                    // period. We use a static mutable cache to detect
                    // the change without re-allocating on every tick.
                    let current = heartbeat_period_ms();
                    if current != LAST_HEARTBEAT_PERIOD_MS.load(Ordering::Relaxed) {
                        LAST_HEARTBEAT_PERIOD_MS.store(current, Ordering::Relaxed);
                        heartbeat = Some(Ticker::every(Duration::from_millis(current as u64)));
                    }
                }
            }
        }
    }
}
