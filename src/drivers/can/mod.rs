//! CAN bus module — owns the FDCAN1 peripheral and the protocol
//! stacks: NMT + heartbeat (CANopen) and UDS (independent on
//! the same bus).
//!
//! This module is **independent of the shell** (which lives on
//! USART2 in `src/tasks/shell.rs`). The shell stack was deliberately
//! left alone when the FDCAN path was added — the two transports
//! serve different purposes:
//!   - USART2 + embedded-cli: human-readable command interface,
//!     used during development for `spin` / `stop` / `version` etc.
//!   - FDCAN1: machine-to-machine interface, used for
//!     diagnostics and OTA. NMT + heartbeat use classic CAN
//!     8-byte frames; UDS uses dedicated CAN-IDs (0x7DF / 0x7E0 /
//!     0x7E8) and (in a follow-up) CAN-FD 64-byte frames.
//!
//! ## Architecture (Phase 6 — decoupling UDS from CANopen)
//!
//! ```
//!   FDCAN1
//!     ├── NMT + heartbeat   (CANopen, classic CAN, 0x000 + 0x701)
//!     └── UDS               (independent protocol, 0x7DF/0x7E0/0x7E8)
//! ```
//!
//! Before Phase 6, UDS was tunneled through CANopen SDO at
//! 0x2F00.0 (vendor-specific). That coupling is removed:
//! `src/can/sdo.rs` and the SDO server are deleted.
//!
//! **Phase 8**: UDS is decoupled from `src/can/` entirely
//! (it lives in `src/uds/`, a top-level application module).
//! `src/can/` now only owns bus-level concerns: NMT, heartbeat,
//! and the FDCAN1 frame I/O. OTA is also application, lives
//! in `src/ota/`. The transport adapter (`src/uds/transport/`)
//! is the only thing that knows about FDCAN frames from the
//! UDS side.

pub mod canopen;
pub mod uds_bridge;

use defmt::info;
use embassy_stm32::{
    bind_interrupts,
    can::{Can, CanConfigurator, IT0InterruptHandler, IT1InterruptHandler},
    peripherals::FDCAN1,
};

bind_interrupts!(struct CanIrqs {
    FDCAN1_IT0 => IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => IT1InterruptHandler<FDCAN1>;
});

/// Nominal (arbitration) bit rate. 500 kbps is the conservative
/// choice for industrial CAN; 1 Mbps is the next step up. Phase 1
/// uses 500 kbps.
pub const CAN_BITRATE_BPS: u32 = 500_000;

/// Data-phase bit rate (CAN-FD only). 2 Mbps is the standard
/// "CAN-FD fast" rate and is 4× the nominal. The driver
/// computes the TDC (transceiver delay compensation) from the
/// bit timing parameters.
pub const CAN_FD_DATA_BITRATE_BPS: u32 = 2_000_000;

/// Configure FDCAN1 on PB9 (TX) and PA11 (RX) for **CAN-FD**
/// (nominal 500 kbps + data 2 Mbps, up to 64-byte frames),
/// and return a `Can<'static>` ready for use.
///
/// The returned handle is in `NormalOperationMode` — the bus is
/// active and can send / receive both classic and FD frames
/// (mixed bus, FDCAN1 hardware supports both simultaneously).
///
/// **Phase 6 commit 2**: this used to be classic-only. The
/// switch enables single-frame UDS up to 64 bytes, which
/// covers the long UDS services (0x34 RequestDownload = 11
/// bytes, 0x19 ReadDTCInformation with many DTCs) without
/// multi-frame segmentation. NMT + heartbeat stay on classic
/// frames (1 byte payload, well under the 8-byte limit).
pub fn init_fdcan1(
    p_fdcan: embassy_stm32::Peri<'static, FDCAN1>,
    p_tx: embassy_stm32::Peri<'static, impl embassy_stm32::can::TxPin<FDCAN1>>,
    p_rx: embassy_stm32::Peri<'static, impl embassy_stm32::can::RxPin<FDCAN1>>,
) -> Can<'static> {
    let mut configurator = CanConfigurator::new(p_fdcan, p_rx, p_tx, CanIrqs);
    // Nominal bitrate (used for arbitration + classic frames).
    configurator.set_bitrate(CAN_BITRATE_BPS);
    // Data-phase bitrate (used for FD frames only). `true` =
    // enable transceiver delay compensation (TDC), required
    // by the FDCAN peripheral at >1 Mbps.
    configurator.set_fd_data_bitrate(CAN_FD_DATA_BITRATE_BPS, true);
    let can = configurator.into_normal_mode();
    info!(
        "FDCAN1 ready: CAN-FD {} kbps nominal + {} kbps data (PB9 TX / PA11 RX)",
        CAN_BITRATE_BPS / 1000, CAN_FD_DATA_BITRATE_BPS / 1000
    );
    can
}
