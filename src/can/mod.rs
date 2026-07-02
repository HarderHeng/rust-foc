//! CAN bus module — owns the FDCAN1 peripheral and the OTA-side
//! protocol stack (CANopen + UDS).
//!
//! This module is **independent of the shell** (which lives on
//! USART2 in `src/tasks/shell.rs`). The shell stack was deliberately
//! left alone when the FDCAN path was added — the two transports
//! serve different purposes:
//!   - USART2 + embedded-cli: human-readable command interface,
//!     used during development for `spin` / `stop` / `version` etc.
//!   - FDCAN1 + CANopen: machine-to-machine interface, used for
//!     diagnostics and OTA. This is what the rest of the spec at
//!     `docs/superpowers/specs/2026-07-02-can-ota-uds-design.md`
//!     covers.
//!
//! Phase 1 (this commit) ships:
//!   - FDCAN1 driver init on PB9 (TX) / PA11 (RX)
//!   - Minimal CANopen: NMT state machine + 1 Hz heartbeat
//!
//! Phase 2 adds the SDO server; Phase 3 adds UDS via a vendor
//! SDO object; Phase 4 adds OTA over the UDS TransferData flow.

pub mod canopen;
pub mod od;
pub mod sdo;

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

/// Bit rate of the FDCAN1 bus. 500 kbps is the conservative choice
/// for industrial CAN, 1 Mbps is the next step up. Phase 1 uses
/// 500 kbps; the bit-timing register is one call away from a
/// re-config if the master is 1 Mbps only.
pub const CAN_BITRATE_BPS: u32 = 500_000;

/// Configure FDCAN1 on PB9 (TX) and PA11 (RX) for classic CAN at
/// 500 kbps, and return a `Can<'static>` ready for use.
///
/// The returned handle is in `NormalOperationMode` — the bus is
/// active and can send / receive classic CAN frames. The acceptance
/// filter is left at the driver default (accept-all) during Phase 1
/// so the node hears every frame on the bus; tightening to a
/// master-only filter is a Phase 2/3 concern.
pub fn init_fdcan1(
    p_fdcan: embassy_stm32::Peri<'static, FDCAN1>,
    p_tx: embassy_stm32::Peri<'static, impl embassy_stm32::can::TxPin<FDCAN1>>,
    p_rx: embassy_stm32::Peri<'static, impl embassy_stm32::can::RxPin<FDCAN1>>,
) -> Can<'static> {
    let mut configurator = CanConfigurator::new(p_fdcan, p_rx, p_tx, CanIrqs);
    // Bit timing first, then mode. The driver writes the timings
    // into the NBTP register; classic CAN doesn't need the FDCAN-
    // specific DBTP / data phase config.
    configurator.set_bitrate(CAN_BITRATE_BPS);
    let can = configurator.into_normal_mode();
    info!(
        "FDCAN1 ready: {} kbps classic CAN (PB9 TX / PA11 RX)",
        CAN_BITRATE_BPS / 1000
    );
    can
}
