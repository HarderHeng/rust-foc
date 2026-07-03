//! 0x28 CommunicationControl handler.
//!
//! Wire format (per ISO 14229):
//!   [0x28, subfunc, network_type] → [0x68, subfunc]
//!
//! Subfuncs (Phase 5b supports all four):
//!   0x00 enableNormalCommunication         (TX ON,  RX ON)
//!   0x01 enableRxDisableTxNormalComm       (TX OFF, RX ON)
//!   0x02 enableTxDisableRxNormalComm       (TX ON,  RX OFF)
//!   0x03 disableNormalCommunication         (TX OFF, RX OFF)
//!
//! `tx_disabled` is checked by the canopen task: when true, it
//! skips heartbeat / NMT ACK / SDO response. OTA-firmware-issued
//! 0x28 0x03 is the standard "go silent" prelude to a flash
//! rewrite — the canopen task will refuse to TX anything until
//! a fresh 0x28 0x00 re-enables it.
//!
//! `rx_disabled` is currently advisory — the dispatcher always
//! accepts incoming SDO frames, but downstream consumers can
//! check `state.rx_disabled` to drop processing.

use defmt::info;

use super::nrc::Nrc;
use super::state::{store_response, UdsState};

pub fn handle(state: &mut UdsState, req: &[u8]) {
    // [0x28, subfunc, network_type] → [0x68, subfunc]
    if req.len() != 3 {
        store_response(&Nrc::IncorrectMessageLengthOrInvalidFormat
            .negative_response(0x28));
        return;
    }
    let subfunc = req[1];
    let _network_type = req[2];  // simplified: ignore (only 0x01 = normalCommNetwork supported)

    match subfunc {
        0x00 => {
            state.tx_disabled = false;
            state.rx_disabled = false;
            info!("UDS: CommControl enable (TX+RX ON)");
            store_response(&[0x68, 0x00]);
        }
        0x01 => {
            // enableRxDisableTx: keep listening, but stop transmitting.
            state.tx_disabled = true;
            state.rx_disabled = false;
            info!("UDS: CommControl enableRxDisableTx (TX OFF, RX ON)");
            store_response(&[0x68, 0x01]);
        }
        0x02 => {
            // enableTxDisableRx: keep transmitting, but drop incoming.
            state.tx_disabled = false;
            state.rx_disabled = true;
            info!("UDS: CommControl enableTxDisableRx (TX ON, RX OFF)");
            store_response(&[0x68, 0x02]);
        }
        0x03 => {
            // disableNormalCommunication: full silence.
            state.tx_disabled = true;
            state.rx_disabled = true;
            info!("UDS: CommControl disable (TX+RX OFF)");
            store_response(&[0x68, 0x03]);
        }
        _ => {
            store_response(&Nrc::SubFunctionNotSupported.negative_response(0x28));
        }
    }
}
