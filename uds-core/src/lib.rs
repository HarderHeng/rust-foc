#![cfg_attr(not(test), no_std)]

//! ISO 14229 UDS (Unified Diagnostic Services) protocol engine.
//!
//! This crate provides a pure application-layer UDS implementation
//! with **zero platform dependencies**. It compiles on any target
//! that supports `core` + the `aes` crate + `critical-section`.
//!
//! ## Architecture
//!
//! - `types` — protocol enums (Session, SecurityLevel, SrvState, Nrc)
//! - `state` — `UdsState` (mutable engine state) + response buffer
//! - `table` — `UdsConfig` schema + all `dispatch_0xNN` methods
//! - `crypto` — AES-128 key derivation (pure functions, unit-tested)
//! - `pending` — pending queue + 0x78 ResponsePending machinery
//! - `dtc` — DTC (Diagnostic Trouble Code) storage
//!
//! ## Porting to a new platform
//!
//! Write a platform crate that:
//!
//! 1. Creates a `static UDS_STATE: UdsState = UdsState::zeroed()`
//! 2. Creates a `static UDS_CONFIG: UdsConfig` with tables + callbacks
//! 3. Calls `uds_core::store_response()` / `uds_core::load_response()`
//!    to exchange bytes with the transport layer
//!
//! See `src/uds/mod.rs` and `src/uds/static_config.rs` in the
//! `foc-rust` project for an example.

#[cfg(feature = "defmt")]
extern crate defmt;

pub mod crypto;
pub mod dtc;
pub mod pending;
pub mod state;
pub mod table;
pub mod types;

// Re-export key types for convenience.
pub use crypto::AesBlock;
pub use state::{load_response, store_response, UdsState};
pub use table::UdsConfig;
pub use types::*;

/// Logging macro: logs via defmt when the `defmt` feature is enabled,
/// compiles to nothing otherwise.
#[macro_export]
macro_rules! uds_log {
    ($($arg:tt)*) => {
        #[cfg(feature = "defmt")]
        defmt::info!($($arg)*);
    };
}
