//! Pure FOC algorithms — no hardware dependencies.
//!
//! Each module is a self-contained math component that can be unit-tested on
//! any platform with `cargo test`.
//!
//! Crate is `no_std` by default; the `std` feature enables platform I/O for
//! host-side tooling (if needed).
//!
//! # Crate graph
//!
//! ```text
//! foc-rust ──→ foc-algo (pure math)
//!             foc-algo ──→ (none — zero dependencies)
//! ```

#![cfg_attr(not(test), no_std)]
#![cfg_attr(test, allow(unused_imports))]

pub mod pid;

pub use pid::{Pid, PidConfig};
