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
//!             foc-algo ──→ libm (sin/cos for Park transforms)
//! ```

#![cfg_attr(not(test), no_std)]
#![cfg_attr(test, allow(unused_imports))]

pub mod current_loop;
pub mod pid;
pub mod svpwm;
pub mod transforms;

pub use current_loop::{FocConfig, FocController};
pub use pid::{Pid, PidConfig};
pub use svpwm::{svpwm, SvpwmDuty};
pub use transforms::{
    Abc, AlphaBeta, Dq, LibmTrig, Trig,
    abc_to_dq, abc_to_dq_default,
    clark, clark_balanced,
    dq_to_abc, dq_to_abc_default,
    inv_clark, inv_park, inv_park_default,
    park, park_default,
};
