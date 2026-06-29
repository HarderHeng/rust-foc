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

pub mod current_loop_controller;
pub mod pid;
pub mod speed_loop_controller;
pub mod svpwm;
pub mod transforms;

pub use current_loop_controller::{CurrentLoopController, Measurements, Runtime, Targets};
pub use pid::Pid;
pub use speed_loop_controller::{Feedforward, SpeedLoopController};
pub use svpwm::{Duty, Svpwm};
pub use transforms::{
    Abc, AlphaBeta, Dq, LibmTrig, Trig,
    clark, clark_balanced, inv_clark, inv_park, park,
};
