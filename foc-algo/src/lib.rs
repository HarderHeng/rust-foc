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

#![no_std]

#[cfg(test)]
extern crate std;

pub mod cascade;
pub mod current_loop_controller;
pub mod feedforward;
pub mod filter;
pub mod pid;
pub mod position_loop_controller;
pub mod ramp;
pub mod speed_loop_controller;
pub mod startup;
pub mod svpwm;
pub mod transforms;

pub use cascade::{FocController, Meas, Mode, Runtime, Target};
pub use current_loop_controller::{CurrentLoop, Runtime as CurrentLoopRuntime};
pub use feedforward::inertia_viscous;
pub use filter::LowPassFilter;
pub use pid::Pid;
pub use position_loop_controller::{PositionFfFn, PositionLoopController};
pub use ramp::Ramp;
pub use speed_loop_controller::{SpeedFfFn, SpeedLoopController};
pub use startup::field_weakening;
pub use svpwm::{Duty, Svpwm};
pub use transforms::{
    Abc, AlphaBeta, Dq, Trig,
    clark, clark_balanced, inv_clark, inv_park, park,
};

#[cfg(feature = "libm-trig")]
pub use transforms::LibmTrig;
