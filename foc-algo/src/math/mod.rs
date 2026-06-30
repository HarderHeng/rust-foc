//! Pure-math primitives — zero dependencies on the rest of the crate.
//!
//! Each module is a self-contained building block used by the control
//! loops.  All modules in this folder can be unit-tested in isolation on
//! any `cargo test` host.

pub mod circle_limitation;
pub mod filter;
pub mod pid;
pub mod ramp;
pub mod svpwm;
pub mod transforms;

pub use circle_limitation::circle_limitation;
pub use filter::LowPassFilter;
pub use pid::{Pid, combine_pi_ff};
pub use ramp::Ramp;
pub use svpwm::{Duty, Svpwm};
pub use transforms::{
    Abc, AlphaBeta, Dq, Trig,
    clark, clark_balanced, ic_from_iab, inv_clark, inv_park, park,
};

#[cfg(feature = "libm-trig")]
pub use transforms::LibmTrig;
