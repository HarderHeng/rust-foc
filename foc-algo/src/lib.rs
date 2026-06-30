//! Pure FOC algorithms — no hardware dependencies.
//!
//! Each module is a self-contained math component that can be unit-tested on
//! any platform with `cargo test`.
//!
//! Crate is `no_std` by default; the `std` feature enables platform I/O for
//! host-side tooling (if needed).
//!
//! # Crate layout
//!
//! ```text
//! foc-rust ──→ foc-algo (pure math)
//!             foc-algo ──→ libm (sin/cos for Park transforms)
//! ```
//!
//! ```text
//! foc-algo/src/
//! ├── lib.rs              (this file — module declarations + re-exports)
//! ├── cascade.rs          FocController (top-level orchestration)
//! ├── startup.rs          I²t thermal limiter + rotor-alignment docs
//! ├── motor.rs            MotorParams + PI/PLL auto-tuning
//! ├── observer.rs         Sliding-mode sensorless observer
//! │
//! ├── loops/              closed-loop controllers
//! │   ├── current.rs      d/q-axis current loop → PWM duty
//! │   ├── speed.rs        speed PI + feedforward → Iq
//! │   ├── position.rs     position PI + feedforward → ω
//! │   └── feedforward.rs  inertia/viscous/coulomb compensation formulas
//! │
//! └── math/               zero-dep primitives
//!     ├── pid.rs          discrete PID
//!     ├── filter.rs       1st-order low-pass
//!     ├── ramp.rs         rate limiter
//!     ├── svpwm.rs        space-vector modulation
//!     ├── transforms.rs   Clarke/Park + 3-phase helpers
//!     └── circle_limitation.rs  vector amplitude clamp
//! ```

#![no_std]

#[cfg(test)]
extern crate std;

pub mod cascade;
pub mod field_weakening;
pub mod loops;
pub mod math;
pub mod motor;
#[cfg(feature = "libm-trig")]
pub mod observer;
pub mod startup;

// ── Re-exports: top-level convenience ──────────────────────────────────────

// Math primitives
pub use math::{
    Abc, AlphaBeta, Dq, Duty, LowPassFilter, Pid, Ramp, Svpwm, Trig,
    circle_limitation, clark, clark_balanced, combine_pi_ff, ic_from_iab,
    inv_clark, inv_park, park,
};
#[cfg(feature = "libm-trig")]
pub use math::LibmTrig;

// Control loops
pub use loops::{
    CurrentLoop, PositionFfFn, PositionLoopController, SpeedFfFn,
    SpeedLoopController, coulomb_friction, inertia_viscous,
};

// Top-level orchestration
pub use cascade::{FocController, Meas, Mode, Runtime, Target};
pub use motor::MotorParams;
pub use field_weakening::field_weakening;
pub use startup::I2tLimiter;
#[cfg(feature = "libm-trig")]
pub use observer::{SmoConfig, SmoObserver, SmoRuntime, pll_pi_gains};
