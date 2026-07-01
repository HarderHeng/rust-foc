//! Pure FOC algorithms — no hardware dependencies.
//!
//! Each module is a self-contained math component that can be unit-tested on
//! any platform with `cargo test`.
//!
//! Crate is `no_std` by default; the `std` feature enables platform I/O for
//! host-side tooling (if needed).
//!
//! # Where to start
//!
//! Most users only need three things from this crate:
//!
//! 1. **[`FocController`]** — the top-level state machine.  Owns the
//!    measurement/state/runtime fields and dispatches to inner loops.
//! 2. **[`MotorParams`]** — pack your motor's physical constants once, then
//!    call its `current_pi_gains` / `speed_pi_gains` / `torque_to_iq`
//!    methods to derive everything else.
//! 3. **Helpers** — [`field_weakening`] / `mtpa` for high-speed `id`
//!    reference, [`decoupling_voltage`] for cross-coupling compensation,
//!    [`SmoObserver`] for sensorless angle, [`I2tLimiter`] for thermal
//!    foldback.
//!
//! The inner loops ([`SpeedLoopController`], [`PositionLoopController`],
//! [`CurrentLoop`]) are exposed for users who want to assemble their own
//! cascade.  The math primitives ([`Pid`], [`Ramp`], [`Duty`], etc.)
//! compose into those loops but you usually don't need them directly.
//!
//! # Example
//!
//! ```ignore
//! use foc_algo::{FocController, Mode, MotorParams};
//!
//! let motor = MotorParams {
//!     r: 0.3, ld: 0.0005, lq: 0.0005, flux_linkage: 0.01,
//!     pole_pairs: 7, continuous_current: 5.0, inertia: 1e-5,
//! };
//!
//! let (kp_i, ki_i) = motor.current_pi_gains(1000.0);
//! let (kp_s, ki_s) = motor.speed_pi_gains(50.0, 1000.0);
//!
//! let mut ctrl = FocController::new(Mode::Speed);
//! ctrl.set_current_limit(motor.continuous_current);
//! ctrl.speed_pid().kp = kp_s;
//! ctrl.speed_pid().ki = ki_s;
//! ctrl.current_pid_d().kp = kp_i;
//! ctrl.current_pid_d().ki = ki_i;
//! ctrl.iq_ramp.rate_limit = 1000.0; // A/s
//!
//! // Each control cycle:
//! ctrl.meas.ia = adc_a.read();
//! ctrl.meas.ib = adc_b.read();
//! ctrl.meas.theta = encoder.read();
//! ctrl.meas.vdc = vbus.read();
//! ctrl.target.speed_ref = setpoint;
//! ctrl.update::<MyTrig>(dt);
//! timer.set_duty(ctrl.duty.to_timer_counts(pwm_period));
//! ```
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
//! ├── state.rs            Target / Meas / ControllerState per-cycle structs
//! ├── protection.rs       I²t thermal limiter + rotor-alignment docs
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
//!     ├── circle_limitation.rs  vector amplitude clamp
//!     └── decoupling.rs   PMSM dq-axis feedforward voltages
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
pub mod protection;
pub mod state;

// ── Re-exports: top-level convenience ──────────────────────────────────────

// Math primitives
pub use math::{
    Abc, AlphaBeta, Dq, Duty, LowPassFilter, Pid, Ramp, Svpwm, Trig,
    circle_limitation, clark, clark_balanced, combine_pi_ff, decoupling_voltage,
    ic_from_iab, inv_clark, inv_park, park,
};
#[cfg(feature = "libm-trig")]
pub use math::LibmTrig;

// Control loops
pub use loops::{
    CurrentLoop, PositionFfFn, PositionLoopController, SpeedFfFn,
    SpeedLoopController, coulomb_friction, inertia_viscous,
};

// Top-level orchestration
pub use cascade::{FocController, Mode};
pub use state::{ControllerState, Meas, Target};
pub use motor::MotorParams;
pub use field_weakening::field_weakening;
pub use protection::I2tLimiter;
#[cfg(feature = "libm-trig")]
pub use observer::{SmoConfig, SmoObserver, SmoRuntime, pll_pi_gains};
