//! Speed loop — PI + optional feedforward → Iq reference.
//!
//! ## Cascaded control
//!
//! ```text
//! speed_ref ──┐
//!             │
//! speed_fdb ──┴─→ [PI]──┬─→ iq_target ──→ current loop
//!                      │
//! ff_callback ─────────┘   (optional, injected as fn pointer)
//! ```
//!
//! The feedforward callback receives the raw reference, measurement, and
//! acceleration so it can implement any model (inertia+viscous, custom, etc.).
//! Set `ff_callback = None` to disable feedforward entirely.
//!
//! ## Usage
//!
//! ```ignore
//! fn my_ff(_ref: f32, speed: f32, accel: f32) -> f32 {
//!     inertia_viscous(0.001, 0.0005, accel, speed)
//! }
//!
//! let mut speed = SpeedLoopController::default();
//! speed.pid.kp = 0.5;
//! speed.ff_callback = Some(my_ff);
//! let iq_ref = speed.update(target_speed, measured_speed, measured_accel, dt);
//! ```

use crate::pid::{self, Pid};

/// Feedforward callback for the speed loop.
///
/// Arguments: `(speed_ref, measured_speed, measured_accel)`.
/// Return: feedforward contribution in amperes (added to PI output, then
/// clamped to `pid.output_limit`).
pub type SpeedFfFn = fn(speed_ref: f32, speed: f32, accel: f32) -> f32;

/// Intermediate values for logging / VOFA / debug.
#[derive(Default, Clone, Copy)]
pub struct Runtime {
    pub pi_output: f32,
    pub ff_total: f32,
    pub speed_measured: f32,
}

/// Speed loop controller — PI + optional feedforward → Iq target.
pub struct SpeedLoopController {
    pub pid: Pid,
    /// Optional feedforward callback.  `None` disables feedforward.
    pub ff_callback: Option<SpeedFfFn>,
    pub runtime: Runtime,
}

impl Default for SpeedLoopController {
    fn default() -> Self {
        let mut pid = Pid::new();
        pid.output_limit = 30.0; // Iq upper bound (A)
        Self {
            pid,
            ff_callback: None,
            runtime: Runtime::default(),
        }
    }
}

impl SpeedLoopController {
    /// Run one speed-loop cycle.
    ///
    /// Returns the Iq reference for the current loop and writes debug values
    /// into `self.runtime`.
    pub fn update(&mut self, speed_ref: f32, speed: f32, accel: f32, dt: f32) -> f32 {
        let pi_out = self.pid.update(speed_ref, speed, dt);
        self.runtime.pi_output = pi_out;
        self.runtime.speed_measured = speed;

        let ff = match self.ff_callback {
            Some(f) => f(speed_ref, speed, accel),
            None => 0.0,
        };
        self.runtime.ff_total = ff;

        pid::combine_pi_ff(&self.pid, pi_out, ff)
    }

    pub fn reset(&mut self) {
        self.pid.reset();
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn with_pid(kp: f32, ki: f32, kd: f32, limit: f32) -> SpeedLoopController {
        let mut s = SpeedLoopController::default();
        s.pid.kp = kp; s.pid.ki = ki; s.pid.kd = kd;
        s.pid.output_limit = limit;
        s
    }

    #[test]
    fn zero_state_zero_output() {
        let mut s = with_pid(1.0, 0.0, 0.0, 10.0);
        let out = s.update(0.0, 0.0, 0.0, 0.001);
        approx(out, 0.0);
    }

    #[test]
    fn pi_only_when_no_callback() {
        let mut s = with_pid(1.0, 0.0, 0.0, 10.0);
        s.ff_callback = None;
        let out = s.update(2.0, 0.0, 0.0, 0.001);
        approx(out, 2.0); // pure P: kp × error = 1 × 2
    }

    #[test]
    fn feedforward_inertia_term() {
        fn inertia_ff(_ref: f32, _speed: f32, accel: f32) -> f32 { 0.5 * accel }

        let mut s = with_pid(0.0, 0.0, 0.0, 10.0);
        s.ff_callback = Some(inertia_ff);
        let out = s.update(0.0, 0.0, 10.0, 0.001);
        approx(s.runtime.ff_total, 5.0);
        approx(out, 5.0);
    }

    #[test]
    fn feedforward_viscous_term() {
        fn viscous_ff(_ref: f32, speed: f32, _accel: f32) -> f32 { 0.1 * speed }

        let mut s = with_pid(0.0, 0.0, 0.0, 10.0);
        s.ff_callback = Some(viscous_ff);
        let out = s.update(0.0, 50.0, 0.0, 0.001);
        approx(s.runtime.ff_total, 5.0);
        approx(out, 5.0);
    }

    #[test]
    fn pi_and_feedforward_sum() {
        fn inertia_ff(_ref: f32, _speed: f32, accel: f32) -> f32 { 0.5 * accel }

        let mut s = with_pid(1.0, 0.0, 0.0, 10.0);
        s.ff_callback = Some(inertia_ff);
        // P = 1 × (2 − 0) = 2, FF = 0.5 × 4 = 2, total = 4
        let out = s.update(2.0, 0.0, 4.0, 0.001);
        approx(out, 4.0);
        approx(s.runtime.ff_total, 2.0);
    }

    #[test]
    fn integrator_accumulates() {
        let mut s = with_pid(0.0, 1.0, 0.0, 10.0);
        for _ in 0..5 { s.update(1.0, 0.0, 0.0, 0.1); }
        approx(s.pid.integral, 0.5);
    }

    #[test]
    fn feedforward_does_not_bypass_clamp() {
        fn huge_ff(_ref: f32, _speed: f32, accel: f32) -> f32 { 100.0 * accel }

        let mut s = with_pid(1.0, 0.0, 0.0, 10.0);
        s.ff_callback = Some(huge_ff);
        // P = 2, FF = 400, unclamped = 402
        let out = s.update(2.0, 0.0, 4.0, 0.001);
        approx(out, 10.0);  // clamped
    }

    #[test]
    fn callback_can_use_ref() {
        // A feedforward that depends on the speed reference, not the measurement
        fn ref_based_ff(speed_ref: f32, _speed: f32, _accel: f32) -> f32 { 0.1 * speed_ref }

        let mut s = with_pid(0.0, 0.0, 0.0, 10.0);
        s.ff_callback = Some(ref_based_ff);
        let out = s.update(30.0, 0.0, 0.0, 0.001);
        approx(out, 3.0);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
