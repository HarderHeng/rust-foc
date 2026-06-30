//! Speed loop — PI + optional feedforward → Iq reference.
//!
//! ## Cascaded control
//!
//! ```text
//! speed_ref ──┐
//!             │
//! speed_fdb ──┴─→ [PI]──┬─→ iq_target ──→ current loop
//!                      │
//! accel ───────────────┤   (inertia feedforward)
//! speed_fdb ───────────┘   (viscous feedforward)
//! ```
//!
//! ## Usage
//!
//! ```ignore
//! let mut speed = SpeedLoopController::default();
//! speed.pid.kp = 0.5;
//! speed.feedforward.inertia_gain = 0.001;
//! let iq_ref = speed.update(target_speed, measured_speed, measured_accel, dt);
//! // feed iq_ref into the current loop
//! ```

use crate::feedforward::Feedforward;
use crate::pid::Pid;

/// Intermediate values for logging / VOFA / debug.
#[derive(Default, Clone, Copy)]
pub struct Runtime {
    pub pi_output: f32,
    pub ff_inertia: f32,
    pub ff_viscous: f32,
    pub ff_total: f32,
    pub speed_measured: f32,
}

/// Speed loop controller — PI + optional feedforward → Iq target.
pub struct SpeedLoopController {
    pub pid: Pid,
    pub feedforward: Feedforward,
    pub runtime: Runtime,
}

impl Default for SpeedLoopController {
    fn default() -> Self {
        let mut pid = Pid::new();
        pid.output_limit = 30.0; // Iq upper bound (A)
        Self {
            pid,
            feedforward: Feedforward::default(),
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

        let (fi, fv, ft) = self.feedforward.compute(accel, speed);
        self.runtime.ff_inertia = fi;
        self.runtime.ff_viscous = fv;
        self.runtime.ff_total = ft;

        (pi_out + ft).clamp(-self.pid.output_limit, self.pid.output_limit)
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
    fn pi_only_when_feedforward_disabled() {
        let mut s = with_pid(1.0, 0.0, 0.0, 10.0);
        s.feedforward.enabled = false;
        s.feedforward.inertia_gain = 100.0;
        s.feedforward.viscous_gain = 100.0;
        let out = s.update(2.0, 0.0, 0.0, 0.001);
        approx(out, 2.0); // pure P: kp × error = 1 × 2
    }

    #[test]
    fn feedforward_inertia_term() {
        let mut s = with_pid(0.0, 0.0, 0.0, 10.0);
        s.feedforward.inertia_gain = 0.5;
        let out = s.update(0.0, 0.0, 10.0, 0.001);
        approx(s.runtime.ff_inertia, 5.0);
        approx(out, 5.0);
    }

    #[test]
    fn feedforward_viscous_term() {
        let mut s = with_pid(0.0, 0.0, 0.0, 10.0);
        s.feedforward.viscous_gain = 0.1;
        let out = s.update(0.0, 50.0, 0.0, 0.001);
        approx(s.runtime.ff_viscous, 5.0);
        approx(out, 5.0);
    }

    #[test]
    fn pi_and_feedforward_sum() {
        let mut s = with_pid(1.0, 0.0, 0.0, 10.0);
        s.feedforward.inertia_gain = 0.5;
        s.feedforward.viscous_gain = 0.0;
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
        let mut s = with_pid(1.0, 0.0, 0.0, 10.0);
        s.feedforward.inertia_gain = 100.0;  // huge FF contribution
        // P = 1 × 2 = 2, FF = 100 × 4 = 400, unclamped = 402
        let out = s.update(2.0, 0.0, 4.0, 0.001);
        approx(out, 10.0);  // clamped to output_limit
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
