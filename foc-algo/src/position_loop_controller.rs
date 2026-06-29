//! Position loop — PI + optional feedforward → ω reference.
//!
//! ## Cascaded control
//!
//! ```text
//! position_ref ──┐
//!                │
//! position_fdb ──┴─→ [PI]──┬─→ omega_ref ──→ speed loop
//!                         │
//! accel ──────────────────┤   (inertia feedforward)
//! velocity ───────────────┘   (viscous feedforward)
//! ```
//!
//! ## Usage
//!
//! ```ignore
//! let mut pos = PositionLoopController::default();
//! pos.pid.kp = 10.0;
//! pos.feedforward.inertia_gain = 0.001;
//! let omega_ref = pos.update(target_pos, measured_pos, measured_vel, measured_accel, dt);
//! // feed omega_ref into the speed loop
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
    pub position_measured: f32,
}

/// Position loop controller — PI + optional feedforward → ω ref (rad/s).
pub struct PositionLoopController {
    pub pid: Pid,
    pub feedforward: Feedforward,
    pub runtime: Runtime,
}

impl Default for PositionLoopController {
    fn default() -> Self {
        let mut pid = Pid::new();
        pid.output_limit = 100.0; // ω upper bound (rad/s)
        Self {
            pid,
            feedforward: Feedforward::default(),
            runtime: Runtime::default(),
        }
    }
}

impl PositionLoopController {
    /// Run one position-loop cycle.
    ///
    /// Returns the speed reference (rad/s) for the speed loop and writes debug
    /// values into `self.runtime`.
    pub fn update(
        &mut self,
        position_ref: f32, position: f32,
        velocity: f32, accel: f32,
        dt: f32,
    ) -> f32 {
        let pi_out = self.pid.update(position_ref, position, dt);
        self.runtime.pi_output = pi_out;
        self.runtime.position_measured = position;

        let (fi, fv, ft) = self.feedforward.compute(accel, velocity);
        self.runtime.ff_inertia = fi;
        self.runtime.ff_viscous = fv;
        self.runtime.ff_total = ft;

        pi_out + ft
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

    fn with_pid(kp: f32, ki: f32, limit: f32) -> PositionLoopController {
        let mut p = PositionLoopController::default();
        p.pid.kp = kp; p.pid.ki = ki;
        p.pid.output_limit = limit;
        p
    }

    #[test]
    fn zero_state_zero_output() {
        let mut p = with_pid(1.0, 0.0, 10.0);
        let out = p.update(0.0, 0.0, 0.0, 0.0, 0.001);
        approx(out, 0.0);
    }

    #[test]
    fn pi_only_when_feedforward_disabled() {
        let mut p = with_pid(1.0, 0.0, 10.0);
        p.feedforward.enabled = false;
        p.feedforward.inertia_gain = 100.0;
        p.feedforward.viscous_gain = 100.0;
        let out = p.update(1.0, 0.0, 0.0, 0.0, 0.001);
        approx(out, 1.0);
    }

    #[test]
    fn feedforward_inertia_term() {
        let mut p = with_pid(0.0, 0.0, 10.0);
        p.feedforward.inertia_gain = 0.5;
        let out = p.update(0.0, 0.0, 0.0, 10.0, 0.001);
        approx(out, 5.0);
    }

    #[test]
    fn feedforward_viscous_term() {
        let mut p = with_pid(0.0, 0.0, 10.0);
        p.feedforward.viscous_gain = 0.1;
        let out = p.update(0.0, 0.0, 50.0, 0.0, 0.001);
        approx(out, 5.0);
    }

    #[test]
    fn pi_and_feedforward_sum() {
        let mut p = with_pid(1.0, 0.0, 10.0);
        p.feedforward.inertia_gain = 0.5;
        // P = 1 × 2 = 2, FF = 0.5 × 4 = 2, total = 4
        let out = p.update(2.0, 0.0, 0.0, 4.0, 0.001);
        approx(out, 4.0);
    }

    #[test]
    fn integrator_accumulates() {
        let mut p = with_pid(0.0, 1.0, 10.0);
        for _ in 0..5 { p.update(1.0, 0.0, 0.0, 0.0, 0.1); }
        approx(p.pid.integral, 0.5);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
