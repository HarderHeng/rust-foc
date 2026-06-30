//! Position loop — PI + optional feedforward → ω reference.
//!
//! ## Cascaded control
//!
//! ```text
//! position_ref ──┐
//!                │
//! position_fdb ──┴─→ [PI]──┬─→ omega_ref ──→ speed loop
//!                         │
//! ff_callback ────────────┘   (optional, injected as fn pointer)
//! ```
//!
//! The feedforward callback receives the raw reference, measurement, velocity,
//! and acceleration so it can implement any model.  Set `ff_callback = None`
//! to disable.
//!
//! ## Usage
//!
//! ```ignore
//! fn my_ff(pos_ref: f32, _pos: f32, vel: f32, accel: f32) -> f32 {
//!     inertia_viscous(0.001, 0.0001, accel, vel)
//! }
//!
//! let mut pos = PositionLoopController::default();
//! pos.pid.kp = 10.0;
//! pos.ff_callback = Some(my_ff);
//! let omega_ref = pos.update(target_pos, measured_pos, measured_vel, measured_accel, dt);
//! ```

use crate::math::pid::{self, Pid};

/// Feedforward callback for the position loop.
///
/// Arguments: `(position_ref, measured_position, measured_velocity, measured_accel)`.
/// Return: feedforward contribution in rad/s (added to PI output, then
/// clamped to `pid.output_limit`).
pub type PositionFfFn = fn(position_ref: f32, position: f32, velocity: f32, accel: f32) -> f32;

/// Intermediate values for logging / VOFA / debug.
#[derive(Default, Clone, Copy)]
pub struct Runtime {
    pub pi_output: f32,
    pub ff_total: f32,
    pub position_measured: f32,
}

/// Position loop controller — PI + optional feedforward → ω ref (rad/s).
pub struct PositionLoopController {
    pub pid: Pid,
    /// Optional feedforward callback.  `None` disables feedforward.
    pub ff_callback: Option<PositionFfFn>,
    pub runtime: Runtime,
}

impl Default for PositionLoopController {
    fn default() -> Self {
        let mut pid = Pid::new();
        pid.output_limit = 100.0; // ω upper bound (rad/s)
        Self {
            pid,
            ff_callback: None,
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

        let ff = match self.ff_callback {
            Some(f) => f(position_ref, position, velocity, accel),
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
    fn pi_only_when_no_callback() {
        let mut p = with_pid(1.0, 0.0, 10.0);
        p.ff_callback = None;
        let out = p.update(1.0, 0.0, 0.0, 0.0, 0.001);
        approx(out, 1.0);
    }

    #[test]
    fn feedforward_inertia_term() {
        fn inertia_ff(_ref: f32, _pos: f32, _vel: f32, accel: f32) -> f32 { 0.5 * accel }

        let mut p = with_pid(0.0, 0.0, 10.0);
        p.ff_callback = Some(inertia_ff);
        let out = p.update(0.0, 0.0, 0.0, 10.0, 0.001);
        approx(out, 5.0);
    }

    #[test]
    fn feedforward_viscous_term() {
        fn viscous_ff(_ref: f32, _pos: f32, vel: f32, _accel: f32) -> f32 { 0.1 * vel }

        let mut p = with_pid(0.0, 0.0, 10.0);
        p.ff_callback = Some(viscous_ff);
        let out = p.update(0.0, 0.0, 50.0, 0.0, 0.001);
        approx(out, 5.0);
    }

    #[test]
    fn pi_and_feedforward_sum() {
        fn inertia_ff(_ref: f32, _pos: f32, _vel: f32, accel: f32) -> f32 { 0.5 * accel }

        let mut p = with_pid(1.0, 0.0, 10.0);
        p.ff_callback = Some(inertia_ff);
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

    #[test]
    fn feedforward_does_not_bypass_clamp() {
        fn huge_ff(_ref: f32, _pos: f32, _vel: f32, accel: f32) -> f32 { 100.0 * accel }

        let mut p = with_pid(1.0, 0.0, 10.0);
        p.ff_callback = Some(huge_ff);
        let out = p.update(2.0, 0.0, 0.0, 4.0, 0.001);
        approx(out, 10.0);  // clamped, not 402
    }

    #[test]
    fn callback_can_use_ref() {
        fn ref_based_ff(pos_ref: f32, _pos: f32, _vel: f32, _accel: f32) -> f32 { 0.5 * pos_ref }

        let mut p = with_pid(0.0, 0.0, 10.0);
        p.ff_callback = Some(ref_based_ff);
        let out = p.update(6.0, 0.0, 0.0, 0.0, 0.001);
        approx(out, 3.0);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
