//! Position loop controller — context-object with explicit field layering.
//!
//! ## Cascaded control
//!
//! ```text
//! position_ref ─┐
//!              │
//! position_fdb ─┴─→ [PI]──┬─→ omega_ref ──→ SpeedLoopController
//!                       │
//! feedforward ──────────┘   (optional feedforward)
//! ```
//!
//! ## Field layers
//!
//! | Layer | Type | Owner |
//! |-------|------|-------|
//! | `pid` | [`Pid`] | controller owns, app writes gains |
//! | `meas` | [`Measurements`] | application writes per cycle |
//! | `target` | [`Targets`] | application writes per cycle |
//! | `feedforward` | [`Feedforward`] | application writes per cycle (gains + accel/vel) |
//! | `runtime` | [`Runtime`] | controller writes per cycle (debug) |
//! | `omega_ref` | `f32` | controller writes per cycle (output) |
//!
//! ## Feedforward (optional)
//!
//! Two configurable feedforward terms added to the PI output:
//!   `omega_ff = inertia_ff_gain · accel + viscous_ff_gain · velocity`
//!
//! Setting both gains to 0 (or `enabled = false`) disables feedforward.
//!
//! ## Usage
//!
//! ```ignore
//! let mut pos = PositionLoopController::new();
//! pos.meas.position = encoder.position();
//! pos.meas.velocity = encoder.velocity();
//! pos.meas.accel = encoder.acceleration();
//! pos.target.position_ref = setpoint;
//!
//! pos.update(dt);
//! speed_loop.target.speed_ref = pos.omega_ref;
//! ```

use crate::pid::Pid;

/// Sensor readings supplied by the application each cycle.
#[derive(Default, Clone, Copy)]
pub struct Measurements {
    /// Mechanical position (rad).
    pub position: f32,
    /// Mechanical velocity (rad/s).  Used by viscous feedforward.
    pub velocity: f32,
    /// Mechanical acceleration (rad/s²).  Used by inertia feedforward.
    pub accel: f32,
}

/// Reference inputs.
#[derive(Default, Clone, Copy)]
pub struct Targets {
    /// Target position (rad).
    pub position_ref: f32,
}

/// Feedforward gains.  Set to 0 to disable (or use `enabled = false`).
#[derive(Clone, Copy)]
pub struct Feedforward {
    /// Inertia compensation: omega_ff_inertia = inertia_gain × accel
    pub inertia_gain: f32,
    /// Viscous friction compensation: omega_ff_viscous = viscous_gain × velocity
    pub viscous_gain: f32,
    /// Master enable (false → all feedforward terms = 0).
    pub enabled: bool,
}

impl Default for Feedforward {
    fn default() -> Self {
        Self { inertia_gain: 0.0, viscous_gain: 0.0, enabled: true }
    }
}

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
    pub meas: Measurements,
    pub target: Targets,
    pub feedforward: Feedforward,
    pub runtime: Runtime,

    /// Speed reference for the speed loop (rad/s).
    pub omega_ref: f32,
}

impl Default for PositionLoopController {
    fn default() -> Self {
        let mut pid = Pid::new();
        pid.output_limit = 100.0; // ω upper bound (rad/s)
        Self {
            pid,
            meas: Measurements::default(),
            target: Targets::default(),
            feedforward: Feedforward::default(),
            runtime: Runtime::default(),
            omega_ref: 0.0,
        }
    }
}

impl PositionLoopController {
    /// Run one position-loop cycle.  Reads `self.meas` and `self.target`,
    /// writes `self.omega_ref` and `self.runtime`.
    pub fn update(&mut self, dt: f32) {
        let pi_out = self.pid.update(self.target.position_ref, self.meas.position, dt);
        self.runtime.pi_output = pi_out;
        self.runtime.position_measured = self.meas.position;

        if self.feedforward.enabled {
            self.runtime.ff_inertia = self.feedforward.inertia_gain * self.meas.accel;
            self.runtime.ff_viscous = self.feedforward.viscous_gain * self.meas.velocity;
        } else {
            self.runtime.ff_inertia = 0.0;
            self.runtime.ff_viscous = 0.0;
        }
        self.runtime.ff_total = self.runtime.ff_inertia + self.runtime.ff_viscous;

        self.omega_ref = pi_out + self.runtime.ff_total;
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
        p.update(0.001);
        approx(p.omega_ref, 0.0);
    }

    #[test]
    fn pi_only_when_feedforward_disabled() {
        let mut p = with_pid(1.0, 0.0, 10.0);
        p.feedforward.enabled = false;
        p.feedforward.inertia_gain = 100.0;
        p.feedforward.viscous_gain = 100.0;
        p.target.position_ref = 1.0;
        p.update(0.001);
        approx(p.omega_ref, 1.0);
    }

    #[test]
    fn feedforward_inertia_term() {
        let mut p = with_pid(0.0, 0.0, 10.0);
        p.feedforward.inertia_gain = 0.5;
        p.meas.accel = 10.0;
        p.update(0.001);
        approx(p.omega_ref, 5.0);
    }

    #[test]
    fn feedforward_viscous_term() {
        let mut p = with_pid(0.0, 0.0, 10.0);
        p.feedforward.viscous_gain = 0.1;
        p.meas.velocity = 50.0;
        p.update(0.001);
        approx(p.omega_ref, 5.0);
    }

    #[test]
    fn pi_and_feedforward_sum() {
        let mut p = with_pid(1.0, 0.0, 10.0);
        p.feedforward.inertia_gain = 0.5;
        p.target.position_ref = 2.0;
        p.meas.accel = 4.0;
        p.update(0.001);
        // P = 1 × 2 = 2, FF = 0.5 × 4 = 2, total = 4
        approx(p.omega_ref, 4.0);
    }

    #[test]
    fn integrator_accumulates() {
        let mut p = with_pid(0.0, 1.0, 10.0);
        p.target.position_ref = 1.0;
        for _ in 0..5 { p.update(0.1); }
        approx(p.pid.integral, 0.5);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}