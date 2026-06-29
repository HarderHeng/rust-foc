//! Speed loop controller — context-object with explicit field layering.
//!
//! ## Cascaded control
//!
//! ```text
//! speed_ref  ──┐
//!              │
//! speed_fdb ───┴─→ [PI]──┬─→ iq_target ──→ CurrentLoopController.update()
//!                       │
//! accel_ff ─────────────┤   (optional feedforward)
//!                       │
//! viscous_ff ───────────┘   (optional feedforward)
//! ```
//!
//! ## Field layers
//!
//! | Layer | Type | Owner |
//! |-------|------|-------|
//! | `pid` | [`Pid`] | controller owns, app writes gains |
//! | `meas` | [`Measurements`] | application writes per cycle |
//! | `target` | [`Targets`] | application writes per cycle |
//! | `feedforward` | [`Feedforward`] | application writes per cycle (gains + accel/viscous) |
//! | `runtime` | [`Runtime`] | controller writes per cycle (debug) |
//! | `iq_target` | `f32` | controller writes per cycle (output) |
//!
//! ## Feedforward (optional)
//!
//! Two configurable feedforward terms are added to the PI output:
//!   `iq_ff = inertia_gain · accel_ref + viscous_gain · speed_ref`
//!
//! Setting both gains to 0 disables feedforward entirely — equivalent to a
//! pure PI controller.  This keeps the API the same regardless of whether
//! the user enables it.
//!
//! ## Usage
//!
//! ```ignore
//! let mut speed = SpeedLoopController::new();
//! speed.meas.speed = encoder.speed();
//! speed.meas.accel = encoder.acceleration();
//! speed.target.speed_ref = user_setpoint;
//! speed.feedforward.inertia_gain = 0.001;
//! speed.feedforward.viscous_gain = 0.0005;
//! speed.update(&trig, dt);
//! current_loop.target.iq = speed.iq_target;
//! current_loop.update(&trig, dt);
//! ```

use crate::pid::Pid;

/// Sensor readings supplied by the application each cycle.
#[derive(Default, Clone, Copy)]
pub struct Measurements {
    /// Mechanical speed (rad/s).
    pub speed: f32,
    /// Mechanical acceleration (rad/s²), used by feedforward.
    /// Set to 0 if not available — feedforward term will be inert-only.
    pub accel: f32,
}

/// Reference inputs.
#[derive(Default, Clone, Copy)]
pub struct Targets {
    /// Target speed (rad/s).
    pub speed_ref: f32,
}

/// Feedforward gains.  Set to 0 to disable.
#[derive(Clone, Copy)]
pub struct Feedforward {
    /// Inertia compensation: iq_ff_inertia = inertia_gain × accel
    pub inertia_gain: f32,
    /// Viscous friction compensation: iq_ff_viscous = viscous_gain × speed
    pub viscous_gain: f32,
    /// Master enable (false → all feedforward terms = 0 regardless of gains).
    /// Useful for one-line disable / reconfigure without touching each gain.
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
    pub speed_measured: f32,
}

/// Speed loop controller — PI + optional feedforward → Iq target.
pub struct SpeedLoopController {
    pub pid: Pid,
    pub meas: Measurements,
    pub target: Targets,
    pub feedforward: Feedforward,
    pub runtime: Runtime,

    /// Iq reference for the current loop.  Application reads this and feeds
    /// it into `CurrentLoopController::target.iq`.
    pub iq_target: f32,
}

impl Default for SpeedLoopController {
    fn default() -> Self {
        let mut pid = Pid::new();
        pid.output_limit = 30.0; // Iq upper bound (A)
        Self {
            pid,
            meas: Measurements::default(),
            target: Targets::default(),
            feedforward: Feedforward::default(),
            runtime: Runtime::default(),
            iq_target: 0.0,
        }
    }
}

impl SpeedLoopController {
    /// Run one speed-loop cycle.  Reads `self.meas` and `self.target`,
    /// writes `self.iq_target` and `self.runtime`.
    pub fn update(&mut self, dt: f32) {
        // PI
        let pi_out = self.pid.update(self.target.speed_ref, self.meas.speed, dt);
        self.runtime.pi_output = pi_out;
        self.runtime.speed_measured = self.meas.speed;

        // Optional feedforward
        if self.feedforward.enabled {
            self.runtime.ff_inertia = self.feedforward.inertia_gain * self.meas.accel;
            self.runtime.ff_viscous = self.feedforward.viscous_gain * self.meas.speed;
        } else {
            self.runtime.ff_inertia = 0.0;
            self.runtime.ff_viscous = 0.0;
        }
        self.runtime.ff_total = self.runtime.ff_inertia + self.runtime.ff_viscous;

        // Combined output
        self.iq_target = pi_out + self.runtime.ff_total;
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
        s.update(0.001);
        approx(s.iq_target, 0.0);
    }

    #[test]
    fn pi_only_when_feedforward_disabled() {
        let mut s = with_pid(1.0, 0.0, 0.0, 10.0);
        s.feedforward.enabled = false;
        s.feedforward.inertia_gain = 100.0;   // would otherwise contribute a lot
        s.feedforward.viscous_gain = 100.0;
        s.target.speed_ref = 2.0;
        s.meas.speed = 0.0;
        s.update(0.001);
        // Pure P: kp × (ref − meas) = 1.0 × 2.0 = 2.0
        approx(s.iq_target, 2.0);
    }

    #[test]
    fn feedforward_inertia_term() {
        let mut s = with_pid(0.0, 0.0, 0.0, 10.0); // no PI contribution
        s.feedforward.inertia_gain = 0.5;
        s.meas.accel = 10.0;  // 10 rad/s²
        s.update(0.001);
        // FF: 0.5 × 10 = 5.0
        approx(s.runtime.ff_inertia, 5.0);
        approx(s.iq_target, 5.0);
    }

    #[test]
    fn feedforward_viscous_term() {
        let mut s = with_pid(0.0, 0.0, 0.0, 10.0);
        s.feedforward.viscous_gain = 0.1;
        s.meas.speed = 50.0;  // 50 rad/s
        s.update(0.001);
        // FF: 0.1 × 50 = 5.0
        approx(s.runtime.ff_viscous, 5.0);
    }

    #[test]
    fn pi_and_feedforward_sum() {
        let mut s = with_pid(1.0, 0.0, 0.0, 10.0);
        s.feedforward.inertia_gain = 0.5;
        s.feedforward.viscous_gain = 0.0;
        s.target.speed_ref = 2.0;  // → P = 2.0
        s.meas.speed = 0.0;
        s.meas.accel = 4.0;        // → FF_inertia = 0.5 × 4 = 2.0
        s.update(0.001);
        // Total = 2.0 + 2.0 = 4.0
        approx(s.iq_target, 4.0);
        approx(s.runtime.ff_total, 2.0);
    }

    #[test]
    fn integrator_accumulates() {
        let mut s = with_pid(0.0, 1.0, 0.0, 10.0);
        s.target.speed_ref = 1.0;
        for _ in 0..5 { s.update(0.1); }
        // I = 1.0 × 1.0 × 0.5 = 0.5
        approx(s.pid.integral, 0.5);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}