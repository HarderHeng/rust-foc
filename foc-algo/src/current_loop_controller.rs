//! Current loop controller — context-object with explicit field layering.
//!
//! ## Field layers
//!
//! | Layer | Type | Owner |
//! |-------|------|-------|
//! | `pid_d`, `pid_q` | [`Pid`] | controller owns state, app writes gains |
//! | `svpwm` | [`Svpwm`] | controller owns, app writes `vdc` |
//! | `meas` | [`Measurements`] | application writes per cycle |
//! | `target` | [`Targets`] | application writes per cycle |
//! | `runtime` | [`Runtime`] | controller writes per cycle (debug-visible) |
//! | `duty` | [`Duty`] | controller writes per cycle |
//!
//! ## Usage
//!
//! ```ignore
//! let mut loop = CurrentLoopController::new();
//!
//! loop {
//!     loop.meas.ia = adc.read_a();
//!     loop.meas.ib = adc.read_b();
//!     loop.meas.theta = encoder.angle();
//!     loop.target.iq = torque_request;
//!
//!     loop.update(&trig, dt);
//!
//!     pwm.set(loop.duty);
//! }
//! ```

use crate::pid::Pid;
use crate::svpwm::{Duty, Svpwm};
use crate::transforms::{clark_balanced, inv_park, park, Trig};

/// Sensor readings supplied by the application each cycle.
#[derive(Default, Clone, Copy)]
pub struct Measurements {
    pub ia: f32,
    pub ib: f32,
    pub theta: f32,
}

/// Reference inputs (typically from a speed/torque outer loop).
#[derive(Default, Clone, Copy)]
pub struct Targets {
    pub id: f32,
    pub iq: f32,
}

/// Intermediate values for logging / VOFA / debug.
#[derive(Default, Clone, Copy)]
pub struct Runtime {
    pub id_measured: f32,
    pub iq_measured: f32,
    pub vd: f32,
    pub vq: f32,
}

/// Current loop controller — runs Clarke → Park → two PIs → inv-Park → SVPWM.
pub struct CurrentLoopController {
    pub pid_d: Pid,
    pub pid_q: Pid,
    pub svpwm: Svpwm,

    pub meas: Measurements,
    pub target: Targets,
    pub runtime: Runtime,
    pub duty: Duty,
}

impl Default for CurrentLoopController {
    fn default() -> Self {
        let mut svpwm = Svpwm::new(24.0);
        svpwm.duty = Duty { ta: 0.5, tb: 0.5, tc: 0.5 };
        Self {
            pid_d: Pid::new(),
            pid_q: Pid::new(),
            svpwm,
            meas: Measurements::default(),
            target: Targets::default(),
            runtime: Runtime::default(),
            duty: Duty { ta: 0.5, tb: 0.5, tc: 0.5 },
        }
    }
}

impl CurrentLoopController {
    /// Run one current-loop cycle.  Reads `self.meas` and `self.target`,
    /// writes `self.runtime` and `self.duty`.
    pub fn update<T: Trig>(&mut self, dt: f32) {
        let ab = clark_balanced(self.meas.ia, self.meas.ib);
        let dq = park::<T>(ab, self.meas.theta);
        self.runtime.id_measured = dq.d;
        self.runtime.iq_measured = dq.q;

        let vd = self.pid_d.update(self.target.id, dq.d, dt);
        let vq = self.pid_q.update(self.target.iq, dq.q, dt);
        self.runtime.vd = vd;
        self.runtime.vq = vq;

        let v_ab = inv_park::<T>(crate::transforms::Dq { d: vd, q: vq }, self.meas.theta);
        self.svpwm.update(v_ab.alpha, v_ab.beta);
        self.duty = self.svpwm.duty;
    }

    pub fn reset(&mut self) {
        self.pid_d.reset();
        self.pid_q.reset();
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transforms::LibmTrig;

    #[test]
    fn zero_state_centred_duty() {
        let mut loop_ = CurrentLoopController::default();
        loop_.update::<LibmTrig>(0.0001);
        approx(loop_.duty.ta, 0.5);
        approx(loop_.duty.tb, 0.5);
        approx(loop_.duty.tc, 0.5);
    }

    #[test]
    fn runtime_reflects_measurements() {
        let mut loop_ = CurrentLoopController::default();
        loop_.meas.ia = 1.0;
        loop_.meas.ib = -0.5;
        loop_.meas.theta = 0.0;
        loop_.update::<LibmTrig>(0.0001);
        approx(loop_.runtime.id_measured, 1.0);
        approx(loop_.runtime.iq_measured, 0.0);
    }

    #[test]
    fn step_target_moves_duty() {
        let mut loop_ = CurrentLoopController::default();
        loop_.pid_d.kp = 1.0; loop_.pid_d.ki = 0.1;
        loop_.pid_q.kp = 1.0; loop_.pid_q.ki = 0.1;
        loop_.target.iq = 1.0;
        for _ in 0..10 { loop_.update::<LibmTrig>(0.0001); }
        assert!(loop_.duty.ta != 0.5 || loop_.duty.tb != 0.5 || loop_.duty.tc != 0.5);
    }

    #[test]
    fn reset_clears_integrators() {
        let mut loop_ = CurrentLoopController::default();
        loop_.pid_d.kp = 1.0; loop_.pid_d.ki = 0.5;
        loop_.pid_q.kp = 1.0; loop_.pid_q.ki = 0.5;
        loop_.target.iq = 1.0;
        for _ in 0..10 { loop_.update::<LibmTrig>(0.0001); }
        let before = loop_.pid_q.integral;
        loop_.reset();
        assert!(loop_.pid_q.integral.abs() < before.abs());
    }

    #[test]
    fn vdc_zero_safe_output() {
        let mut loop_ = CurrentLoopController::default();
        loop_.svpwm.vdc = 0.0;
        loop_.update::<LibmTrig>(0.0001);
        approx(loop_.duty.ta, 0.0);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}