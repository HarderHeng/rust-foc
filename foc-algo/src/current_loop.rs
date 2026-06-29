//! FOC controller — context-object with explicit field layering.
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
//! let mut foc = FocController::new();
//!
//! loop {
//!     foc.meas.ia = adc.read_a();
//!     foc.meas.ib = adc.read_b();
//!     foc.meas.theta = encoder.angle();
//!     foc.target.iq = torque_request;
//!
//!     foc.update(&trig, dt);
//!
//!     pwm.set(foc.duty);
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

/// Reference inputs to the current loop (typically from a speed/torque loop).
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

/// FOC current controller. All state lives here.
pub struct FocController {
    pub pid_d: Pid,
    pub pid_q: Pid,
    pub svpwm: Svpwm,

    pub meas: Measurements,
    pub target: Targets,
    pub runtime: Runtime,
    pub duty: Duty,
}

impl Default for FocController {
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

impl FocController {
    /// Run one control cycle.  Reads `self.meas` and `self.target`, writes
    /// `self.runtime` and `self.duty`.
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
        let mut foc = FocController::default();
        foc.update::<LibmTrig>(0.0001);
        approx(foc.duty.ta, 0.5);
        approx(foc.duty.tb, 0.5);
        approx(foc.duty.tc, 0.5);
    }

    #[test]
    fn runtime_reflects_measurements() {
        let mut foc = FocController::default();
        foc.meas.ia = 1.0;
        foc.meas.ib = -0.5;
        foc.meas.theta = 0.0;
        foc.update::<LibmTrig>(0.0001);
        approx(foc.runtime.id_measured, 1.0);
        approx(foc.runtime.iq_measured, 0.0);
    }

    #[test]
    fn step_target_moves_duty() {
        let mut foc = FocController::default();
        foc.pid_d.kp = 1.0; foc.pid_d.ki = 0.1;
        foc.pid_q.kp = 1.0; foc.pid_q.ki = 0.1;
        foc.target.iq = 1.0;
        for _ in 0..10 { foc.update::<LibmTrig>(0.0001); }
        assert!(foc.duty.ta != 0.5 || foc.duty.tb != 0.5 || foc.duty.tc != 0.5);
    }

    #[test]
    fn reset_clears_integrators() {
        let mut foc = FocController::default();
        foc.pid_d.kp = 1.0; foc.pid_d.ki = 0.5;
        foc.pid_q.kp = 1.0; foc.pid_q.ki = 0.5;
        foc.target.iq = 1.0;
        for _ in 0..10 { foc.update::<LibmTrig>(0.0001); }
        let before = foc.pid_q.integral;
        foc.reset();
        assert!(foc.pid_q.integral.abs() < before.abs());
    }

    #[test]
    fn vdc_zero_safe_output() {
        let mut foc = FocController::default();
        foc.svpwm.vdc = 0.0;
        foc.update::<LibmTrig>(0.0001);
        approx(foc.duty.ta, 0.0);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}