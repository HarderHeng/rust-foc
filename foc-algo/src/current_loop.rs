//! FOC controller — context-object style.
//!
//! All state lives inside [`FocController`]. Callers mutate inputs via setters
//! and step the loop with one call.  No long parameter lists per update.
//!
//! Typical usage:
//! ```ignore
//! let mut foc = FocController::new(cfg);
//! loop {
//!     foc.set_currents(ia, ib);             // from ADC
//!     foc.set_target_torque(iq_target);     // from outer loop / user
//!     foc.set_vdc(bus_voltage);             // from ADC
//!     foc.update(theta, dt);                // runs full chain → SVPWM
//!     let duty = foc.duty();                 // push to PWM hardware
//! }
//! ```

use crate::pid::{Pid, PidConfig};
use crate::svpwm::{svpwm, SvpwmDuty};
use crate::transforms::{clark_balanced, inv_park, park, Trig};

/// Controller configuration.
#[derive(Clone, Copy)]
pub struct FocConfig {
    pub pid_d: PidConfig,
    pub pid_q: PidConfig,
    /// DC bus voltage (V). Set once via [`FocController::set_vdc`] or directly.
    pub vdc: f32,
}

impl Default for FocConfig {
    fn default() -> Self {
        Self {
            pid_d: PidConfig { kp: 0.0, ki: 0.0, kd: 0.0, output_limit: 12.0, d_filter_cycles: 0 },
            pid_q: PidConfig { kp: 0.0, ki: 0.0, kd: 0.0, output_limit: 12.0, d_filter_cycles: 0 },
            vdc: 24.0,
        }
    }
}

/// FOC controller with full internal state.
pub struct FocController {
    cfg: FocConfig,
    pid_d: Pid,
    pid_q: Pid,

    // Inputs (last set by setters, consumed by update)
    ia: f32,
    ib: f32,
    theta: f32,

    // Targets
    id_target: f32,
    iq_target: f32,

    // Output
    duty: SvpwmDuty,
    vd: f32,
    vq: f32,
}

impl FocController {
    pub fn new(cfg: FocConfig) -> Self {
        Self {
            cfg,
            pid_d: Pid::new(cfg.pid_d),
            pid_q: Pid::new(cfg.pid_q),
            ia: 0.0, ib: 0.0, theta: 0.0,
            id_target: 0.0, iq_target: 0.0,
            duty: SvpwmDuty { ta: 0.5, tb: 0.5, tc: 0.5 },
            vd: 0.0, vq: 0.0,
        }
    }

    // ── Setters (called by the application before each update) ──

    /// Set measured phase currents (A).  `Ic` is derived as `-(ia + ib)`.
    pub fn set_currents(&mut self, ia: f32, ib: f32) {
        self.ia = ia; self.ib = ib;
    }

    /// Set target currents (A) — typically from a torque / speed controller.
    pub fn set_targets(&mut self, id: f32, iq: f32) {
        self.id_target = id; self.iq_target = iq;
    }

    /// Set DC bus voltage (V).
    pub fn set_vdc(&mut self, vdc: f32) { self.cfg.vdc = vdc; }

    /// Set rotor electrical angle (rad) and step the controller.
    ///
    /// Combining the angle set with `update()` avoids the pitfall of using a
    /// stale angle when the timer interrupt fires between two calls.
    pub fn update<T: Trig>(&mut self, trig: &T, theta: f32, dt: f32) {
        self.theta = theta;

        // Clarke → Park
        let ab = clark_balanced(self.ia, self.ib);
        let dq = park(trig, ab, theta);

        // PI
        self.vd = self.pid_d.update(self.id_target, dq.d, dt);
        self.vq = self.pid_q.update(self.iq_target, dq.q, dt);

        // inv-Park → SVPWM
        let v_ab = inv_park(trig, crate::transforms::Dq { d: self.vd, q: self.vq }, theta);
        self.duty = svpwm(v_ab.alpha, v_ab.beta, self.cfg.vdc);
    }

    // ── Accessors ──

    /// Latest SVPWM duty cycles in [0, 1].
    pub fn duty(&self) -> SvpwmDuty { self.duty }

    /// Latest Id/Iq (post-Park, debug / logging).
    pub fn idq(&self) -> (f32, f32) {
        let ab = clark_balanced(self.ia, self.ib);
        let dq = park(&crate::transforms::LibmTrig, ab, self.theta);
        (dq.d, dq.q)
    }

    /// Reset both PI controllers.
    pub fn reset(&mut self) {
        self.pid_d.reset(); self.pid_q.reset();
    }

    /// Direct access to PI controllers for tuning.
    pub fn pid_d(&mut self) -> &mut Pid { &mut self.pid_d }
    pub fn pid_q(&mut self) -> &mut Pid { &mut self.pid_q }

    pub fn config(&self) -> &FocConfig { &self.cfg }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transforms::LibmTrig;
    static TRIG: LibmTrig = LibmTrig;

    #[test]
    fn zero_state_centred_duty() {
        let mut foc = FocController::new(FocConfig::default());
        foc.update(&TRIG, 0.0, 0.0001);
        approx(foc.duty().ta, 0.5);
        approx(foc.duty().tb, 0.5);
        approx(foc.duty().tc, 0.5);
    }

    #[test]
    fn update_consumes_latest_setters() {
        // Set non-zero current before update.
        let mut foc = FocController::new(FocConfig::default());
        foc.set_currents(1.0, -0.5);
        foc.update(&TRIG, 0.0, 0.0001);
        // After update, idq() reflects what was measured.
        let (d, q) = foc.idq();
        approx(d, 1.0);
        approx(q, 0.0);
    }

    #[test]
    fn step_target_moves_duty() {
        let mut foc = FocController::new(FocConfig {
            pid_d: PidConfig { kp: 1.0, ki: 0.1, kd: 0.0, output_limit: 12.0, d_filter_cycles: 0 },
            pid_q: PidConfig { kp: 1.0, ki: 0.1, kd: 0.0, output_limit: 12.0, d_filter_cycles: 0 },
            vdc: 24.0,
        });
        foc.set_targets(0.0, 1.0);
        for _ in 0..10 { foc.update(&TRIG, 0.0, 0.0001); }
        let d = foc.duty();
        assert!(d.ta != 0.5 || d.tb != 0.5 || d.tc != 0.5);
    }

    #[test]
    fn reset_clears_integrators() {
        let mut foc = FocController::new(FocConfig {
            pid_d: PidConfig { kp: 1.0, ki: 0.5, kd: 0.0, output_limit: 12.0, d_filter_cycles: 0 },
            pid_q: PidConfig { kp: 1.0, ki: 0.5, kd: 0.0, output_limit: 12.0, d_filter_cycles: 0 },
            vdc: 24.0,
        });
        foc.set_targets(0.0, 1.0);
        for _ in 0..10 { foc.update(&TRIG, 0.0, 0.0001); }
        let before = foc.pid_q().integral();
        foc.reset();
        assert!(foc.pid_q().integral().abs() < before.abs());
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}