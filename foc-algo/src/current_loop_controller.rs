//! Current loop — pure transform from current vector to PWM duty.
//!
//! Stateless beyond the two PI integrators and the SVPWM modulator.
//! Measurements and references are passed directly to `update()` — no field
//! copying needed.

use crate::pid::Pid;
use crate::svpwm::{Duty, Svpwm};
use crate::transforms::{clark_balanced, inv_park, park, Trig};

/// Per-cycle diagnostic values for logging / VOFA / debug.
#[derive(Default, Clone, Copy)]
pub struct Runtime {
    pub id: f32,
    pub iq: f32,
    pub vd: f32,
    pub vq: f32,
    pub pi_d_output: f32,
    pub pi_q_output: f32,
    pub duty: Duty,
}

/// Current loop — owns two PIs and a SVPWM modulator.
pub struct CurrentLoop {
    pub d_pid: Pid,
    pub q_pid: Pid,
    /// Per-cycle diagnostics: d/q currents, voltages, PI contributions.
    pub runtime: Runtime,
    svpwm: Svpwm,
    vdc: f32,
}

impl CurrentLoop {
    #[must_use]
    pub fn new() -> Self {
        Self {
            d_pid: Pid::new(),
            q_pid: Pid::new(),
            runtime: Runtime::default(),
            svpwm: Svpwm::new(24.0),
            vdc: 24.0,
        }
    }

    /// Update the bus voltage.  Call once per cycle before [`update`](Self::update),
    /// or whenever Vdc changes.
    pub fn set_vdc(&mut self, vdc: f32) {
        self.vdc = vdc;
    }

    /// One current-loop step.  Returns the new duty cycles and writes
    /// diagnostics into `self.runtime`.
    #[allow(clippy::similar_names)]
    #[inline]
    pub fn update<T: Trig>(
        &mut self,
        ia: f32, ib: f32, angle: f32,
        id_ref: f32, iq_ref: f32,
        dt: f32,
    ) -> Duty {
        self.svpwm.vdc = self.vdc;
        let ab = clark_balanced(ia, ib);
        let dq = park::<T>(ab, angle);
        self.runtime.id = dq.d;
        self.runtime.iq = dq.q;

        let vd = self.d_pid.update(id_ref, dq.d, dt);
        let vq = self.q_pid.update(iq_ref, dq.q, dt);
        self.runtime.pi_d_output = vd;
        self.runtime.pi_q_output = vq;
        self.runtime.vd = vd;
        self.runtime.vq = vq;

        let v_ab = inv_park::<T>(crate::transforms::Dq { d: vd, q: vq }, angle);
        self.svpwm.update(v_ab.alpha, v_ab.beta);
        self.runtime.duty = self.svpwm.duty;
        self.svpwm.duty
    }
}

impl Default for CurrentLoop {
    fn default() -> Self { Self::new() }
}

#[cfg(all(test, feature = "libm-trig"))]
mod tests {
    use super::*;
    use crate::transforms::LibmTrig;

    #[test]
    fn zero_state_centred_duty() {
        let mut cl = CurrentLoop::new();
        let d = cl.update::<LibmTrig>(0.0, 0.0, 0.0, 0.0, 0.0, 0.0001);
        approx(d.ta, 0.5);
    }

    #[test]
    fn vdc_zero_safe_output() {
        let mut cl = CurrentLoop::new();
        cl.set_vdc(0.0);
        let d = cl.update::<LibmTrig>(1.0, -0.5, 0.0, 0.0, 0.0, 0.0001);
        approx(d.ta, 0.0);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
