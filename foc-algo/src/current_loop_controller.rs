//! Current loop — pure transform from current vector to PWM duty.
//!
//! Stateless beyond the two PI integrators and the SVPWM modulator.
//! Measurements and references are passed directly to `update()` — no field
//! copying needed.

use crate::pid::Pid;
use crate::svpwm::{Duty, Svpwm};
use crate::transforms::{clark_balanced, inv_park, park, Trig};

/// Current loop — owns two PIs and a SVPWM modulator.
pub struct CurrentLoop {
    pub d_pid: Pid,
    pub q_pid: Pid,
    pub svpwm: Svpwm,
}

impl CurrentLoop {
    pub fn new() -> Self {
        Self {
            d_pid: Pid::new(),
            q_pid: Pid::new(),
            svpwm: Svpwm::new(24.0),
        }
    }

    /// One current-loop step.  Returns the new duty cycles.
    pub fn update<T: Trig>(
        &mut self,
        ia: f32, ib: f32, angle: f32,
        id_ref: f32, iq_ref: f32,
        dt: f32,
    ) -> Duty {
        let ab = clark_balanced(ia, ib);
        let dq = park::<T>(ab, angle);
        let vd = self.d_pid.update(id_ref, dq.d, dt);
        let vq = self.q_pid.update(iq_ref, dq.q, dt);
        let v_ab = inv_park::<T>(crate::transforms::Dq { d: vd, q: vq }, angle);
        self.svpwm.update(v_ab.alpha, v_ab.beta);
        self.svpwm.duty
    }
}

impl Default for CurrentLoop {
    fn default() -> Self { Self::new() }
}

#[cfg(test)]
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
        cl.svpwm.vdc = 0.0;
        let d = cl.update::<LibmTrig>(1.0, -0.5, 0.0, 0.0, 0.0, 0.0001);
        approx(d.ta, 0.0);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
