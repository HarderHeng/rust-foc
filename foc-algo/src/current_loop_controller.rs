//! Current loop — pure transform from current vector to PWM duty.
//!
//! Stateless beyond the two PI integrators.  Application passes the
//! measurements and references directly to `update()`.  No meas / target
//! field copying is needed.

use crate::pid::Pid;
use crate::svpwm::{Duty, Svpwm};
use crate::transforms::{clark_balanced, inv_park, park, Trig};

/// D-axis and Q-axis PI configuration.
#[derive(Clone)]
pub struct AxisPid {
    pub pid: Pid,
}

impl Default for AxisPid {
    fn default() -> Self { Self { pid: Pid::new() } }
}

/// Current loop — owns two PIs, a SVPWM modulator, and the DC bus voltage.
pub struct CurrentLoop {
    pub d: AxisPid,
    pub q: AxisPid,
    pub svpwm: Svpwm,
    pub duty: Duty,
}

impl CurrentLoop {
    pub fn new() -> Self {
        Self { d: AxisPid::default(), q: AxisPid::default(), svpwm: Svpwm::new(24.0), duty: Duty::default() }
    }

    /// One current-loop step.  Returns the new duty.
    pub fn update<T: Trig>(
        &mut self,
        ia: f32, ib: f32, angle: f32,
        id_ref: f32, iq_ref: f32,
        dt: f32,
    ) -> Duty {
        let ab = clark_balanced(ia, ib);
        let dq = park::<T>(ab, angle);
        let vd = self.d.pid.update(id_ref, dq.d, dt);
        let vq = self.q.pid.update(iq_ref, dq.q, dt);
        let v_ab = inv_park::<T>(crate::transforms::Dq { d: vd, q: vq }, angle);
        self.svpwm.update(v_ab.alpha, v_ab.beta);
        self.duty = self.svpwm.duty;
        self.duty
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
}

#[cfg(test)]
fn approx(a: f32, b: f32) {
    assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
}