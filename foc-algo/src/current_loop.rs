//! Field-oriented current controller — pure math, no hardware deps.
//!
//! Composes Clarke, Park, two PIDs, inverse Park, and SVPWM into one step:
//!
//! ```text
//! Ia,Ib ─Clarke──→ Iα,Iβ ─Park(θ)──→ Id,Iq ─PID──→ Vd,Vq ─inv_Park(θ)──→ Vα,Vβ ─SVPWM──→ duties
//! ```

use crate::pid::{Pid, PidConfig};
use crate::svpwm::{svpwm, SvpwmDuty};
use crate::transforms::{clark_balanced, inv_park, park, Trig};

/// Current loop configuration.
///
/// D and Q axes typically need different gains (especially with salient
/// PMSMs where Ld ≠ Lq).
pub struct CurrentLoopConfig {
    pub pid_d: PidConfig,
    pub pid_q: PidConfig,
}

/// FOC current controller.
///
/// Each `update()` runs the full chain from phase currents and rotor angle
/// to PWM duty cycles.  No hidden state beyond the two PI accumulators.
pub struct CurrentLoop {
    pid_d: Pid,
    pid_q: Pid,
}

impl CurrentLoop {
    pub fn new(cfg: CurrentLoopConfig) -> Self {
        Self {
            pid_d: Pid::new(cfg.pid_d),
            pid_q: Pid::new(cfg.pid_q),
        }
    }

    /// One control cycle.
    ///
    /// | Arg | Unit | Description |
    /// |-----|------|-------------|
    /// | `id_ref`, `iq_ref` | A | Target currents (from outer speed/torque loop) |
    /// | `ia`, `ib` | A | Measured phase currents (ic is derived as -(ia+ib)) |
    /// | `theta` | rad | Rotor electrical angle |
    /// | `vdc` | V | DC bus voltage |
    /// | `dt` | s | Time since last call |
    ///
    /// Returns SVPWM duty cycles in [0, 1].
    pub fn update<T: Trig>(
        &mut self,
        trig: &T,
        id_ref: f32,
        iq_ref: f32,
        ia: f32,
        ib: f32,
        theta: f32,
        vdc: f32,
        dt: f32,
    ) -> SvpwmDuty {
        // 1. Clarke: balanced Ia,Ib → stationary frame
        let ab = clark_balanced(ia, ib);

        // 2. Park: stationary → rotating frame
        let dq = park(trig, ab, theta);

        // 3. PI on each axis
        let vd = self.pid_d.update(id_ref, dq.d, dt);
        let vq = self.pid_q.update(iq_ref, dq.q, dt);

        // 4. Inverse Park: rotating → stationary
        let v_ab = inv_park(trig, crate::transforms::Dq { d: vd, q: vq }, theta);

        // 5. SVPWM: voltage vector → duty cycles
        svpwm(v_ab.alpha, v_ab.beta, vdc)
    }

    /// Reset both PI controllers.
    pub fn reset(&mut self) {
        self.pid_d.reset();
        self.pid_q.reset();
    }

    pub fn pid_d(&self) -> &Pid { &self.pid_d }
    pub fn pid_q(&self) -> &Pid { &self.pid_q }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transforms::LibmTrig;
    static TRIG: LibmTrig = LibmTrig;

    /// Zero currents, zero reference → 50 % duty on all phases.
    #[test]
    fn zero_inputs_centre_output() {
        let mut cl = CurrentLoop::new(CurrentLoopConfig {
            pid_d: PidConfig { kp: 1.0, ki: 0.1, kd: 0.0, output_limit: 12.0, d_filter_cycles: 0 },
            pid_q: PidConfig { kp: 1.0, ki: 0.1, kd: 0.0, output_limit: 12.0, d_filter_cycles: 0 },
        });
        let d = cl.update(&TRIG, 0.0, 0.0, 0.0, 0.0, 0.0, 24.0, 0.0001);
        approx(d.ta, 0.5);
        approx(d.tb, 0.5);
        approx(d.tc, 0.5);
    }

    /// A non-zero Iq reference with matching measured current → steady output.
    #[test]
    fn tracking_steady_state() {
        let mut cl = CurrentLoop::new(CurrentLoopConfig {
            pid_d: PidConfig { kp: 2.0, ki: 0.5, kd: 0.0, output_limit: 12.0, d_filter_cycles: 0 },
            pid_q: PidConfig { kp: 2.0, ki: 0.5, kd: 0.0, output_limit: 12.0, d_filter_cycles: 0 },
        });
        // θ=0, Ia=1, Ib=-0.5 → Ic=-0.5 → Id=1, Iq=0
        let d = cl.update(&TRIG, 1.0, 0.0, 1.0, -0.5, 0.0, 24.0, 0.0001);
        // Error is 0 on both axes → output stays at 50% (integrator starts from 0)
        approx(d.ta, 0.5);
        approx(d.tb, 0.5);
        approx(d.tc, 0.5);
    }

    /// Step response: after several updates, the controller should drive
    /// the output away from centre.
    #[test]
    fn step_response_integrates() {
        let mut cl = CurrentLoop::new(CurrentLoopConfig {
            pid_d: PidConfig { kp: 1.0, ki: 0.1, kd: 0.0, output_limit: 12.0, d_filter_cycles: 0 },
            pid_q: PidConfig { kp: 1.0, ki: 0.1, kd: 0.0, output_limit: 12.0, d_filter_cycles: 0 },
        });
        // Ten steps with Iq error = 1 A
        for _ in 0..10 {
            cl.update(&TRIG, 0.0, 1.0, 0.0, 0.0, 0.0, 24.0, 0.0001);
        }
        let d = cl.update(&TRIG, 0.0, 1.0, 0.0, 0.0, 0.0, 24.0, 0.0001);
        // Output should have moved away from centre
        assert!(d.ta != 0.5 || d.tb != 0.5 || d.tc != 0.5);
    }

    /// Reset clears the PI accumulators.
    #[test]
    fn reset_clears_integrators() {
        let mut cl = CurrentLoop::new(CurrentLoopConfig {
            pid_d: PidConfig { kp: 1.0, ki: 0.5, kd: 0.0, output_limit: 12.0, d_filter_cycles: 0 },
            pid_q: PidConfig { kp: 1.0, ki: 0.5, kd: 0.0, output_limit: 12.0, d_filter_cycles: 0 },
        });
        for _ in 0..10 {
            cl.update(&TRIG, 0.0, 1.0, 0.0, 0.0, 0.0, 24.0, 0.0001);
        }
        let before_iq = cl.pid_q().integral();
        cl.reset();
        assert!(cl.pid_q().integral().abs() < before_iq.abs());
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
