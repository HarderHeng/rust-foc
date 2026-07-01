//! Current loop — pure transform from current vector to PWM duty.
//!
//! Stateless beyond the two PI integrators and the SVPWM modulator.
//! Measurements and references are passed directly to `update()` — no field
//! copying needed.

use crate::math::circle_limitation::circle_limitation;
use crate::math::pid::Pid;
use crate::math::svpwm::{Duty, Svpwm};
use crate::math::transforms::{clark_balanced, inv_park, park, Trig};

/// Per-cycle diagnostic values for logging / VOFA / debug.
#[derive(Default, Clone, Copy)]
pub struct Runtime {
    pub id: f32,
    pub iq: f32,
    /// Applied d-axis voltage (V) — PI output + feedforward.  This is what
    /// enters SVPWM, and what the sensorless observer should consume.
    pub vd: f32,
    /// Applied q-axis voltage (V) — PI output + feedforward.
    pub vq: f32,
    pub pi_d_output: f32,
    pub pi_q_output: f32,
    /// Feedforward d-axis voltage (V), 0 when decoupling disabled.
    pub vd_ff: f32,
    /// Feedforward q-axis voltage (V), 0 when decoupling disabled.
    pub vq_ff: f32,
    /// True when the voltage-domain circle limit clamped (vd, vq) this cycle.
    pub voltage_limited: bool,
    pub duty: Duty,
}

/// Current loop — owns two PIs and a SVPWM modulator.
pub struct CurrentLoop {
    pub d_pid: Pid,
    pub q_pid: Pid,
    /// Per-cycle diagnostics: d/q currents, voltages, PI contributions.
    pub runtime: Runtime,
    svpwm: Svpwm,
    /// Maximum voltage vector magnitude (V).  Applied after PI + FF, before
    /// SVPWM, to keep the modulator in its linear region.  `f32::MAX` (the
    /// default) disables the limit.
    ///
    /// Typical: `vdc / √3` — the linear-modulation boundary for centred SVM.
    /// Set via [`set_v_max`](Self::set_v_max) once at init or whenever Vdc
    /// changes.
    v_max: f32,
}

impl CurrentLoop {
    #[must_use]
    pub fn new() -> Self {
        Self {
            d_pid: Pid::new(),
            q_pid: Pid::new(),
            runtime: Runtime::default(),
            // 0 V default — `set_vdc` must be called before `update` for any
            // meaningful output.  When Vdc = 0, the SVPWM duty collapses to 0
            // for safety.
            svpwm: Svpwm::new(0.0),
            v_max: f32::MAX,
        }
    }

    /// Update the bus voltage.  Call once per cycle before [`update`](Self::update),
    /// or whenever Vdc changes.
    pub fn set_vdc(&mut self, vdc: f32) {
        self.svpwm.set_vdc(vdc);
    }

    /// Set the maximum applied voltage magnitude (V) for the voltage-domain
    /// circle limit.  Pass `f32::MAX` to disable.  Typical: `vdc / √3`.
    /// `0` or any negative value also disables.
    pub fn set_v_max(&mut self, v_max: f32) {
        self.v_max = if v_max > 0.0 { v_max } else { f32::MAX };
    }

    /// One current-loop step.  Returns the new duty cycles and writes
    /// diagnostics into `self.runtime`.
    ///
    /// `vd_ff` / `vq_ff` are feedforward decoupling voltages (V), added to
    /// the PI outputs before SVPWM.  Pass `0.0` for both to disable
    /// decoupling.  See [`crate::math::decoupling_voltage`].
    #[allow(clippy::similar_names, clippy::too_many_arguments)]
    #[inline]
    pub fn update<T: Trig>(
        &mut self,
        ia: f32, ib: f32, angle: f32,
        id_ref: f32, iq_ref: f32,
        vd_ff: f32, vq_ff: f32,
        dt: f32,
    ) -> Duty {
        let ab = clark_balanced(ia, ib);
        let dq = park::<T>(ab, angle);
        self.runtime.id = dq.d;
        self.runtime.iq = dq.q;

        let vd_pi = self.d_pid.update(id_ref, dq.d, dt);
        let vq_pi = self.q_pid.update(iq_ref, dq.q, dt);
        self.runtime.pi_d_output = vd_pi;
        self.runtime.pi_q_output = vq_pi;
        self.runtime.vd_ff = vd_ff;
        self.runtime.vq_ff = vq_ff;
        let mut vd = vd_pi + vd_ff;
        let mut vq = vq_pi + vq_ff;
        // Voltage-domain circle limit: keep SVPWM in its linear region.
        // When disabled (v_max = MAX) this is a pass-through.
        if self.v_max < f32::MAX {
            let (vd_c, vq_c) = circle_limitation(vd, vq, self.v_max);
            self.runtime.voltage_limited = vd_c != vd || vq_c != vq;
            vd = vd_c;
            vq = vq_c;
        } else {
            self.runtime.voltage_limited = false;
        }
        self.runtime.vd = vd;
        self.runtime.vq = vq;

        let v_ab = inv_park::<T>(crate::math::transforms::Dq { d: vd, q: vq }, angle);
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
    use crate::math::transforms::LibmTrig;

    #[test]
    fn zero_state_centred_duty() {
        let mut cl = CurrentLoop::new();
        cl.set_vdc(24.0);
        let d = cl.update::<LibmTrig>(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001);
        approx(d.ta, 0.5);
    }

    #[test]
    fn vdc_zero_safe_output() {
        let mut cl = CurrentLoop::new();
        cl.set_vdc(0.0);
        let d = cl.update::<LibmTrig>(1.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001);
        approx(d.ta, 0.0);
    }

    #[test]
    fn feedforward_adds_to_pi_output() {
        let mut cl = CurrentLoop::new();
        cl.set_vdc(24.0);
        // No PI excitation (refs = measured = 0), so PI output ≈ 0.
        // FF should land directly on applied voltage.
        cl.update::<LibmTrig>(0.0, 0.0, 0.0, 0.0, 0.0, 1.5, -2.5, 0.0001);
        approx(cl.runtime.pi_d_output, 0.0);
        approx(cl.runtime.pi_q_output, 0.0);
        approx(cl.runtime.vd_ff,  1.5);
        approx(cl.runtime.vq_ff, -2.5);
        approx(cl.runtime.vd,  1.5);
        approx(cl.runtime.vq, -2.5);
    }

    #[test]
    fn voltage_limit_disabled_passes_through() {
        let mut cl = CurrentLoop::new();
        cl.set_vdc(24.0);
        // No v_max set → unlimited.  Apply 30V through FF (well past vdc).
        cl.update::<LibmTrig>(0.0, 0.0, 0.0, 0.0, 0.0, 20.0, 20.0, 0.0001);
        approx(cl.runtime.vd, 20.0);
        approx(cl.runtime.vq, 20.0);
        assert!(!cl.runtime.voltage_limited);
    }

    #[test]
    fn voltage_limit_scales_to_boundary() {
        let mut cl = CurrentLoop::new();
        cl.set_vdc(24.0);
        // Linear-modulation bound for vdc=24: 24/√3 ≈ 13.8564
        let v_lim = 24.0 * crate::math::transforms::INV_SQRT_3;
        cl.set_v_max(v_lim);
        // Request |Vdq| = √(900+900) ≈ 42.4 V — way over.
        cl.update::<LibmTrig>(0.0, 0.0, 0.0, 0.0, 0.0, 30.0, 30.0, 0.0001);
        approx(cl.runtime.vd, v_lim * (1.0 / core::f32::consts::SQRT_2));
        approx(cl.runtime.vq, v_lim * (1.0 / core::f32::consts::SQRT_2));
        assert!(cl.runtime.voltage_limited);
        // Magnitude must land exactly on v_lim (within fp rounding).
        let mag = (cl.runtime.vd.powi(2) + cl.runtime.vq.powi(2)).sqrt();
        assert!((mag - v_lim).abs() < 1e-3, "mag={mag}, want {v_lim}");
    }

    #[test]
    fn voltage_limit_no_op_inside_circle() {
        let mut cl = CurrentLoop::new();
        cl.set_vdc(24.0);
        cl.set_v_max(100.0);  // generous limit
        cl.update::<LibmTrig>(0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 5.0, 0.0001);
        approx(cl.runtime.vd, 5.0);
        approx(cl.runtime.vq, 5.0);
        assert!(!cl.runtime.voltage_limited);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
