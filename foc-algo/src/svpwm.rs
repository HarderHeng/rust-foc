//! Space-Vector PWM — αβ voltage vector → three-phase duty cycles.
//!
//! Uses max/min zero-sequence injection (centred SVM) to extend the linear
//! modulation range to Vdc/√3 without distortion.
//!
//! Phase voltages are derived from αβ via inverse Clarke, inlined to avoid the
//! intermediate `Abc` struct and extra function call.

use crate::transforms::HALF_SQRT3;

/// Three-phase duty cycles in [0, 1].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Duty {
    pub ta: f32,
    pub tb: f32,
    pub tc: f32,
}

impl Duty {
    /// Convert [0, 1] duty cycles to timer compare counts.
    ///
    /// `period` is the timer auto-reload value (e.g. `ARR` on STM32).  The
    /// result is clamped to `[0, period]` for safety.
    ///
    /// ```ignore
    /// let duty = current_loop.update::<LibmTrig>(...);
    /// let (ccr_a, ccr_b, ccr_c) = duty.to_timer_counts(7199);  // 7200-count period
    /// ```
    #[must_use]
    pub fn to_timer_counts(self, period: u16) -> (u16, u16, u16) {
        let p = f32::from(period);
        // Clamp duty to [0, 1], scale, then round to nearest integer.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let scale = |d: f32| -> u16 { (d.clamp(0.0, 1.0) * p + 0.5) as u16 };
        (scale(self.ta), scale(self.tb), scale(self.tc))
    }

    /// Apply per-phase dead-time compensation based on current direction.
    ///
    /// Dead-time distorts the output voltage — current flows through the diode
    /// during the dead band, producing a voltage error proportional to
    /// `dead_time_ns / pwm_period_ns`.  This method subtracts the error from
    /// the duty of the phase where current is flowing *out* (positive) and adds
    /// it where current is flowing *in* (negative).
    ///
    /// `dt_ns` is the dead-time in nanoseconds.  `pwm_period_ns` is the PWM
    /// period (`1/f_sw`) also in nanoseconds.  `ia/ib/ic` are the phase
    /// currents (A); their sign determines the compensation direction.
    ///
    /// Call **after** SVPWM modulation, just before writing timer registers.
    ///
    /// ```ignore
    /// let duty = svpwm.duty.apply_dead_time(200, 50_000, ia, ib, ic);
    /// ```
    #[must_use]
    pub fn apply_dead_time(
        mut self,
        dead_time_ns: u32,
        pwm_period_ns: u32,
        ia: f32,
        ib: f32,
        ic: f32,
    ) -> Self {
        if dead_time_ns == 0 || pwm_period_ns == 0 {
            return self;
        }
        #[allow(clippy::cast_precision_loss)]
        let offset = dead_time_ns as f32 / pwm_period_ns as f32;
        // Positive current → subtract duty (current flows out through high-side
        // switch, dead-time creates a negative voltage error).
        // Negative current → add duty (current commutates to low-side diode).
        // Zero/near-zero current → no compensation.
        if ia > 0.0 { self.ta -= offset; } else if ia < 0.0 { self.ta += offset; }
        if ib > 0.0 { self.tb -= offset; } else if ib < 0.0 { self.tb += offset; }
        if ic > 0.0 { self.tc -= offset; } else if ic < 0.0 { self.tc += offset; }
        // Re-clamp to [0, 1] for safety.
        self.ta = self.ta.clamp(0.0, 1.0);
        self.tb = self.tb.clamp(0.0, 1.0);
        self.tc = self.tc.clamp(0.0, 1.0);
        self
    }
}

/// SVPWM modulator — context-object style.
///
/// | Field | Layer | Owner |
/// |-------|-------|-------|
/// | `vdc` | config | application |
/// | `duty` | output | controller |
pub struct Svpwm {
    pub vdc: f32,
    pub duty: Duty,
}

impl Svpwm {
    /// Default with Vdc = 24 V and centred duty.
    #[must_use]
    pub fn new(vdc: f32) -> Self {
        Self { vdc, duty: Duty { ta: 0.5, tb: 0.5, tc: 0.5 } }
    }

    /// One modulation step.  Writes `self.duty`.
    ///
    /// If `vdc ≤ 0` (config not yet set), output stays at the safe 0 % duty
    /// across all phases — a hardware-safe fallback.
    #[inline]
    pub fn update(&mut self, v_alpha: f32, v_beta: f32) {
        if self.vdc <= 0.0 {
            self.duty = Duty { ta: 0.0, tb: 0.0, tc: 0.0 };
            return;
        }

        // Inverse Clarke inlined: va=α, vb=-½α+½√3·β, vc=-½α-½√3·β
        let va = v_alpha;
        let vb = -0.5 * v_alpha + HALF_SQRT3 * v_beta;
        let vc = -0.5 * v_alpha - HALF_SQRT3 * v_beta;

        // Zero-sequence injection (centred SVM).
        let vmax = va.max(vb).max(vc);
        let vmin = va.min(vb).min(vc);
        let voffset = -0.5 * (vmax + vmin);

        let inv_vdc = 1.0 / self.vdc;
        self.duty = Duty {
            ta: (0.5 + (va + voffset) * inv_vdc).clamp(0.0, 1.0),
            tb: (0.5 + (vb + voffset) * inv_vdc).clamp(0.0, 1.0),
            tc: (0.5 + (vc + voffset) * inv_vdc).clamp(0.0, 1.0),
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_vector_centred() {
        let mut m = Svpwm::new(24.0);
        m.update(0.0, 0.0);
        approx(m.duty.ta, 0.5);
        approx(m.duty.tb, 0.5);
        approx(m.duty.tc, 0.5);
    }

    #[test]
    fn vdc_zero_safe_default() {
        let mut m = Svpwm::new(0.0);
        m.update(1.0, 0.0);
        approx(m.duty.ta, 0.0);
        approx(m.duty.tb, 0.0);
        approx(m.duty.tc, 0.0);
    }

    #[test]
    fn linear_limit_in_bounds() {
        let vdc = 24.0;
        let mut m = Svpwm::new(vdc);
        m.update(vdc / 1.732_050_8, 0.0);
        assert!((0.0..=1.0).contains(&m.duty.ta));
        assert!(m.duty.ta > m.duty.tb);
    }

    #[test]
    fn lll_voltage_matches_phase_diff() {
        let mut m = Svpwm::new(24.0);
        m.update(5.0, 0.0);
        let vab = (m.duty.ta - m.duty.tb) * 24.0;
        approx(vab, 7.5);
    }

    #[test]
    fn over_modulation_clamps() {
        let mut m = Svpwm::new(24.0);
        m.update(24.0 * 0.75, 0.0);
        approx(m.duty.ta, 1.0);
        approx(m.duty.tb, 0.0);
    }

    // ── Duty helpers ──

    #[test]
    fn timer_counts_scale() {
        let d = Duty { ta: 0.5, tb: 0.25, tc: 0.75 };
        let (a, b, c) = d.to_timer_counts(7200);
        assert_eq!(a, 3600);
        assert_eq!(b, 1800);
        assert_eq!(c, 5400);
    }

    #[test]
    fn timer_counts_clamped() {
        let d = Duty { ta: -0.1, tb: 1.5, tc: 0.5 };
        let (a, b, c) = d.to_timer_counts(1000);
        assert_eq!(a, 0);
        assert_eq!(b, 1000);
        assert_eq!(c, 500);
    }

    #[test]
    fn dead_time_zero_is_identity() {
        let d = Duty { ta: 0.6, tb: 0.3, tc: 0.8 };
        let dt = d.apply_dead_time(0, 50_000, 1.0, -0.5, 0.2);
        approx(dt.ta, 0.6);
        approx(dt.tb, 0.3);
        approx(dt.tc, 0.8);
    }

    #[test]
    fn dead_time_positive_current_reduces_duty() {
        let d = Duty { ta: 0.5, tb: 0.5, tc: 0.5 };
        let dt = d.apply_dead_time(200, 50_000, 1.0, 0.0, 0.0);
        assert!(dt.ta < 0.5);
        approx(dt.tb, 0.5);
        approx(dt.tc, 0.5);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}