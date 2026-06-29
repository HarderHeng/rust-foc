//! Space-Vector PWM — αβ voltage vector → three-phase duty cycles.
//!
//! Uses max/min zero-sequence injection (centred SVM) to extend the linear
//! modulation range to Vdc/√3 without distortion.

/// Three-phase duty cycles in [0, 1].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Duty {
    pub ta: f32,
    pub tb: f32,
    pub tc: f32,
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
    pub fn new(vdc: f32) -> Self {
        Self { vdc, duty: Duty { ta: 0.5, tb: 0.5, tc: 0.5 } }
    }

    /// One modulation step.  Writes `self.duty`.
    ///
    /// If `vdc ≤ 0` (config not yet set), output stays at the safe 0 % duty
    /// across all phases — a hardware-safe fallback.
    pub fn update(&mut self, v_alpha: f32, v_beta: f32) {
        if self.vdc <= 0.0 {
            self.duty = Duty { ta: 0.0, tb: 0.0, tc: 0.0 };
            return;
        }

        // Inverse Clarke via shared transform.
        let abc = crate::inv_clark(crate::AlphaBeta { alpha: v_alpha, beta: v_beta });
        let va = abc.a;
        let vb = abc.b;
        let vc = abc.c;

        // Zero-sequence injection (centred SVM).
        let vmax = va.max(vb).max(vc);
        let vmin = va.min(vb).min(vc);
        let voffset = -0.5 * (vmax + vmin);

        // Duty cycles, clamped to [0, 1] for natural six-step transition.
        let inv_vdc = 1.0 / self.vdc;
        self.duty = Duty {
            ta: (0.5 + (va + voffset) * inv_vdc).clamp(0.0, 1.0),
            tb: (0.5 + (vb + voffset) * inv_vdc).clamp(0.0, 1.0),
            tc: (0.5 + (vc + voffset) * inv_vdc).clamp(0.0, 1.0),
        };
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

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

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}