//! Space-Vector PWM — voltage vector → three-phase duty cycles.
//!
//! Pure math, no hardware deps.  `cargo test` runs on host.
//!
//! Uses the max/min zero-sequence injection (centred SVM) to extend the
//! linear modulation range to Vdc/√3 without distortion.

/// Three-phase duty cycles for centre-aligned PWM.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SvpwmDuty {
    pub ta: f32,
    pub tb: f32,
    pub tc: f32,
}

/// Compute SVPWM duty cycles from an αβ voltage vector.
///
/// | Argument | Unit | Range | Description |
/// |----------|------|-------|-------------|
/// | `v_alpha`, `v_beta` | V | Any | Stator voltage vector (inverse-Park output) |
/// | `vdc` | V | > 0 | DC bus voltage |
///
/// Returns duty cycles in **[0, 1]**, suitable for centre-aligned PWM timers.
/// When the vector exceeds the linear range (|v| > Vdc/√3), the output
/// naturally saturates into six-step mode.
///
/// If `vdc ≤ 0`, returns `SvpwmDuty { ta: 0.0, tb: 0.0, tc: 0.0 }` (safe).
pub fn svpwm(v_alpha: f32, v_beta: f32, vdc: f32) -> SvpwmDuty {
    if vdc <= 0.0 {
        return SvpwmDuty::default();
    }

    // ── 1. Inverse Clarke: αβ → raw phase voltages ──
    // Delegated to the shared transform so the math stays in one place.
    let abc = crate::inv_clark(crate::AlphaBeta { alpha: v_alpha, beta: v_beta });
    let va = abc.a;
    let vb = abc.b;
    let vc = abc.c;

    // ── 2. Max/min zero-sequence injection ──
    //     Voffset = −(Vmax + Vmin) / 2
    //
    // This pushes the three phases into the PWM range [−Vdc/2, +Vdc/2]
    // without distorting the line-to-line voltage.
    let vmax = va.max(vb).max(vc);
    let vmin = va.min(vb).min(vc);
    let voffset = -0.5 * (vmax + vmin);

    // ── 3. Duty cycles (centre-aligned) ──
    //     duty = 0.5 + (Vphase + Voffset) / Vdc
    //     Clamped to [0, 1] in case the vector exceeds the linear range
    //     (|v| > Vdc/√3) — the transition to six-step overmodulation.
    let inv_vdc = 1.0 / vdc;
    SvpwmDuty {
        ta: (0.5 + (va + voffset) * inv_vdc).clamp(0.0, 1.0),
        tb: (0.5 + (vb + voffset) * inv_vdc).clamp(0.0, 1.0),
        tc: (0.5 + (vc + voffset) * inv_vdc).clamp(0.0, 1.0),
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Zero vector → all phases at 50 % (centre).
    #[test]
    fn zero_input_centre_all() {
        let d = svpwm(0.0, 0.0, 24.0);
        approx(d.ta, 0.5);
        approx(d.tb, 0.5);
        approx(d.tc, 0.5);
    }

    /// DC bus zero → safe default.
    #[test]
    fn zero_vdc_returns_zero() {
        let d = svpwm(1.0, 0.0, 0.0);
        approx(d.ta, 0.0);
        approx(d.tb, 0.0);
        approx(d.tc, 0.0);
    }

    /// A pure α-voltage at the linear limit Vdc/√3 should stay in [0, 1].
    #[test]
    fn max_linear_range_stays_in_bounds() {
        let vdc = 24.0;
        let vmax = vdc / 1.732_050_8; // Vdc/√3
        let d = svpwm(vmax, 0.0, vdc);
        assert!(0.0 <= d.ta && d.ta <= 1.0);
        assert!(0.0 <= d.tb && d.tb <= 1.0);
        assert!(0.0 <= d.tc && d.tc <= 1.0);
        assert!(d.ta > d.tb); // A should be highest, B=C lowest
        approx(d.tb, d.tc);
    }

    /// Line-to-line voltage matches the original phase difference.
    #[test]
    fn lll_voltage_matches_phase_diff() {
        let vdc = 24.0;
        let d = svpwm(5.0, 0.0, vdc);
        // With Vβ=0: Va = Vα, Vb = −Vα/2, so Vab = 1.5·Vα = 7.5 V
        let vab = (d.ta - d.tb) * vdc;
        approx(vab, 7.5);
    }

    /// At the middle of sector 1 (30°), phases are monotonically decreasing.
    #[test]
    fn sector_1_order() {
        let vdc = 24.0;
        let vmag = 10.0;
        let angle = core::f32::consts::FRAC_PI_6; // 30°
        let v_alpha = vmag * libm::cosf(angle);
        let v_beta = vmag * libm::sinf(angle);
        let d = svpwm(v_alpha, v_beta, vdc);
        let cycles = [d.ta, d.tb, d.tc];
        for &c in &cycles { assert!(0.0 <= c && c <= 1.0, "duty {c} out of [0,1]"); }
        // Sector 1 (0–60°): A > B > C
        assert!(d.ta > d.tb);
        assert!(d.tb > d.tc);
    }

    /// Over-modulation (|v| > Vdc/√3) is clamped to [0, 1].
    #[test]
    fn over_modulation_clamps() {
        let vdc = 24.0;
        let v_over = vdc * 0.75; // exceeds Vdc/√3 ≈ 13.86
        let d = svpwm(v_over, 0.0, vdc);
        approx(d.ta, 1.0); // clamped at max
        approx(d.tb, 0.0); // clamped at min
        approx(d.tc, 0.0);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
