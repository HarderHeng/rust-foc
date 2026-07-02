//! DQ-axis feedforward voltage decoupling for PMSM current control.
//!
//! In the rotor-aligned `dq` frame, the stator voltage equations include
//! cross-coupling terms from the synchronous inductance and a back-EMF from
//! the rotor magnet:
//!
//! ```text
//! Vd = R·Id − ω·Lq·Iq + d(Ld·Id)/dt
//! Vq = R·Iq + ω·Ld·Id + ω·λ + d(Lq·Iq)/dt
//! ```
//!
//! The two PI controllers in the current loop see the *full* voltage
//! including the `−ω·Lq·Iq` and `+ω·Ld·Id` cross terms.  At speed those
//! become large and make the d- and q-axis loops interfere: tuning one PI
//! fights the other.
//!
//! ## Compensation
//!
//! Subtract the known cross-coupling terms from the PI output so each axis
//! only has to track its own residual error:
//!
//! ```text
//! Vd_ff = −ω · Lq · Iq
//! Vq_ff =  ω · Ld · Id + ω · λ
//! ```
//!
//! Add `V_ff` to the PI output **after** each PI loop runs, **before**
//! SVPWM.  This is the standard decoupling scheme in every textbook and
//! in ST's MCSDK `feed_forward_ctrl.c`.
//!
//! ## Caveats
//!
//! - **Slow / idle**: `ω ≈ 0` ⇒ `V_ff ≈ 0`.  No effect.  Safe to leave always-on
//!   in this regime.
//! - **Fast / detuned**: a wrong `Lq` or `Ld` estimate turns the decoupling
//!   from a correction into a disturbance.  ST's MCSDK gates the decoupling
//!   on `|speed| > speed_threshold` for this reason; we don't, because the
//!   caller can just leave it disabled.  Use [`MotorParams`](crate::MotorParams)
//!   values that match your motor.
//! - **Sign convention**: we use the standard "rotor-aligned dq" frame
//!   where positive `ω` produces a positive `Vq_ff`.  Verify against your
//!   hardware by sweeping `Iq` at constant speed and confirming `Vd ≈ 0`.

/// Voltage feedforward decoupling for PMSM current loop.
///
/// # Arguments
/// * `omega_e` — electrical angular speed (rad/s).  Mechanical speed
///   multiplied by pole pairs.
/// * `ld`, `lq` — d/q-axis inductance (H).
/// * `flux_linkage` — PM flux linkage (Wb).
/// * `id`, `iq` — commanded d/q currents (A).
///
/// # Returns
/// `(Vd_ff, Vq_ff)` — voltage feedforward terms to add to the PI outputs
/// (V).  Both terms are zero when `omega_e = 0`.
#[inline]
#[must_use]
pub fn decoupling_voltage(
    omega_e: f32,
    ld: f32, lq: f32,
    flux_linkage: f32,
    id: f32, iq: f32,
) -> (f32, f32) {
    // NaN / INF guard. The observer can briefly emit non-finite `omega_e`
    // during ramp-up or PLL divergence; feeding NaN into the current PI
    // integrators NaN-poisons every downstream node (SVPWM, PWM duty),
    // and the loop has no built-in recovery path. Returning (0, 0) keeps
    // the PI running across the transient and lets the observer re-converge.
    if !omega_e.is_finite()
        || !ld.is_finite() || !lq.is_finite()
        || !flux_linkage.is_finite()
        || !id.is_finite() || !iq.is_finite()
    {
        return (0.0, 0.0);
    }
    let vd_ff = -omega_e * lq * iq;
    let vq_ff = omega_e * (ld * id + flux_linkage);
    (vd_ff, vq_ff)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }

    #[test]
    fn zero_speed_gives_zero() {
        let (vd, vq) = decoupling_voltage(
            0.0,
            0.0005, 0.0005,
            0.01,
            1.0, 5.0,
        );
        approx(vd, 0.0);
        approx(vq, 0.0);
    }

    #[test]
    fn zero_current_gives_only_back_emf_in_q() {
        // Iq=0, Id=0 → Vd = 0, Vq = ω·λ
        let (vd, vq) = decoupling_voltage(
            100.0,
            0.0005, 0.0008,   // ld < lq (IPM)
            0.05,
            0.0, 0.0,
        );
        approx(vd, 0.0);
        approx(vq, 100.0 * 0.05);  // 5.0 V
    }

    #[test]
    fn cross_coupling_d_term() {
        // ω·Lq·Iq appears with negative sign in Vd.
        // 100 · 0.0008 · 5 = 0.4, so Vd_ff = -0.4
        let (vd, _vq) = decoupling_voltage(
            100.0,
            0.0005, 0.0008,
            0.0,  // flux = 0 to isolate cross-coupling
            0.0, 5.0,
        );
        approx(vd, -0.4);
    }

    #[test]
    fn cross_coupling_q_term() {
        // ω·Ld·Id contributes positively to Vq.
        // 100 · 0.0005 · (-3.0) = -0.15 (MTPA injects negative Id)
        let (_vd, vq) = decoupling_voltage(
            100.0,
            0.0005, 0.0008,
            0.0,
            -3.0, 5.0,
        );
        approx(vq, -0.15);
    }

    #[test]
    fn sign_inverts_with_speed() {
        let (vd_pos, _) = decoupling_voltage(100.0, 0.0005, 0.0008, 0.0, 0.0, 5.0);
        let (vd_neg, _) = decoupling_voltage(-100.0, 0.0005, 0.0008, 0.0, 0.0, 5.0);
        approx(vd_pos, -vd_neg);
    }

    /// NaN / INF in any input must produce (0, 0) so a transient observer
    /// failure cannot NaN-poison the downstream current-loop integrators.
    #[test]
    fn non_finite_input_returns_zero() {
        let cases: [(f32, f32, f32, f32, f32, f32); 6] = [
            (f32::NAN, 0.0005, 0.0008, 0.05, 1.0, 5.0),     // omega_e NaN
            (100.0, f32::NAN, 0.0008, 0.05, 1.0, 5.0),      // ld NaN
            (100.0, 0.0005, 0.0008, f32::INFINITY, 1.0, 5.0), // flux INF
            (100.0, 0.0005, 0.0008, 0.05, f32::NAN, 5.0),   // id NaN
            (100.0, 0.0005, 0.0008, 0.05, 1.0, f32::NEG_INFINITY), // iq -INF
            (f32::NAN, f32::NAN, f32::NAN, f32::NAN, f32::NAN, f32::NAN), // all NaN
        ];
        for (omega, ld, lq, flux, id, iq) in cases {
            let (vd, vq) = decoupling_voltage(omega, ld, lq, flux, id, iq);
            approx(vd, 0.0);
            approx(vq, 0.0);
        }
    }
}
