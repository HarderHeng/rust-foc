//! Field weakening and MTPA — Id-reference optimisation above base speed.
//!
//! # Field weakening
//!
//! Above base speed, the back-EMF approaches the bus voltage limit.  To go
//! faster, inject negative d-axis current to oppose the rotor flux, reducing
//! the effective back-EMF.
//!
//! ```text
//! V_limit = Vdc / √3           (linear modulation limit)
//! ω_base  = V_limit / λ_pm     (base electrical speed, rad/s)
//!
//! For ω > ω_base:
//!   id_ref = (λ_pm / Ld) · (1 − ω_base / ω)
//! ```
//!
//! # MTPA — Maximum Torque Per Ampere
//!
//! For interior PMSMs where `Lq > Ld`, injecting negative `id` produces
//! reluctance torque that *adds* to the PM torque.  The MTPA curve finds
//! the `(id, iq)` pair that maximises torque for a given current magnitude.
//!
//! For surface-mount PMSMs (`Lq ≈ Ld`): returns 0 — no benefit.
//!
//! ```text
//! id = (λ_pm / (2·(Lq−Ld))) − √( (λ_pm/(2·(Lq−Ld)))² + iq² )
//! ```

/// Constant √3 — used for voltage-limit calculation.
const SQRT_3: f32 = 1.732_050_8;

/// Voltage-based field-weakening Id reference.
///
/// Returns a negative `id_ref` (in amperes) for speeds above the base-speed
/// threshold.  Below threshold, returns 0 — no field weakening needed.
///
/// # Arguments
/// * `vdc` — bus voltage (V).
/// * `omega_e` — current electrical speed (rad/s).  Use `0.0` or a small
///   positive value at standstill; negative speeds have no physical meaning
///   here and are clamped.
/// * `flux_linkage` — permanent-magnet flux linkage `λ_pm` (Wb or V·s/rad).
///   Typical values: 0.001–0.1 for small PMSMs.
/// * `ld` — d-axis inductance (H).  Must be > 0.
///
/// # Returns
/// `id_ref` ≤ 0 (A).  Negative — reduces the net d-axis flux.
///
/// # Panics
/// Panics in debug if `ld ≤ 0`.
///
/// # Example
///
/// ```ignore
/// // 24 V bus, 3000 rpm (314 rad/s), λ_pm = 0.005 Wb, Ld = 0.2 mH
/// let id_fw = field_weakening(24.0, 314.0, 0.005, 0.0002);
/// ```
#[must_use]
pub fn field_weakening(vdc: f32, omega_e: f32, flux_linkage: f32, ld: f32) -> f32 {
    debug_assert!(ld > 0.0, "Ld must be positive");
    debug_assert!(vdc > 0.0, "Vdc must be positive");

    // Linear modulation limit: Vdc / √3.
    let v_limit = vdc / SQRT_3;

    // Base speed: where back-EMF hits the voltage limit.
    let omega_e = omega_e.max(0.0);
    if flux_linkage <= 0.0 || omega_e <= 0.0 {
        return 0.0;
    }
    let omega_base = v_limit / flux_linkage;

    if omega_e <= omega_base {
        return 0.0;
    }

    // id = (λ_pm / Ld) · (ω_base/ω − 1)
    // Clamp ratio to [-1, 0] — at infinite speed, ratio → −1, giving the
    // characteristic current limit id = −λ_pm / Ld.
    let ratio = (omega_base / omega_e - 1.0).clamp(-1.0, 0.0);
    (flux_linkage / ld) * ratio
}

/// Compute the optimal d-axis current for maximum torque per ampere.
///
/// For interior PMSMs where `Lq > Ld`, injecting negative `id` produces
/// reluctance torque that *adds* to the PM torque.
///
/// For surface-mount PMSMs (`Lq ≈ Ld`): this returns 0 — no benefit.
///
/// # Formula
///
/// ```text
/// id = (λ_pm / (2·(Lq−Ld))) − √( (λ_pm/(2·(Lq−Ld)))² + iq² )
/// ```
///
/// # Arguments
/// * `flux_linkage` — PM flux linkage `λ_pm` (Wb).
/// * `lq` — q-axis inductance (H).  Must be > `ld`.
/// * `ld` — d-axis inductance (H).
/// * `iq` — q-axis current reference (A).  Use the absolute value; the sign
///   of the returned `id` is always negative (or zero for SPM).
///
/// # Returns
/// `id_mtpa ≤ 0` (A).  Negative — creates positive reluctance torque.
///
/// # Panics
/// Panics in debug if `lq ≤ ld` (requires interior PMSM for non-trivial result).
#[must_use]
#[cfg(feature = "libm-trig")]
pub fn mtpa(flux_linkage: f32, lq: f32, ld: f32, iq: f32) -> f32 {
    debug_assert!(lq > ld, "MTPA requires interior PMSM (Lq > Ld)");
    debug_assert!(lq > 0.0 && ld > 0.0, "Inductances must be positive");

    let delta_l = lq - ld;
    if delta_l <= 0.0 {
        return 0.0;
    }
    let iq_abs = iq.abs();
    let term = flux_linkage / (2.0 * delta_l);
    term - libm::sqrtf(term * term + iq_abs * iq_abs)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── field_weakening ──

    #[test]
    fn below_base_speed_returns_zero() {
        let id = field_weakening(24.0, 100.0, 0.01, 0.001);
        approx(id, 0.0);
    }

    #[test]
    fn above_base_speed_returns_negative() {
        let id = field_weakening(24.0, 2000.0, 0.01, 0.001);
        assert!(id < 0.0);
        assert!(id > -20.0);
    }

    #[test]
    fn standstill_returns_zero() {
        let id = field_weakening(24.0, 0.0, 0.01, 0.001);
        approx(id, 0.0);
    }

    #[test]
    fn negative_speed_clamped_to_zero() {
        let id = field_weakening(24.0, -100.0, 0.01, 0.001);
        approx(id, 0.0);
    }

    #[test]
    fn zero_flux_returns_zero() {
        let id = field_weakening(24.0, 2000.0, 0.0, 0.001);
        approx(id, 0.0);
    }

    #[test]
    fn characteristic_current_clamps() {
        let id = field_weakening(24.0, 1_000_000.0, 0.01, 0.001);
        assert!(id < -9.9, "id={id} should approach -10 at high speed");
        assert!(id >= -10.0, "id={id} must never exceed characteristic current");
    }

    // ── MTPA ──

    #[test]
    #[cfg(feature = "libm-trig")]
    fn mtpa_spm_returns_zero() {
        let id = mtpa(0.01, 0.00101, 0.001, 10.0);
        assert!(id <= 0.0);
    }

    #[test]
    #[cfg(feature = "libm-trig")]
    fn mtpa_zero_iq_returns_zero() {
        let id = mtpa(0.01, 0.002, 0.001, 0.0);
        approx(id, 0.0);
    }

    #[test]
    #[cfg(feature = "libm-trig")]
    fn mtpa_negative_at_rated_iq() {
        let id = mtpa(0.01, 0.002, 0.001, 5.0);
        assert!(id < 0.0);
        assert!(id > -10.0);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
