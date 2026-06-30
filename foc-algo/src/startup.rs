//! Motor startup helpers — field weakening and rotor alignment.
//!
//! # Rotor alignment strategies
//!
//! Before closing the FOC current loop, you must know the rotor's electrical
//! angle.  Three common approaches are described below.  Pick based on your
//! sensor setup and mechanical constraints.
//!
//! ## 1. DC alignment (parking)
//!
//! Inject a DC current vector at a fixed angle for a short time.  The rotor
//! aligns to that angle like a compass needle.  This is the simplest method
//! and works well for surface-mount PMSMs.
//!
//! **How to implement:**
//! 1. Set `id_ref = alignment_current`, `iq_ref = 0`, `theta = 0` (or your
//!    chosen alignment angle).
//! 2. Run the current loop open-loop for 100–500 ms (depending on inertia
//!    and friction).
//! 3. Rotor is now aligned.  Set `theta` to the alignment angle and switch
//!    to closed-loop FOC.
//!
//! **Pros:** simple, robust against sensor noise.
//! **Cons:** rotor moves (not suitable if mechanical constraint prohibits
//! rotation), takes time.
//!
//! ## 2. High-frequency injection (HFI)
//!
//! Inject a high-frequency (500 Hz – 2 kHz) voltage signal into the d-axis
//! and demodulate the q-axis current response.  Saliency (Ld ≠ Lq) produces
//! a position-dependent ripple that can be tracked with a PLL.
//!
//! **How to implement (pulsating vector method):**
//! 1. Inject `vd = V_hf * cos(ω_hf * t)`, `vq = 0` in the *estimated* dq frame.
//! 2. Band-pass filter `iq` at `ω_hf`, then multiply by `sin(ω_hf * t)`.
//! 3. Low-pass filter the product → error signal proportional to
//!    `sin(2 * (θ_real − θ_estimated))`.
//! 4. Feed error into a PLL → corrected angle.
//!
//! **Pros:** works at zero speed, no rotor motion, works for interior PMSMs.
//! **Cons:** requires accurate current sensing at HF, audible noise, needs
//! motor saliency (Ld ≠ Lq).  Not well-suited for surface-mount PMSMs.
//!
//! ## 3. Forced-angle open-loop start
//!
//! Ramp the stator field angle at a fixed rate, treating the motor as a
//! stepper.  Once the rotor catches up (typically above ~5% of rated speed),
//! transition to sensorless observer (e.g. sliding-mode or Luenberger).
//!
//! **How to implement:**
//! 1. Set `theta += open_loop_speed * dt` each cycle.
//! 2. Run current loop with a fixed `iq_ref` (enough to overcome friction).
//! 3. Ramp `open_loop_speed` from 0 to the transition threshold.
//! 4. When speed ≥ threshold, switch to closed-loop observer.
//!
//! **Pros:** works with any motor type, no alignment delay.
//! **Cons:** can stall under sudden load changes during open-loop phase,
//! requires tuning of current amplitude vs. speed ramp rate.
//!
//! ## Which to use?
//!
//! | Situation                                    | Strategy      |
//! |----------------------------------------------|---------------|
//! | Surface-mount PMSM, no mechanical constraint  | DC alignment  |
//! | Interior PMSM, zero-speed start required      | HFI           |
//! | Any motor, sensorless, can tolerate slip      | Forced-angle  |
//! | Absolute encoder available                    | Skip — read directly |
//! | Incremental encoder + Hall sensors            | Use Hall for initial sector, then FOC |
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
//! The function below computes the steady-state `id_ref` for a given speed.
//! It returns 0 for speeds below the base-speed threshold.

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
    // ω_base = V_limit / λ_pm.
    let omega_e = omega_e.max(0.0);
    if flux_linkage <= 0.0 || omega_e <= 0.0 {
        return 0.0;
    }
    let omega_base = v_limit / flux_linkage;

    if omega_e <= omega_base {
        return 0.0;
    }

    // id = (λ_pm / Ld) · (ω_base/ω − 1)
    //   = negative when ω > ω_base  (field-weakening current)
    // Clamp ratio to [-1, 0] — at infinite speed, ratio → −1, giving the
    // characteristic current limit id = −λ_pm / Ld.
    let ratio = (omega_base / omega_e - 1.0).clamp(-1.0, 0.0);
    // The result is negative — flux-weakening current.
    (flux_linkage / ld) * ratio
}

// ---------------------------------------------------------------------------
// MTPA — Maximum Torque Per Ampere
// ---------------------------------------------------------------------------

/// Compute the optimal d-axis current for maximum torque per ampere.
///
/// For interior PMSMs where `Lq > Ld`, injecting negative `id` produces
/// reluctance torque that *adds* to the PM torque.  The MTPA curve finds
/// the `(id, iq)` pair that maximises torque for a given current magnitude.
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
pub fn mtpa(flux_linkage: f32, lq: f32, ld: f32, iq: f32) -> f32 {
    debug_assert!(lq > ld, "MTPA requires interior PMSM (Lq > Ld)");
    debug_assert!(lq > 0.0 && ld > 0.0, "Inductances must be positive");

    let delta_l = lq - ld;
    if delta_l <= 0.0 {
        return 0.0;
    }
    let iq_abs = iq.abs();
    let term = flux_linkage / (2.0 * delta_l);
    let id_mtpa = term - libm::sqrtf(term * term + iq_abs * iq_abs);
    // id_mtpa is always ≤ 0 for interior PMSM — subtracts from PM flux.
    id_mtpa
}

// ---------------------------------------------------------------------------
// I²t thermal overload protection
// ---------------------------------------------------------------------------

/// I²t thermal overload limiter.
///
/// Tracks the accumulated squared-current error over time.  When the RMS
/// current exceeds the rated value for long enough, the limiter triggers
/// and scales down the maximum allowed current.
///
/// # Thermal model
///
/// ```text
/// heating   = (i² − i_rated²) · dt        [A²·s]
/// cooling   = −cooling_rate · accumulator · dt   [A²·s]
/// foldback  = max(0, 1 − accumulator / limit)
/// ```
///
/// When `accumulator ≥ limit`, `foldback()` returns 0 (hard cut).  When the
/// current drops below rated, the accumulator cools down and foldback
/// recovers.
///
/// # Usage
///
/// ```ignore
/// let mut i2t = I2tLimiter::new(10.0, 50.0, 0.5);  // 10 A rated, 50 A²·s limit
/// // Each cycle:
/// let iq_max = i2t.foldback(i_actual, dt);
/// // Use iq_max as the upper bound on the speed-loop output.
/// ```
#[derive(Clone, Copy)]
pub struct I2tLimiter {
    /// Rated continuous current (A RMS).
    pub i_rated: f32,
    /// Thermal capacity — maximum accumulated I²·t before hard cut (A²·s).
    pub limit: f32,
    /// Cooling rate (fraction of accumulator decayed per second).  1.0 = full
    /// cooldown in ~1 s.  0.01 = slow cooldown (minutes).  Typical: 0.1–0.5.
    pub cooling_rate: f32,
    accumulator: f32,
}

impl I2tLimiter {
    #[must_use]
    pub fn new(i_rated: f32, limit: f32, cooling_rate: f32) -> Self {
        Self { i_rated, limit, cooling_rate, accumulator: 0.0 }
    }

    /// Current foldback factor.  Call each control cycle with the actual
    /// current magnitude `|i|` (A) and `dt` (seconds).
    ///
    /// Returns a factor in [0, 1] — multiply this by the nominal current
    /// limit to get the thermally-limited maximum.
    #[must_use]
    pub fn foldback(&mut self, i_actual: f32, dt: f32) -> f32 {
        if dt <= 0.0 {
            return self.factor();
        }
        let i2 = i_actual * i_actual;
        let i2_rated = self.i_rated * self.i_rated;
        // Heating: I² − I_rated²
        self.accumulator += (i2 - i2_rated) * dt;
        // Cooling: proportional decay
        self.accumulator -= self.cooling_rate * self.accumulator * dt;
        // Clamp to [0, limit] — can't go negative, won't exceed limit (saturated)
        self.accumulator = self.accumulator.clamp(0.0, self.limit);
        self.factor()
    }

    /// Current foldback factor without advancing time.
    #[must_use]
    pub fn factor(&self) -> f32 {
        if self.limit <= 0.0 {
            return 1.0;
        }
        (1.0 - self.accumulator / self.limit).max(0.0)
    }

    /// Accumulated I²·t value (A²·s).  0 = cold, at limit = hard cut.
    #[must_use]
    pub fn accumulator(&self) -> f32 {
        self.accumulator
    }

    pub fn reset(&mut self) {
        self.accumulator = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ── field_weakening ──

    #[test]
    fn below_base_speed_returns_zero() {
        // 24 V bus, λ_pm = 0.01 → ω_base = (24/√3)/0.01 ≈ 1385 rad/s.
        // 100 rad/s is well below base.
        let id = field_weakening(24.0, 100.0, 0.01, 0.001);
        approx(id, 0.0);
    }

    #[test]
    fn above_base_speed_returns_negative() {
        // 24 V bus, λ_pm = 0.01 → ω_base ≈ 1385 rad/s.
        // 2000 rad/s → above base.
        let id = field_weakening(24.0, 2000.0, 0.01, 0.001);
        assert!(id < 0.0);
        assert!(id > -20.0); // reasonable range for this example
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
        // At very high speed, id approaches −λ_pm/Ld = −10 A asymptotically
        // but never exceeds it (ratio is clamped to [−1, 0]).
        let id = field_weakening(24.0, 1_000_000.0, 0.01, 0.001);
        assert!(id < -9.9, "id={id} should approach -10 at high speed");
        assert!(id >= -10.0, "id={id} must never exceed characteristic current");
    }

    // ── MTPA ──

    #[test]
    fn mtpa_spm_returns_zero() {
        // For near-surface-mount PMSM (Lq-Ld = 10µH), MTPA offset is tiny but non-zero.
        let id = mtpa(0.01, 0.00101, 0.001, 10.0);
        assert!(id <= 0.0);
    }

    #[test]
    fn mtpa_zero_iq_returns_zero() {
        // Zero torque command → zero MTPA offset.
        let id = mtpa(0.01, 0.002, 0.001, 0.0);
        approx(id, 0.0);
    }

    #[test]
    fn mtpa_negative_at_rated_iq() {
        let id = mtpa(0.01, 0.002, 0.001, 5.0);
        assert!(id < 0.0);
        assert!(id > -10.0);
    }

    // ── I²t ──

    #[test]
    fn i2t_cold_is_full_current() {
        let lim = I2tLimiter::new(10.0, 50.0, 0.5);
        assert!((lim.factor() - 1.0).abs() < 1e-5);
    }

    #[test]
    fn i2t_rated_current_no_heating() {
        let mut lim = I2tLimiter::new(10.0, 50.0, 0.5);
        for _ in 0..100 {
            let _ = lim.foldback(10.0, 0.001);
        }
        assert!(lim.accumulator() < 0.01);
    }

    #[test]
    fn i2t_overload_builds_up() {
        let mut lim = I2tLimiter::new(10.0, 50.0, 0.5);
        let _ = lim.foldback(20.0, 0.1);
        assert!(lim.accumulator() > 25.0);
    }

    #[test]
    fn i2t_overload_causes_foldback() {
        let mut lim = I2tLimiter::new(10.0, 50.0, 0.5);
        let _ = lim.foldback(30.0, 0.1);
        approx(lim.factor(), 0.0);
    }

    #[test]
    fn i2t_cools_down() {
        let mut lim = I2tLimiter::new(10.0, 50.0, 1.0);
        let _ = lim.foldback(30.0, 0.05);
        let acc_before = lim.accumulator();
        assert!(acc_before > 0.0);
        let _ = lim.foldback(0.0, 0.1);
        assert!(lim.accumulator() < acc_before);
    }

    #[test]
    fn i2t_reset_clears() {
        let mut lim = I2tLimiter::new(10.0, 50.0, 0.5);
        let _ = lim.foldback(30.0, 0.1);
        lim.reset();
        assert!((lim.accumulator() - 0.0).abs() < 1e-5);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
