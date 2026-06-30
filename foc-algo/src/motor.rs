//! Motor parameters and PI/PLL auto-tuning.
//!
//! Instead of hand-calculating PI gains from first principles every time,
//! pack your motor's physical constants into [`MotorParams`] and let the
//! library derive them from your desired control bandwidth.
//!
//! # Quick start
//!
//! ```ignore
//! use foc_algo::MotorParams;
//!
//! let motor = MotorParams {
//!     r: 0.3,             // 0.3 Ω phase resistance
//!     ld: 0.0005,         // 0.5 mH d-axis inductance
//!     lq: 0.0005,         // 0.5 mH q-axis inductance (SPMSM)
//!     flux_linkage: 0.01, // 0.01 Wb
//!     pole_pairs: 7,
//!     continuous_current: 5.0,
//!     inertia: 0.000_01,  // 1e-5 kg·m²
//! };
//!
//! // Current loop: 1000 Hz bandwidth → PI gains
//! let (kp, ki) = motor.current_pi_gains(1000.0);
//!
//! // Speed loop: 50 Hz bandwidth → PI gains
//! let (kp, ki) = motor.speed_pi_gains(50.0, 1000.0);
//!
//! // Torque reference (SPMSM):
//! let iq = motor.torque_to_iq(0.5);  // 0.5 N·m
//! ```

use core::f32::consts::PI;

/// Packed motor physical constants.
///
/// All values in SI units unless noted otherwise.  Fill in once, then use
/// the methods to derive PI gains, torque references, and observer gains.
#[derive(Clone, Copy, Debug)]
pub struct MotorParams {
    /// Phase resistance (Ω).  Must be > 0.
    pub r: f32,
    /// d-axis inductance (H).  Must be > 0.
    pub ld: f32,
    /// q-axis inductance (H).  Must be > 0.
    pub lq: f32,
    /// Permanent-magnet flux linkage λ_pm (Wb = V·s/rad).
    /// Typical values: 0.001–0.1 for small PMSMs.
    pub flux_linkage: f32,
    /// Number of rotor pole **pairs** (not poles).  7 → 14-pole rotor.
    pub pole_pairs: u8,
    /// Maximum **continuous** current the motor windings can sustain
    /// indefinitely without overheating (A).  This is **informational
    /// only** — the FOC algorithm does not read or enforce it.  Use it in
    /// your application code:
    ///
    /// ```ignore
    /// ctrl.set_current_limit(motor.continuous_current);
    /// ```
    ///
    /// Short-term peaks above this value are fine if thermal limits (see
    /// [`I2tLimiter`](crate::I2tLimiter)) say so; the two work in tandem.
    pub continuous_current: f32,
    /// Rotor + load inertia (kg·m²).  Used for speed-loop tuning and
    /// inertia-compensating feedforward.
    pub inertia: f32,
}

impl MotorParams {
    // ── PI gain derivation ─────────────────────────────────────────────

    /// Current-loop PI gains via pole-zero cancellation.
    ///
    /// Models the stator as an RL load `1/(R + sL)`.  Placing the PI zero
    /// at the electrical pole (`Ki/Kp = R/L`) cancels it, giving a
    /// first-order closed-loop response with the specified bandwidth.
    ///
    /// ```text
    /// ω_c  = 2π · bandwidth_hz
    /// Kp   = ω_c · L         (V/A)
    /// Ki   = ω_c · R         (V/(A·s))
    /// ```
    ///
    /// Returns `(kp, ki)` suitable for both d- and q-axis current PIs.
    ///
    /// Uses the **average** inductance `(Ld + Lq) / 2` when the two axes
    /// differ (IPM motors).  For axes with very different dynamics, tune
    /// each axis separately with [`current_d_pi_gains`](Self::current_d_pi_gains)
    /// and [`current_q_pi_gains`](Self::current_q_pi_gains).
    #[must_use]
    pub fn current_pi_gains(&self, bandwidth_hz: f32) -> (f32, f32) {
        let l_avg = 0.5 * (self.ld + self.lq);
        debug_assert!(bandwidth_hz > 0.0, "bandwidth must be positive");
        debug_assert!(self.r > 0.0, "R must be positive");
        debug_assert!(l_avg > 0.0, "L must be positive");
        let wc = 2.0 * PI * bandwidth_hz;
        (wc * l_avg, wc * self.r)
    }

    /// d-axis current PI gains (uses `Ld`).
    #[must_use]
    pub fn current_d_pi_gains(&self, bandwidth_hz: f32) -> (f32, f32) {
        debug_assert!(bandwidth_hz > 0.0 && self.r > 0.0 && self.ld > 0.0);
        let wc = 2.0 * PI * bandwidth_hz;
        (wc * self.ld, wc * self.r)
    }

    /// q-axis current PI gains (uses `Lq`).
    #[must_use]
    pub fn current_q_pi_gains(&self, bandwidth_hz: f32) -> (f32, f32) {
        debug_assert!(bandwidth_hz > 0.0 && self.r > 0.0 && self.lq > 0.0);
        let wc = 2.0 * PI * bandwidth_hz;
        (wc * self.lq, wc * self.r)
    }

    /// Speed-loop PI gains via symmetrical optimum.
    ///
    /// Treats the closed current loop as a first-order lag with time
    /// constant `τ_i ≈ 1 / ω_ci` and the mechanical plant as `1/(J·s)`.
    ///
    /// ```text
    /// ω_cs  = 2π · bandwidth_hz
    /// Kp    = J · ω_cs
    /// Ki    = Kp · ω_cs / 4      (symmetrical-optimum tuning factor)
    /// ```
    ///
    /// `current_bandwidth_hz` should be the bandwidth you passed to
    /// [`current_pi_gains`](Self::current_pi_gains).  A typical ratio is
    /// `speed_bw : current_bw ≈ 1 : 10` to 1 : 20.
    ///
    /// # Panics
    ///
    /// Panics in debug if bandwidths ≤ 0, inertia ≤ 0, or
    /// `current_bandwidth_hz < bandwidth_hz` (inner loop must be faster).
    #[must_use]
    pub fn speed_pi_gains(
        &self,
        bandwidth_hz: f32,
        current_bandwidth_hz: f32,
    ) -> (f32, f32) {
        debug_assert!(bandwidth_hz > 0.0, "speed bandwidth must be positive");
        debug_assert!(current_bandwidth_hz > bandwidth_hz,
            "current loop bandwidth ({current_bandwidth_hz} Hz) must exceed speed loop ({bandwidth_hz} Hz)");
        debug_assert!(self.inertia > 0.0, "inertia must be positive");

        let w_cs = 2.0 * PI * bandwidth_hz;
        let kp = self.inertia * w_cs;
        let ki = kp * w_cs / 4.0;
        (kp, ki)
    }

    // ── Reference conversion ──────────────────────────────────────────────

    /// Torque (N·m) → q-axis current (A) for a surface-mount PMSM.
    ///
    /// ```text
    /// T = 1.5 · pp · λ_pm · iq
    /// iq = T / (1.5 · pp · λ_pm)
    /// ```
    ///
    /// For interior PMSMs, use [`torque_to_iq_ipm`](Self::torque_to_iq_ipm)
    /// together with the MTPA-optimal `id` from [`mtpa`](crate::mtpa).
    #[must_use]
    pub fn torque_to_iq(&self, torque: f32) -> f32 {
        debug_assert!(self.flux_linkage > 0.0, "flux_linkage must be positive");
        debug_assert!(self.pole_pairs > 0, "pole_pairs must be > 0");
        torque / (1.5 * f32::from(self.pole_pairs) * self.flux_linkage)
    }

    /// Torque → Iq for interior PM motors, given a known Id.
    ///
    /// Uses the full torque equation including reluctance torque:
    ///
    /// ```text
    /// T = 1.5 · pp · (λ_pm · iq + (Ld − Lq) · id · iq)
    /// iq = T / (1.5 · pp · (λ_pm + (Ld − Lq) · id))
    /// ```
    #[must_use]
    pub fn torque_to_iq_ipm(&self, torque: f32, id: f32) -> f32 {
        debug_assert!(self.pole_pairs > 0, "pole_pairs must be > 0");
        let denom = self.flux_linkage + (self.ld - self.lq) * id;
        debug_assert!(denom.abs() > 1e-12, "denominator too small — flux may be zero");
        torque / (1.5 * f32::from(self.pole_pairs) * denom)
    }

    // ── Observer gain derivation ─────────────────────────────────────────

    /// Sliding-mode observer gain from max expected electrical speed.
    ///
    /// ```text
    /// k_slide = safety_factor · λ_pm · ω_max
    /// ```
    ///
    /// The gain must exceed the maximum possible back-EMF amplitude to
    /// guarantee sliding-mode convergence.  `safety_factor` is typically
    /// 1.5–2.0.  `max_speed_rads` is the maximum expected **electrical**
    /// speed (mechanical speed × pole_pairs).
    #[must_use]
    pub fn smo_slide_gain(&self, max_speed_rads: f32, safety_factor: f32) -> f32 {
        safety_factor * self.flux_linkage * max_speed_rads
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn example_spm() -> MotorParams {
        MotorParams {
            r: 0.3, ld: 0.0005, lq: 0.0005,
            flux_linkage: 0.01, pole_pairs: 7,
            continuous_current: 5.0, inertia: 1e-5,
        }
    }

    fn example_ipm() -> MotorParams {
        MotorParams {
            r: 0.2, ld: 0.0003, lq: 0.0008,
            flux_linkage: 0.05, pole_pairs: 4,
            continuous_current: 10.0, inertia: 5e-4,
        }
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-4, "expected {b}, got {a}");
    }

    // ── PI gains ──

    #[test]
    fn current_pi_spm() {
        let m = example_spm();
        let (kp, ki) = m.current_pi_gains(1000.0);
        // ω_c = 2π·1000 ≈ 6283, L_avg = 0.0005, R = 0.3
        assert!((kp - 3.14).abs() < 0.05);
        assert!((ki - 1884.0).abs() < 2.0);
    }

    #[test]
    fn current_pi_ipm_splits_axes() {
        let m = example_ipm();
        let (kp_d, ki_d) = m.current_d_pi_gains(500.0);
        let (kp_q, ki_q) = m.current_q_pi_gains(500.0);
        assert!(kp_d < kp_q);
        approx(ki_d, ki_q);
    }

    #[test]
    fn speed_pi_symmetrical_optimum() {
        let m = example_spm();
        let (kp, ki) = m.speed_pi_gains(50.0, 1000.0);
        assert!(kp > 0.0 && kp < 0.01);
        assert!(ki > 0.0 && ki < 0.5);
    }

    #[test]
    #[should_panic(expected = "current loop bandwidth")]
    fn speed_pi_rejects_slower_inner_loop() {
        let m = example_spm();
        let _ = m.speed_pi_gains(100.0, 50.0);
    }

    // ── Torque → Iq ──

    #[test]
    fn torque_to_iq_spm() {
        let m = example_spm();
        let iq = m.torque_to_iq(1.05);
        approx(iq, 10.0);
    }

    #[test]
    fn torque_zero_gives_zero_iq() {
        approx(example_spm().torque_to_iq(0.0), 0.0);
    }

    #[test]
    fn torque_to_iq_ipm_with_id() {
        let m = example_ipm();
        let iq = m.torque_to_iq_ipm(6.0, 0.0);
        approx(iq, 20.0);
    }

    // ── SMO gain ──

    #[test]
    fn smo_slide_gain_formula() {
        let m = example_spm();
        // λ=0.01, ω_max=1000, safety=1.5 → 1.5 × 0.01 × 1000 = 15
        let gain = m.smo_slide_gain(1000.0, 1.5);
        approx(gain, 15.0);
    }
}
