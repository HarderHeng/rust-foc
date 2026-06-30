//! Sensorless rotor angle observer — sliding-mode + PLL.
//!
//! Estimates rotor electrical angle θ̂ and speed ω̂ from phase voltages and
//! currents.  No encoder needed — works with any PMSM.
//!
//! # Theory
//!
//! **Current observer** (SMO):
//!
//! ```text
//! dîα/dt = (vα − Rs·îα + k·sign(îα − iα)) / Ls
//! dîβ/dt = (vβ − Rs·îβ + k·sign(îβ − iβ)) / Ls
//! ```
//!
//! The sign correction `k·sign(error)` forces the estimated currents to track
//! the measured ones.  The switching signal **contains** the back-EMF.
//!
//! **Back-EMF extraction:**
//!
//! ```text
//! êα = LPF(k·sign(îα − iα))
//! êβ = LPF(k·sign(îβ − iβ))
//! ```
//!
//! **Angle from back-EMF:**
//!
//! ```text
//! θ_raw = atan2(−êα, êβ)
//! ```
//!
//! **PLL for smooth angle/speed:**
//!
//! ```text
//! error = wrap(θ_raw − θ̂)
//! ω̂ += kp_pll·error + ki_pll·∫error·dt
//! θ̂ += ω̂·dt
//! ```
//!
//! # Usage
//!
//! ```ignore
//! let mut smo = SmoObserver::new(SmoConfig {
//!     rs: 0.5,          // stator resistance (Ω)
//!     ls: 0.0005,       // stator inductance (H)
//!     k_slide: 50.0,    // sliding-mode gain (> λ_pm·ω_max)
//!     emf_cutoff: 200.0,// back-EMF filter cutoff (Hz)
//!     pll_kp: 0.1,      // PLL proportional gain
//!     pll_ki: 10.0,     // PLL integral gain
//! });
//!
//! // Each control cycle:
//! smo.update(v_alpha, v_beta, i_alpha, i_beta, dt);
//! let theta = smo.theta_hat();
//! let omega = smo.omega_hat();
//! ```
//!
//! # Tuning
//!
//! | Parameter  | Guideline |
//! |-----------|-----------|
//! | `k_slide` | 1.5–2× the maximum back-EMF (`λ_pm·ω_max`). Too low → unstable. Too high → noisy back-EMF. |
//! | `emf_cutoff` | Start at 0.5–1× the PWM frequency. Lower → smoother but more phase lag. |
//! | `pll_kp` | Start at 0.01·ω_max. Higher → faster lock but more jitter. |
//! | `pll_ki` | Start at kp·100. Higher → faster convergence but possible overshoot. |

use crate::math::filter::LowPassFilter;
use libm::atan2f;

#[cfg(test)]
use crate::math::LibmTrig;

/// SMO configuration — set once at init.
#[derive(Clone, Copy, Debug)]
pub struct SmoConfig {
    /// Stator resistance (Ω).
    pub rs: f32,
    /// Stator inductance (H).  For IPM motors, use Ld (the smaller inductance).
    pub ls: f32,
    /// Sliding-mode gain.  Must exceed the maximum possible back-EMF amplitude
    /// (`λ_pm · ω_max`).  Typical: 1.5–2× `λ_pm·ω_max`.
    pub k_slide: f32,
    /// Back-EMF low-pass filter cutoff (Hz).
    pub emf_cutoff: f32,
    /// PLL proportional gain.
    pub pll_kp: f32,
    /// PLL integral gain.
    pub pll_ki: f32,
}

/// Per-cycle diagnostic values.
#[derive(Default, Clone, Copy)]
pub struct SmoRuntime {
    pub i_alpha_hat: f32,
    pub i_beta_hat: f32,
    pub e_alpha: f32,
    pub e_beta: f32,
    pub theta_raw: f32,
    pub pll_error: f32,
}

/// Sliding-mode observer + PLL for sensorless FOC.
pub struct SmoObserver {
    cfg: SmoConfig,
    i_alpha_hat: f32,
    i_beta_hat: f32,
    lpf_e_alpha: LowPassFilter,
    lpf_e_beta: LowPassFilter,
    theta_hat: f32,
    omega_hat: f32,
    pll_integral: f32,

    /// Per-cycle diagnostics.
    pub runtime: SmoRuntime,
}

impl SmoObserver {
    #[must_use]
    pub fn new(cfg: SmoConfig) -> Self {
        Self {
            cfg,
            i_alpha_hat: 0.0,
            i_beta_hat: 0.0,
            lpf_e_alpha: LowPassFilter::new(cfg.emf_cutoff),
            lpf_e_beta: LowPassFilter::new(cfg.emf_cutoff),
            theta_hat: 0.0,
            omega_hat: 0.0,
            pll_integral: 0.0,
            runtime: SmoRuntime::default(),
        }
    }

    /// Estimated rotor electrical angle (radians, [0, 2π)).
    #[must_use]
    pub fn theta_hat(&self) -> f32 {
        self.theta_hat
    }

    /// Estimated rotor electrical speed (rad/s).
    #[must_use]
    pub fn omega_hat(&self) -> f32 {
        self.omega_hat
    }

    /// Seed the PLL angle (e.g. from DC alignment or Hall sensors).
    pub fn set_angle(&mut self, theta: f32) {
        self.theta_hat = theta;
    }

    /// Seed the PLL speed (e.g. for open-loop → closed-loop transition).
    pub fn set_speed(&mut self, omega: f32) {
        self.omega_hat = omega;
    }

    /// Reset all internal state.  Configuration is kept.
    pub fn reset(&mut self) {
        self.i_alpha_hat = 0.0;
        self.i_beta_hat = 0.0;
        self.lpf_e_alpha.reset();
        self.lpf_e_beta.reset();
        self.theta_hat = 0.0;
        self.omega_hat = 0.0;
        self.pll_integral = 0.0;
        self.runtime = SmoRuntime::default();
    }

    /// One observer step with dq-axis inputs.
    ///
    /// `v_d, v_q` are the current-loop PI outputs (voltage references in
    /// the rotor frame).  `theta` is the *previous* estimated angle used to
    /// transform them to αβ.  `i_d, i_q` are the measured currents in the
    /// rotor frame — typically obtained from `CurrentLoop::runtime`.
    ///
    /// When `dt ≤ 0`, the call is a no-op.
    pub fn update_dq<T: crate::math::Trig>(
        &mut self,
        v_d: f32, v_q: f32, theta: f32,
        i_d: f32, i_q: f32,
        dt: f32,
    ) {
        if dt <= 0.0 {
            return;
        }
        let v_ab = crate::math::inv_park::<T>(
            crate::math::Dq { d: v_d, q: v_q }, theta,
        );
        let i_ab = crate::math::inv_park::<T>(
            crate::math::Dq { d: i_d, q: i_q }, theta,
        );
        self.update(v_ab.alpha, v_ab.beta, i_ab.alpha, i_ab.beta, dt);
    }

    /// One observer step with αβ-axis inputs.
    ///
    /// `v_alpha, v_beta` are the commanded voltages (from the current-loop PI
    /// outputs).  `i_alpha, i_beta` are the measured (Clarke-transformed)
    /// currents.  `dt` is the control period in seconds.
    ///
    /// When `dt ≤ 0`, the call is a no-op.
    pub fn update(
        &mut self,
        v_alpha: f32,  v_beta: f32,
        i_alpha: f32,  i_beta: f32,
        dt: f32,
    ) {
        if dt <= 0.0 {
            return;
        }

        // Current observer error
        let err_alpha = self.i_alpha_hat - i_alpha;
        let err_beta  = self.i_beta_hat  - i_beta;
        let z_alpha = self.cfg.k_slide * sign(err_alpha);
        let z_beta  = self.cfg.k_slide * sign(err_beta);

        // Current observer dynamics (Euler integration):
        // dî/dt = (v − Rs·î + z) / Ls
        let inv_ls = 1.0 / self.cfg.ls;
        self.i_alpha_hat += (v_alpha - self.cfg.rs * self.i_alpha_hat + z_alpha) * inv_ls * dt;
        self.i_beta_hat  += (v_beta  - self.cfg.rs * self.i_beta_hat  + z_beta)  * inv_ls * dt;
        self.runtime.i_alpha_hat = self.i_alpha_hat;
        self.runtime.i_beta_hat  = self.i_beta_hat;

        // Back-EMF: low-pass the switching signal.
        let e_alpha = self.lpf_e_alpha.update(z_alpha, dt);
        let e_beta  = self.lpf_e_beta.update(z_beta, dt);
        self.runtime.e_alpha = e_alpha;
        self.runtime.e_beta  = e_beta;

        // Raw angle: θ = atan2(−e_α, e_β).  Normalise to [0, 2π).
        let mut theta_raw = atan2f(-e_alpha, e_beta);
        if theta_raw < 0.0 {
            theta_raw += core::f32::consts::TAU;
        }
        self.runtime.theta_raw = theta_raw;

        // PLL — wrap error to [−π, π], then integrate.
        let mut pll_error = theta_raw - self.theta_hat;
        if pll_error > core::f32::consts::PI {
            pll_error -= core::f32::consts::TAU;
        } else if pll_error < -core::f32::consts::PI {
            pll_error += core::f32::consts::TAU;
        }
        self.runtime.pll_error = pll_error;

        self.pll_integral += self.cfg.pll_ki * pll_error * dt;
        self.omega_hat = self.cfg.pll_kp * pll_error + self.pll_integral;
        self.theta_hat += self.omega_hat * dt;

        // Normalise θ̂ to [0, 2π).
        if self.theta_hat >= core::f32::consts::TAU {
            self.theta_hat -= core::f32::consts::TAU;
        } else if self.theta_hat < 0.0 {
            self.theta_hat += core::f32::consts::TAU;
        }
    }
}

/// Sign function — returns ±1.0.
#[inline]
fn sign(x: f32) -> f32 {
    if x > 0.0 { 1.0 } else if x < 0.0 { -1.0 } else { 0.0 }
}

// ── PLL tuning ─────────────────────────────────────────────────────────────

/// Type-2 PLL PI gains from desired bandwidth.
///
/// For the sensorless observer's angle-tracking PLL.  A higher bandwidth
/// locks faster but is more sensitive to noise.
///
/// ```text
/// ω_n   = 2π · bandwidth_hz
/// Kp    = √2 · ω_n        (critical damping ζ ≈ 0.707)
/// Ki    = ω_n²
/// ```
///
/// Typical values: 10–50 Hz for most PMSMs.
#[must_use]
pub fn pll_pi_gains(bandwidth_hz: f32) -> (f32, f32) {
    let wn = 2.0 * core::f32::consts::PI * bandwidth_hz;
    (1.414_213_56 * wn, wn * wn)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_cfg() -> SmoConfig {
        SmoConfig {
            rs: 0.5,
            ls: 0.0005,
            k_slide: 50.0,
            emf_cutoff: 200.0,
            pll_kp: 0.1,
            pll_ki: 10.0,
        }
    }

    #[test]
    fn starts_at_zero() {
        let smo = SmoObserver::new(default_cfg());
        assert!((smo.theta_hat() - 0.0).abs() < 1e-5);
        assert!((smo.omega_hat() - 0.0).abs() < 1e-5);
    }

    #[test]
    fn set_angle_works() {
        let mut smo = SmoObserver::new(default_cfg());
        smo.set_angle(1.5);
        assert!((smo.theta_hat() - 1.5).abs() < 1e-5);
    }

    #[test]
    fn set_speed_works() {
        let mut smo = SmoObserver::new(default_cfg());
        smo.set_speed(100.0);
        assert!((smo.omega_hat() - 100.0).abs() < 1e-5);
    }

    #[test]
    fn update_converges_at_zero_speed() {
        // At standstill, v=0, i=0 — observer should stay at zero.
        let mut smo = SmoObserver::new(default_cfg());
        for _ in 0..100 {
            smo.update(0.0, 0.0, 0.0, 0.0, 0.0001);
        }
        assert!(smo.i_alpha_hat.abs() < 0.01);
        assert!(smo.i_beta_hat.abs() < 0.01);
    }

    #[test]
    fn reset_clears_state() {
        let mut smo = SmoObserver::new(default_cfg());
        smo.set_angle(1.0);
        smo.set_speed(50.0);
        smo.reset();
        assert!((smo.theta_hat() - 0.0).abs() < 1e-5);
        assert!((smo.omega_hat() - 0.0).abs() < 1e-5);
    }

    #[test]
    fn dt_zero_is_noop() {
        let mut smo = SmoObserver::new(default_cfg());
        smo.set_angle(1.0);
        smo.set_speed(50.0);
        smo.update(1.0, 0.0, 0.1, 0.0, 0.0);
        assert!((smo.theta_hat() - 1.0).abs() < 1e-5);
        assert!((smo.omega_hat() - 50.0).abs() < 1e-5);
    }

    #[test]
    fn sign_function() {
        assert!((sign(5.0) - 1.0).abs() < 1e-5);
        assert!((sign(-3.0) + 1.0).abs() < 1e-5);
        assert!((sign(0.0) - 0.0).abs() < 1e-5);
    }

    // ── update_dq ──

    #[test]
    fn update_dq_routes_to_alpha_beta() {
        // Same dq inputs transformed by inv_park should match update()'s
        // αβ path.  Both should produce the same observer state.
        let cfg = default_cfg();

        // Pick a non-zero theta so the inv_park transform is non-trivial.
        let theta = 1.7_f32;
        let v_d = 0.0; let v_q = 5.0;
        let i_d = 0.0; let i_q = 0.5;

        // Path 1: compute αβ externally, call update()
        let v_ab = crate::math::inv_park::<LibmTrig>(crate::math::Dq { d: v_d, q: v_q }, theta);
        let i_ab = crate::math::inv_park::<LibmTrig>(crate::math::Dq { d: i_d, q: i_q }, theta);

        let mut smo_a = SmoObserver::new(cfg);
        let mut smo_b = SmoObserver::new(cfg);
        for _ in 0..50 {
            smo_a.update(v_ab.alpha, v_ab.beta, i_ab.alpha, i_ab.beta, 0.0001);
            smo_b.update_dq::<LibmTrig>(v_d, v_q, theta, i_d, i_q, 0.0001);
        }

        // State should be identical within float rounding.
        let eps = 1e-4;
        assert!((smo_a.theta_hat() - smo_b.theta_hat()).abs() < eps);
        assert!((smo_a.omega_hat() - smo_b.omega_hat()).abs() < eps);
    }

    #[test]
    fn update_dq_dt_zero_noop() {
        let mut smo = SmoObserver::new(default_cfg());
        smo.set_angle(1.0);
        smo.update_dq::<LibmTrig>(1.0, 2.0, 0.5, 0.1, 0.2, 0.0);
        assert!((smo.theta_hat() - 1.0).abs() < 1e-5);
    }

    // ── PLL tuning ──

    #[test]
    fn pll_pi_gains_10hz() {
        let (kp, ki) = pll_pi_gains(10.0);
        assert!(kp > 80.0 && kp < 100.0);
        assert!(ki > 3500.0 && ki < 4500.0);
    }
}
