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

use crate::filter::LowPassFilter;
use libm::atan2f;

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

    // Current observer state
    i_alpha_hat: f32,
    i_beta_hat: f32,

    // Back-EMF filters (LPF the switching signal)
    lpf_e_alpha: LowPassFilter,
    lpf_e_beta: LowPassFilter,

    // PLL state
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

    /// One observer step.
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

        // ── 1. Current observer error ──
        let err_alpha = self.i_alpha_hat - i_alpha;
        let err_beta  = self.i_beta_hat  - i_beta;

        // Switching function: sign(error) — extracts the back-EMF.
        let z_alpha = self.cfg.k_slide * sign(err_alpha);
        let z_beta  = self.cfg.k_slide * sign(err_beta);

        // ── 2. Current observer dynamics (Euler integration) ──
        let inv_ls = 1.0 / self.cfg.ls;
        // dî/dt = (v − Rs·î + z) / Ls
        self.i_alpha_hat += (v_alpha - self.cfg.rs * self.i_alpha_hat + z_alpha) * inv_ls * dt;
        self.i_beta_hat  += (v_beta  - self.cfg.rs * self.i_beta_hat  + z_beta)  * inv_ls * dt;

        self.runtime.i_alpha_hat = self.i_alpha_hat;
        self.runtime.i_beta_hat  = self.i_beta_hat;

        // ── 3. Back-EMF extraction (low-pass filter the switching signal) ──
        let e_alpha = self.lpf_e_alpha.update(z_alpha, dt);
        let e_beta  = self.lpf_e_beta.update(z_beta, dt);

        self.runtime.e_alpha = e_alpha;
        self.runtime.e_beta  = e_beta;

        // ── 4. Raw angle from back-EMF ──
        // For PMSM: e_α = −ω·λ_pm·sin(θ), e_β = ω·λ_pm·cos(θ)
        // Angle: θ = atan2(−e_α, e_β)
        let theta_raw = atan2f(-e_alpha, e_beta);
        // Normalise to [0, 2π)
        let theta_raw = if theta_raw < 0.0 { theta_raw + core::f32::consts::TAU } else { theta_raw };

        self.runtime.theta_raw = theta_raw;

        // ── 5. PLL — tracks the raw angle smoothly ──
        let mut pll_error = theta_raw - self.theta_hat;
        // Angle wrap to [−π, π]
        if pll_error > core::f32::consts::PI {
            pll_error -= core::f32::consts::TAU;
        } else if pll_error < -core::f32::consts::PI {
            pll_error += core::f32::consts::TAU;
        }

        self.runtime.pll_error = pll_error;

        self.pll_integral += self.cfg.pll_ki * pll_error * dt;
        self.omega_hat = self.cfg.pll_kp * pll_error + self.pll_integral;
        self.theta_hat += self.omega_hat * dt;

        // Normalise θ̂ to [0, 2π)
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
}
