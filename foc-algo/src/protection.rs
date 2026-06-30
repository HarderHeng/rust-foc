//! Motor protection helpers — thermal foldback and rotor alignment.
//!
//! The historical name `startup` was misleading: this module is mostly about
//! protecting the motor from overcurrent / thermal damage, not just the
//! startup phase.  The rotor-alignment procedure in the module doc is the
//! only piece that's truly startup-related.
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

// ── I²t thermal overload protection ───────────────────────────────────────

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
        self.accumulator += (i2 - i2_rated) * dt;
        self.accumulator -= self.cooling_rate * self.accumulator * dt;
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn i2t_cold_is_full_current() {
        let lim = I2tLimiter::new(10.0, 50.0, 0.5);
        assert!((lim.factor() - 1.0).abs() < 1e-5);
    }

    #[test]
    fn i2t_continuous_current_no_heating() {
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
