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
use libm::sqrtf;

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

// ── Instant overcurrent ─────────────────────────────────────────────────────

/// Single-cycle overcurrent trip.  Fires when the αβ current vector
/// magnitude exceeds the configured threshold.  No debounce — the caller is
/// expected to call `check()` exactly once per control cycle and act on
/// the result before driving the PWM.  Use [`I2tLimiter`] for thermal
/// integration; this is purely an instantaneous hardware-protection trip.
#[derive(Clone, Copy, Debug)]
pub struct InstantOvercurrent {
    threshold_a: f32,
}

impl InstantOvercurrent {
    /// Trip threshold (A).  |i| > threshold trips.
    #[must_use]
    pub const fn new(threshold_a: f32) -> Self {
        Self { threshold_a }
    }

    /// True when |i| > threshold.
    #[must_use]
    pub fn check(&self, i_alpha: f32, i_beta: f32) -> bool {
        sqrtf(i_alpha * i_alpha + i_beta * i_beta) > self.threshold_a
    }
}

// ── Bus voltage monitor ─────────────────────────────────────────────────────

/// DC bus voltage window monitor.  Returns an error when vdc falls outside
/// `[vdc_min, vdc_max]`.  Useful as a per-cycle sanity check on the ADC
/// reading (overvoltage → regen cap failure; undervoltage → brownout).
#[derive(Clone, Copy, Debug)]
pub struct BusVoltageMonitor {
    vdc_min: f32,
    vdc_max: f32,
}

impl BusVoltageMonitor {
    /// Construct a window monitor.  Caller chooses `vdc_min < vdc_max`; if
    /// the limits are inverted, the window collapses to an empty set and
    /// every input is rejected.
    #[must_use]
    pub const fn new(vdc_min: f32, vdc_max: f32) -> Self {
        Self { vdc_min, vdc_max }
    }

    /// `Ok` when `vdc ∈ [vdc_min, vdc_max]`, `Err` with a static label
    /// otherwise.
    #[must_use]
    pub fn check(&self, vdc: f32) -> Result<(), &'static str> {
        if vdc < self.vdc_min {
            Err("undervoltage")
        } else if vdc > self.vdc_max {
            Err("overvoltage")
        } else {
            Ok(())
        }
    }
}

// ── Stall detector ──────────────────────────────────────────────────────────

/// Stall detector — fires when commanded current is being delivered but
/// the rotor is not moving.  Typical use: trip into `Mode::Off` after a
/// sustained near-zero speed while `Iq` has been non-zero for at least
/// the grace period.
///
/// `armed_at_ms == None` ⇒ detector is inert (returns `false`).  Call
/// [`arm`](Self::arm) once Iq first becomes non-zero; the detector then
/// becomes active for as long as it remains armed.  Call
/// [`disarm`](Self::disarm) when Iq returns to zero or the controller
/// leaves the active mode.
#[derive(Clone, Copy, Debug)]
pub struct StallDetector {
    /// Speed magnitude below which we consider the rotor stalled (rad/s).
    pub min_speed_rad_s: f32,
    /// Grace period after arming (ms) during which the check is suppressed
    /// — the rotor needs time to start moving from rest.
    pub min_iq_after_ms: u32,
    /// Wall-clock time (ms) at which [`arm`](Self::arm) was last called.
    /// `None` ⇒ detector is disarmed / inert.
    pub armed_at_ms: Option<u32>,
}

impl StallDetector {
    #[must_use]
    pub fn new(min_speed_rad_s: f32, min_iq_after_ms: u32) -> Self {
        Self {
            min_speed_rad_s,
            min_iq_after_ms,
            armed_at_ms: None,
        }
    }

    /// Arm the detector (begin grace period).
    pub fn arm(&mut self, now_ms: u32) {
        self.armed_at_ms = Some(now_ms);
    }

    /// Disarm the detector (return to inert).
    pub fn disarm(&mut self) {
        self.armed_at_ms = None;
    }

    /// True when armed, grace period elapsed, and |speed| below threshold.
    #[must_use]
    pub fn check(&self, speed: f32, now_ms: u32) -> bool {
        match self.armed_at_ms {
            None => false,
            Some(t) if now_ms.saturating_sub(t) < self.min_iq_after_ms => false,
            Some(_) => speed.abs() < self.min_speed_rad_s,
        }
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

    // ── InstantOvercurrent ──

    #[test]
    fn instant_oc_below_threshold_safe() {
        let oc = InstantOvercurrent::new(10.0);
        assert!(!oc.check(0.0, 0.0));
        assert!(!oc.check(7.0, 0.0));
        assert!(!oc.check(3.0, 4.0));
        assert!(!oc.check(-7.0, 0.0));
    }

    #[test]
    fn instant_oc_at_threshold_safe() {
        // |i| == threshold does NOT trip — only strictly greater does.
        let oc = InstantOvercurrent::new(10.0);
        assert!(!oc.check(10.0, 0.0));
        assert!(!oc.check(0.0, 10.0));
        assert!(!oc.check(6.0, 8.0));
    }

    #[test]
    fn instant_oc_above_threshold_trips() {
        let oc = InstantOvercurrent::new(10.0);
        assert!(oc.check(10.0001, 0.0));
        assert!(oc.check(7.0, 8.0));
        assert!(oc.check(-11.0, 0.0));
        assert!(oc.check(0.0, -10.0001));
    }

    #[test]
    fn instant_oc_negative_threshold_always_trips() {
        // Misconfigured negative threshold: any non-negative |i| is
        // strictly greater than a negative threshold, so every input trips
        // (including zero).  Documents the failure mode.
        let oc = InstantOvercurrent::new(-1.0);
        assert!(oc.check(0.0, 0.0));
        assert!(oc.check(0.1, 0.0));
    }

    // ── BusVoltageMonitor ──

    #[test]
    fn bus_voltage_inside_window_ok() {
        let mon = BusVoltageMonitor::new(12.0, 48.0);
        assert!(mon.check(12.0).is_ok());
        assert!(mon.check(24.0).is_ok());
        assert!(mon.check(48.0).is_ok());
    }

    #[test]
    fn bus_voltage_undervoltage_err() {
        let mon = BusVoltageMonitor::new(12.0, 48.0);
        assert_eq!(mon.check(11.99), Err("undervoltage"));
        assert_eq!(mon.check(0.0), Err("undervoltage"));
    }

    #[test]
    fn bus_voltage_overvoltage_err() {
        let mon = BusVoltageMonitor::new(12.0, 48.0);
        assert_eq!(mon.check(48.01), Err("overvoltage"));
        assert_eq!(mon.check(100.0), Err("overvoltage"));
    }

    // ── StallDetector ──

    #[test]
    fn stall_disarmed_returns_false() {
        let sd = StallDetector::new(1.0, 100);
        assert!(!sd.check(0.0, 1000));
        assert!(!sd.check(0.0, 0));
    }

    #[test]
    fn stall_within_grace_period_returns_false() {
        let mut sd = StallDetector::new(1.0, 100);
        sd.arm(1000);
        assert!(!sd.check(0.0, 1000));
        assert!(!sd.check(0.0, 1050));
        assert!(!sd.check(0.0, 1099));
    }

    #[test]
    fn stall_after_grace_with_low_speed_trips() {
        let mut sd = StallDetector::new(1.0, 100);
        sd.arm(1000);
        assert!(sd.check(0.0, 1100));
        assert!(sd.check(0.5, 1100));
        assert!(sd.check(-0.999, 1100));
    }

    #[test]
    fn stall_after_grace_with_high_speed_safe() {
        let mut sd = StallDetector::new(1.0, 100);
        sd.arm(1000);
        assert!(!sd.check(1.0, 1100));
        assert!(!sd.check(50.0, 1100));
        assert!(!sd.check(-1.0, 1100));
    }

    #[test]
    fn stall_disarm_resets_to_inert() {
        let mut sd = StallDetector::new(1.0, 100);
        sd.arm(1000);
        sd.disarm();
        assert!(!sd.check(0.0, 5000));
    }

    #[test]
    fn stall_rearm_resets_grace_clock() {
        let mut sd = StallDetector::new(1.0, 100);
        sd.arm(1000);
        assert!(sd.check(0.0, 5000));
        sd.arm(5000);
        assert!(!sd.check(0.0, 5050));
        assert!(sd.check(0.0, 5150));
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
