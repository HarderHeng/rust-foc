//! Rate limiter (ramp) — softens step changes in reference signals.
//!
//! When the target jumps instantaneously (e.g. speed ref 0 → 100 rad/s), a raw
//! step into the PID causes a large error spike and potential integrator
//! windup.  The ramp converts the step into a linear slope, giving the
//! mechanical system time to follow without saturating the controller.
//!
//! ```text
//! target ──→ [Ramp] ──→ ramped ──→ PID.setpoint
//! ```
//!
//! ## Usage
//!
//! ```ignore
//! let mut ramp = Ramp::new(50.0);     // 50 units/sec max slew rate
//! ramp.set(current_value);           // initialise to avoid first-step jump
//! let ref = ramp.update(target, dt); // call each control cycle
//! ```
//!
//! Set `rate_limit = 0.0` (default) to disable — output tracks target
//! instantly.  Use `set()` to initialise or reset the internal state.

/// Rate-of-change limiter for reference signals.
#[derive(Clone, Copy)]
pub struct Ramp {
    /// Maximum change per second.  0 = disabled (pass-through).
    pub rate_limit: f32,
    value: f32,
}

impl Ramp {
    /// New ramp with the given rate limit.  Internal state starts at 0.
    #[must_use]
    pub const fn new(rate_limit: f32) -> Self {
        Self { rate_limit, value: 0.0 }
    }

    /// Override the internal state (e.g. on initialisation, mode switch, or
    /// fault recovery where you want to resume from the current measurement).
    pub fn set(&mut self, value: f32) {
        self.value = value;
    }

    /// Current ramped value (read-only).
    #[must_use]
    pub fn value(&self) -> f32 {
        self.value
    }

    /// Advance one step toward `target`, respecting `rate_limit`.
    ///
    /// When `rate_limit ≤ 0`, the output jumps to `target` immediately.
    pub fn update(&mut self, target: f32, dt: f32) -> f32 {
        if self.rate_limit <= 0.0 {
            self.value = target;
            return target;
        }
        let max_step = self.rate_limit * dt;
        let error = target - self.value;
        self.value += error.clamp(-max_step, max_step);
        self.value
    }

    /// Reset internal state to 0.
    pub fn reset(&mut self) {
        self.value = 0.0;
    }
}

impl Default for Ramp {
    fn default() -> Self { Self::new(0.0) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn disabled_tracks_instantly() {
        let mut r = Ramp::new(0.0);
        assert!((r.update(100.0, 0.001) - 100.0).abs() < 1e-5);
    }

    #[test]
    fn linear_ramp_up() {
        let mut r = Ramp::new(10.0);
        r.set(0.0);
        let v = r.update(5.0, 0.1);
        approx(v, 1.0);
    }

    #[test]
    fn linear_ramp_down() {
        let mut r = Ramp::new(10.0);
        r.set(5.0);
        let v = r.update(0.0, 0.1);
        approx(v, 4.0);
    }

    #[test]
    fn converges_to_target() {
        let mut r = Ramp::new(50.0);
        for _ in 0..100 { r.update(10.0, 0.01); }
        approx(r.value, 10.0);
    }

    #[test]
    fn set_bypasses_ramp() {
        let mut r = Ramp::new(1.0);
        r.set(100.0);
        approx(r.value, 100.0);
    }

    #[test]
    fn small_step_within_limit() {
        let mut r = Ramp::new(1000.0);
        r.set(0.0);
        let v = r.update(0.5, 0.001);
        approx(v, 0.5);
    }

    #[test]
    fn reset_clears() {
        let mut r = Ramp::new(100.0);
        r.set(50.0);
        r.reset();
        approx(r.value, 0.0);
    }

    #[test]
    fn value_reads_state() {
        let mut r = Ramp::new(10.0);
        r.set(42.0);
        approx(r.value(), 42.0);
    }

    #[test]
    fn set_seeds_nonzero() {
        let mut r = Ramp::new(10.0);
        r.set(5.0);
        approx(r.value(), 5.0);
        let v = r.update(10.0, 0.1);
        approx(v, 6.0);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
