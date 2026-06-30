//! First-order low-pass filter — sensor noise suppression.
//!
//! ```text
//! y[n] = y[n-1] + α · (x[n] − y[n-1]),   α = 2π · f_c · dt  (clamped to [0, 1])
//! ```
//!
//! ## Usage
//!
//! ```ignore
//! let mut lpf = LowPassFilter::new(100.0);  // 100 Hz cutoff
//! let filtered = lpf.update(raw_reading, dt);
//! ```
//!
//! Set `cutoff_hz = 0.0` (default) to disable — output equals input.

/// First-order (single-pole) IIR low-pass filter.
#[derive(Clone, Copy)]
pub struct LowPassFilter {
    /// -3 dB cutoff frequency (Hz).  0 = disabled (pass-through).
    pub cutoff_hz: f32,
    state: f32,
}

impl LowPassFilter {
    #[must_use]
    pub const fn new(cutoff_hz: f32) -> Self {
        Self { cutoff_hz, state: 0.0 }
    }

    /// Seed the filter state (e.g. on initialisation or mode switch).
    pub fn set(&mut self, value: f32) {
        self.state = value;
    }

    /// Filter one sample.  Returns the filtered value.
    pub fn update(&mut self, input: f32, dt: f32) -> f32 {
        if self.cutoff_hz <= 0.0 {
            return input;
        }
        let alpha = (2.0 * core::f32::consts::PI * self.cutoff_hz * dt)
            .clamp(0.0, 1.0);
        self.state += alpha * (input - self.state);
        self.state
    }

    /// True when filtering is active (`cutoff_hz > 0`).  When false, the
    /// filter is a pass-through and `state` is never updated.
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        self.cutoff_hz > 0.0
    }

    pub fn reset(&mut self) {
        self.state = 0.0;
    }
}

impl Default for LowPassFilter {
    fn default() -> Self { Self::new(0.0) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn disabled_is_pass_through() {
        let mut f = LowPassFilter::new(0.0);
        assert!((f.update(42.0, 0.001) - 42.0).abs() < 1e-5);
    }

    #[test]
    fn dc_steady_state() {
        let mut f = LowPassFilter::new(10.0);
        for _ in 0..500 { f.update(5.0, 0.001); }
        approx(f.state, 5.0);
    }

    #[test]
    fn step_response_rising() {
        let mut f = LowPassFilter::new(10.0);
        let y = f.update(1.0, 0.001);
        assert!(y > 0.0 && y < 0.1);
    }

    #[test]
    fn set_seeds_state() {
        let mut f = LowPassFilter::new(10.0);
        f.set(3.0);
        approx(f.state, 3.0);
    }

    #[test]
    fn reset_clears() {
        let mut f = LowPassFilter::new(10.0);
        f.set(5.0);
        f.reset();
        approx(f.state, 0.0);
    }

    #[test]
    fn alpha_clamped_at_one() {
        let mut f = LowPassFilter::new(1_000_000.0);
        let y = f.update(10.0, 0.1);
        approx(y, 10.0);
    }

    #[test]
    fn is_enabled_default_false() {
        let f = LowPassFilter::new(0.0);
        assert!(!f.is_enabled());
    }

    #[test]
    fn is_enabled_when_nonzero() {
        let f = LowPassFilter::new(10.0);
        assert!(f.is_enabled());
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
