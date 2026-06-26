//! Discrete-time PID controller (parallel form).
//!
//! Pure math – no dependencies.  `cargo test` runs on host.
//!
//! ```text
//! u(t) = Kp·e(t) + Ki·∫e − Kd·dy/dt   (derivative on measurement)
//! ```

use core::f32;

/// PID configuration.
///
/// Gains and limits can be changed at runtime via [`Pid::set_gains`] and
/// [`Pid::set_output_limit`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    /// Symmetric output clamp.  Default: unlimited (`f32::MAX`).
    pub output_limit: f32,
    /// Derivative low-pass, in control cycles (τ = N·dt).
    ///
    /// | N | α = 1/(1+N) | Effect |
    /// |---|-------------|--------|
    /// | 0 | 1           | No filter |
    /// | 3 | 1/4         | Moderate |
    /// | 9 | 1/10        | Heavy |
    pub d_filter_cycles: u16,
}

impl Default for PidConfig {
    fn default() -> Self {
        Self { kp: 0.0, ki: 0.0, kd: 0.0, output_limit: f32::MAX, d_filter_cycles: 0 }
    }
}

/// Discrete-time PID in parallel form.  Pure math, no HAL dependency.
///
/// Clamping anti-windup: integrator freezes while the pre-clamp output exceeds
/// the limit.  Derivative on measurement (no setpoint kick).  Optional one-pole
/// low-pass on the derivative (see [`PidConfig::d_filter_cycles`]).
#[derive(Debug, Clone)]
pub struct Pid {
    cfg: PidConfig,
    integral: f32,
    prev_measurement: Option<f32>,
    d_state: f32,  // low-pass filter state for derivative
    output: f32,
}

impl Pid {
    #[inline]
    pub fn new(cfg: PidConfig) -> Self {
        Self { cfg, integral: 0.0, prev_measurement: None, d_state: 0.0, output: 0.0 }
    }

    /// One control step.  `dt` is the elapsed time **in seconds**.
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;
        let p = self.cfg.kp * error;

        // D term: derivative on measurement, optional one-pole low-pass.
        // Skipped on first call (prev_measurement is None).
        let d = match self.prev_measurement {
            Some(prev) if dt > 0.0 => {
                let raw = (measurement - prev) / dt;
                let alpha = if self.cfg.d_filter_cycles > 0 {
                    1.0 / (1.0 + self.cfg.d_filter_cycles as f32)
                } else {
                    1.0
                };
                self.d_state += alpha * (raw - self.d_state);
                -self.cfg.kd * self.d_state
            }
            _ => 0.0,
        };

        // I term with clamping anti-windup
        let raw = p + self.integral + d;
        if raw.abs() < self.cfg.output_limit {
            self.integral += self.cfg.ki * error * dt;
        }

        self.prev_measurement = Some(measurement);
        self.output = raw.clamp(-self.cfg.output_limit, self.cfg.output_limit);
        self.output
    }

    #[inline]
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_measurement = None;
        self.d_state = 0.0;
        self.output = 0.0;
    }

    #[inline]
    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.cfg.kp = kp; self.cfg.ki = ki; self.cfg.kd = kd;
    }

    #[inline]
    pub fn set_output_limit(&mut self, limit: f32) { self.cfg.output_limit = limit; }

    #[inline]
    pub fn set_d_filter_cycles(&mut self, n: u16) { self.cfg.d_filter_cycles = n; }

    // ── Read-only accessors ──

    #[inline] pub fn output(&self) -> f32 { self.output }
    #[inline] pub fn integral(&self) -> f32 { self.integral }
    #[inline] pub fn config(&self) -> &PidConfig { &self.cfg }
}

// ---------------------------------------------------------------------------
// Tests – run `cargo test -p foc-algo --target x86_64-unknown-linux-gnu`
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test] fn p_only() {
        let mut pid = Pid::new(PidConfig { kp: 2.0, ki: 0.0, kd: 0.0, output_limit: 100.0, d_filter_cycles: 0 });
        approx(pid.update(10.0, 8.0, 0.001), 4.0);
    }

    #[test] fn i_accumulates() {
        let mut pid = Pid::new(PidConfig { kp: 0.0, ki: 1.0, kd: 0.0, output_limit: 100.0, d_filter_cycles: 0 });
        for _ in 0..5 { pid.update(10.0, 9.0, 0.1); }
        approx(pid.integral(), 0.5);
    }

    #[test] fn d_zero_when_constant() {
        let mut pid = Pid::new(PidConfig { kp: 0.0, ki: 0.0, kd: 5.0, output_limit: 100.0, d_filter_cycles: 0 });
        pid.update(10.0, 8.0, 0.001);
        approx(pid.update(10.0, 8.0, 0.001), 0.0);
    }

    #[test] fn d_spikes_on_step() {
        let mut pid = Pid::new(PidConfig { kp: 0.0, ki: 0.0, kd: 1.0, output_limit: 10_000.0, d_filter_cycles: 0 });
        pid.update(0.0, 10.0, 0.01);
        approx(pid.update(0.0, 0.0, 0.01), 1000.0);
    }

    #[test] fn output_clamping() {
        let mut pid = Pid::new(PidConfig { kp: 100.0, ki: 0.0, kd: 0.0, output_limit: 10.0, d_filter_cycles: 0 });
        approx(pid.update(100.0, 0.0, 0.001), 10.0);
    }

    #[test] fn anti_windup() {
        let mut pid = Pid::new(PidConfig { kp: 100.0, ki: 10.0, kd: 0.0, output_limit: 10.0, d_filter_cycles: 0 });
        pid.update(100.0, 0.0, 1.0);
        approx(pid.integral(), 0.0);
    }

    #[test] fn reset_clears() {
        let mut pid = Pid::new(PidConfig { kp: 1.0, ki: 0.1, kd: 0.0, output_limit: 100.0, d_filter_cycles: 0 });
        pid.update(10.0, 5.0, 0.1);
        pid.reset();
        approx(pid.integral(), 0.0);
        approx(pid.output(), 0.0);
    }

    #[test] fn set_gains_works() {
        let mut pid = Pid::new(PidConfig { kp: 1.0, ki: 0.0, kd: 0.0, output_limit: 100.0, d_filter_cycles: 0 });
        pid.set_gains(5.0, 0.0, 0.0);
        approx(pid.update(10.0, 8.0, 0.001), 10.0);
    }

    #[test] fn set_d_filter_reduces_spike() {
        let mut pid = Pid::new(PidConfig { kp: 0.0, ki: 0.0, kd: 1.0, output_limit: 10_000.0, d_filter_cycles: 0 });
        pid.update(0.0, 10.0, 0.001);
        let raw = pid.update(0.0, 0.0, 0.001);
        approx(raw, 10_000.0);
        pid.reset();
        pid.set_d_filter_cycles(9);
        pid.update(0.0, 10.0, 0.001);
        assert!(pid.update(0.0, 0.0, 0.001) < raw);
    }

    #[test] fn filtered_d_smooths() {
        let mut pid = Pid::new(PidConfig { kp: 0.0, ki: 0.0, kd: 1.0, output_limit: 10_000.0, d_filter_cycles: 9 });
        pid.update(0.0, 10.0, 0.001);
        let u = pid.update(0.0, 0.0, 0.001);
        assert!(u < 10_000.0, "filtered D = {u}");
        assert!(u > 100.0, "filtered D = {u}");
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
