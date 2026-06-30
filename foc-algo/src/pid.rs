//! Discrete-time PID controller (parallel form).
//!
//! Pure math – no dependencies.  `cargo test` runs on host.
//!
//! ```text
//! u(t) = Kp·e(t) + Ki·∫e − Kd·dy/dt   (derivative on measurement)
//! ```

/// PID controller state.  All fields are public — caller reads/writes directly.
#[derive(Debug, Clone)]
pub struct Pid {
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

    /// Integrator accumulator.  Public for monitoring; do NOT mutate.
    pub integral: f32,
    /// Previous measurement for D-term (None on first call).
    pub prev_measurement: Option<f32>,
    /// Low-pass state for the D term.
    pub d_state: f32,
    /// Most recent clamped output.
    pub output: f32,
}

impl Pid {
    /// Default: zero gains, unlimited output.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            kp: 0.0, ki: 0.0, kd: 0.0,
            output_limit: f32::MAX,
            d_filter_cycles: 0,
            integral: 0.0,
            prev_measurement: None,
            d_state: 0.0,
            output: 0.0,
        }
    }

    /// Reset integrator, D-filter, and output state.  Gains and limits kept.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_measurement = None;
        self.d_state = 0.0;
        self.output = 0.0;
    }

    /// One control step.  `dt` is the elapsed time in seconds.
    ///
    /// When `dt ≤ 0` (invalid or first-cycle), the integrator and derivative
    /// are frozen — only the proportional term is applied.  This is safe
    /// behaviour for timing glitches.
    #[inline]
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;
        let p = self.kp * error;

        let d = match self.prev_measurement {
            Some(prev) if dt > 0.0 => {
                let raw = (measurement - prev) / dt;
                let alpha = if self.d_filter_cycles > 0 {
                    1.0 / (1.0 + f32::from(self.d_filter_cycles))
                } else {
                    1.0
                };
                self.d_state += alpha * (raw - self.d_state);
                -self.kd * self.d_state
            }
            _ => 0.0,
        };

        // Conditional integration: freeze when saturated or dt ≤ 0.
        let raw = p + self.integral + d;
        if dt > 0.0 && raw.abs() < self.output_limit {
            self.integral += self.ki * error * dt;
        }

        self.prev_measurement = Some(measurement);
        self.output = raw.clamp(-self.output_limit, self.output_limit);
        self.output
    }
}

impl Default for Pid {
    fn default() -> Self { Self::new() }
}

/// Combine PI output and feedforward, clamped to the PID's output limit.
///
/// Shared by speed and position loop controllers.
#[inline]
#[must_use]
pub fn combine_pi_ff(pid: &Pid, pi_output: f32, ff_total: f32) -> f32 {
    (pi_output + ff_total).clamp(-pid.output_limit, pid.output_limit)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pid(kp: f32, ki: f32, kd: f32, limit: f32) -> Pid {
        Pid { kp, ki, kd, output_limit: limit, d_filter_cycles: 0,
              integral: 0.0, prev_measurement: None, d_state: 0.0, output: 0.0 }
    }

    #[test] fn p_only() {
        let mut p = pid(2.0, 0.0, 0.0, 100.0);
        approx(p.update(10.0, 8.0, 0.001), 4.0);
    }

    #[test] fn i_accumulates() {
        let mut p = pid(0.0, 1.0, 0.0, 100.0);
        for _ in 0..5 { p.update(10.0, 9.0, 0.1); }
        approx(p.integral, 0.5);
    }

    #[test] fn d_zero_when_constant() {
        let mut p = pid(0.0, 0.0, 5.0, 100.0);
        p.update(10.0, 8.0, 0.001);
        approx(p.update(10.0, 8.0, 0.001), 0.0);
    }

    #[test] fn d_spikes_on_step() {
        let mut p = pid(0.0, 0.0, 1.0, 10_000.0);
        p.update(0.0, 10.0, 0.01);
        approx(p.update(0.0, 0.0, 0.01), 1000.0);
    }

    #[test] fn output_clamping() {
        let mut p = pid(100.0, 0.0, 0.0, 10.0);
        approx(p.update(100.0, 0.0, 0.001), 10.0);
    }

    #[test] fn anti_windup() {
        let mut p = pid(100.0, 10.0, 0.0, 10.0);
        p.update(100.0, 0.0, 1.0);
        approx(p.integral, 0.0);
    }

    #[test] fn reset_clears() {
        let mut p = pid(1.0, 0.1, 0.0, 100.0);
        p.update(10.0, 5.0, 0.1);
        p.reset();
        approx(p.integral, 0.0);
        approx(p.output, 0.0);
    }

    #[test] fn filtered_d_smooths() {
        let mut p = pid(0.0, 0.0, 1.0, 10_000.0);
        p.d_filter_cycles = 9;
        p.update(0.0, 10.0, 0.001);
        let u = p.update(0.0, 0.0, 0.001);
        assert!(u < 10_000.0);
        assert!(u > 100.0);
    }

    #[test] fn dt_zero_freeze_integral() {
        let mut p = pid(1.0, 10.0, 0.0, 100.0);
        p.update(10.0, 0.0, 0.1);
        let integral_before = p.integral;
        let u = p.update(10.0, 0.0, 0.0);
        approx(u, 20.0);
        approx(p.integral, integral_before);
    }

    #[test] fn dt_negative_freeze_integral() {
        let mut p = pid(1.0, 10.0, 0.0, 100.0);
        p.update(10.0, 0.0, 0.1);
        let integral_before = p.integral;
        p.update(10.0, 0.0, -0.1);
        approx(p.integral, integral_before);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}