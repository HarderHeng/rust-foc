//! Discrete-time PID controller (parallel form).
//!
//! Pure math — no dependencies.  `cargo test` runs the full test suite on host.
//!
//! # Derivation
//!
//! ```text
//!                        dt
//! u(t) = Kp · e(t) + Ki · ∫ e(τ) dτ − Kd · ── y(t)
//!                                             dt
//! ```
//!
//! Uses **derivative on measurement** (setpoint-isolated) to avoid derivative
//! kick when the setpoint changes abruptly.  Integral term uses **clamping
//! anti-windup**: the integrator stops accumulating while the raw (pre-clamp)
//! output exceeds `output_limit`.

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// PID controller configuration.
///
/// Tune these gains and limits for the controlled plant.  All values can be
/// changed at runtime via [`Pid::set_gains`] and [`Pid::set_limits`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidConfig {
    /// Proportional gain Kp.
    pub kp: f32,
    /// Integral gain Ki.
    pub ki: f32,
    /// Derivative gain Kd.
    pub kd: f32,
    /// Maximum absolute output value (symmetric clamping).
    pub output_limit: f32,
    /// Maximum absolute integral term (anti-windup backstop).
    pub integral_limit: f32,
}

impl Default for PidConfig {
    fn default() -> Self {
        Self {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            output_limit: f32::MAX,
            integral_limit: f32::MAX,
        }
    }
}

// ---------------------------------------------------------------------------
// PID controller
// ---------------------------------------------------------------------------

/// Discrete-time PID controller in parallel form.
///
/// # Decoupling
///
/// `Pid` is a **pure math** component — zero dependencies on any HAL, timer,
/// or hardware peripheral.  It works with `f32` and a time delta in seconds.
///
/// # Example
///
/// ```ignore
/// let mut pid = Pid::new(PidConfig {
///     kp: 1.5, ki: 0.1, kd: 0.005,
///     output_limit: 12.0, integral_limit: 10.0,
/// });
///
/// let u = pid.update(target, actual, dt);
/// ```
#[derive(Debug, Clone)]
pub struct Pid {
    cfg: PidConfig,
    integral: f32,
    /// Previous measurement for the D term.
    /// `None` on the first update (D term is skipped).
    prev_measurement: Option<f32>,
    output: f32,
}

impl Pid {
    /// Build a PID controller from a configuration.
    #[inline]
    pub fn new(cfg: PidConfig) -> Self {
        Self {
            cfg,
            integral: 0.0,
            prev_measurement: None,
            output: 0.0,
        }
    }

    /// Advance one control step.
    ///
    /// | Argument | Description |
    /// |----------|-------------|
    /// | `setpoint` | Desired value (target). |
    /// | `measurement` | Actual value (feedback). |
    /// | `dt` | Time since last `update()` call, **in seconds**.  Must be > 0. |
    ///
    /// Returns the clamped control output.
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;

        // ── P term ──
        let p = self.cfg.kp * error;

        // ── D term (on measurement — no derivative kick) ──
        //
        // Derivative on measurement avoids the "derivative kick" that
        // occurs when the setpoint changes abruptly.
        //
        // On the very first update `prev_measurement` is `None`, so D is
        // skipped to avoid a spurious transient from the unknown initial
        // condition.
        let d = match self.prev_measurement {
            Some(prev) if dt > 0.0 => {
                -self.cfg.kd * (measurement - prev) / dt
            }
            _ => 0.0,
        };

        // ── I term with clamping anti-windup ──
        // Only integrate when the raw (pre-clamp) output is inside the limit.
        let raw = p + self.integral + d;
        if raw.abs() < self.cfg.output_limit {
            self.integral += self.cfg.ki * error * dt;
            self.integral = self
                .integral
                .clamp(-self.cfg.integral_limit, self.cfg.integral_limit);
        }

        self.prev_measurement = Some(measurement);
        self.output = raw.clamp(-self.cfg.output_limit, self.cfg.output_limit);
        self.output
    }

    /// Reset integrator and previous-measurement state.
    /// Gains and limits are preserved.
    #[inline]
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_measurement = None;
        self.output = 0.0;
    }

    /// Change PID gains at runtime.
    #[inline]
    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.cfg.kp = kp;
        self.cfg.ki = ki;
        self.cfg.kd = kd;
    }

    /// Change output / integral limits at runtime.
    #[inline]
    pub fn set_limits(&mut self, output_limit: f32, integral_limit: f32) {
        self.cfg.output_limit = output_limit;
        self.cfg.integral_limit = integral_limit;
    }

    // ── Read-only accessors ──

    /// Output value from the most recent [`update()`](Self::update) call.
    #[inline]
    pub fn output(&self) -> f32 {
        self.output
    }

    /// Current integral term.
    #[inline]
    pub fn integral(&self) -> f32 {
        self.integral
    }

    /// Current configuration (gains + limits).
    #[inline]
    pub fn config(&self) -> &PidConfig {
        &self.cfg
    }
}

// ---------------------------------------------------------------------------
// Unit tests — runnable on host (x86_64) with `cargo test`.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// A P-only controller should behave as a simple proportional gain.
    #[test]
    fn p_only_tracks_error() {
        let mut pid = Pid::new(PidConfig {
            kp: 2.0,
            ki: 0.0,
            kd: 0.0,
            output_limit: 100.0,
            integral_limit: 100.0,
        });
        let u = pid.update(10.0, 8.0, 0.001);
        approx(u, 4.0); // 2.0 · (10 − 8) = 4.0
    }

    /// Integral term accumulates over time when error persists.
    #[test]
    fn i_term_accumulates() {
        let mut pid = Pid::new(PidConfig {
            kp: 0.0,
            ki: 1.0,
            kd: 0.0,
            output_limit: 100.0,
            integral_limit: 100.0,
        });
        // Five updates with 1.0 error, each dt = 0.1 s
        // ∫ 1 dt = 0.1 per step → after 5 steps = 0.5
        for _ in 0..5 {
            pid.update(10.0, 9.0, 0.1);
        }
        approx(pid.integral(), 0.5);
    }

    /// D term is zero when measurement is constant.
    #[test]
    fn d_term_constant_measurement() {
        let mut pid = Pid::new(PidConfig {
            kp: 0.0,
            ki: 0.0,
            kd: 5.0,
            output_limit: 100.0,
            integral_limit: 100.0,
        });
        // First update: D is skipped (prev_measurement is None).
        pid.update(10.0, 8.0, 0.001);
        // Second update: measurement unchanged → d(meas)/dt = 0.
        let u = pid.update(10.0, 8.0, 0.001);
        approx(u, 0.0);
    }

    /// A step change in measurement between two updates produces a D spike.
    #[test]
    fn d_term_spikes_on_step() {
        let mut pid = Pid::new(PidConfig {
            kp: 0.0,
            ki: 0.0,
            kd: 1.0,
            output_limit: 10_000.0,
            integral_limit: 100.0,
        });
        // First update: D skipped (initialization).
        pid.update(0.0, 10.0, 0.01);

        // Second update: measurement changes from 10 → 0, dt = 0.01 s
        // d = -1.0 * (0 − 10) / 0.01 = 1000
        let u = pid.update(0.0, 0.0, 0.01);
        approx(u, 1000.0);
    }

    /// Output clamping.
    #[test]
    fn output_clamping() {
        let mut pid = Pid::new(PidConfig {
            kp: 100.0,
            ki: 0.0,
            kd: 0.0,
            output_limit: 10.0,
            integral_limit: 100.0,
        });
        let u = pid.update(100.0, 0.0, 0.001);
        approx(u, 10.0); // clamped
    }

    /// Integral anti-windup: when output is saturated the integrator should
    /// NOT accumulate.
    #[test]
    fn integral_anti_windup() {
        let mut pid = Pid::new(PidConfig {
            kp: 100.0,
            ki: 10.0,
            kd: 0.0,
            output_limit: 10.0, // saturation at ±10
            integral_limit: 100.0,
        });
        // Large error saturates the output immediately.
        pid.update(100.0, 0.0, 1.0);
        let i_after_saturated = pid.integral();
        // Because raw output (Kp·e = 10 000) >> limit, the integrator should
        // have been gated — integral must be 0 (or very close).
        approx(i_after_saturated, 0.0);
    }

    /// `reset()` clears integral and output.
    #[test]
    fn reset_clears_state() {
        let mut pid = Pid::new(PidConfig {
            kp: 1.0,
            ki: 0.1,
            kd: 0.0,
            output_limit: 100.0,
            integral_limit: 100.0,
        });
        pid.update(10.0, 5.0, 0.1);
        assert!(pid.integral() > 0.0);
        assert!(pid.output() > 0.0);

        pid.reset();
        approx(pid.integral(), 0.0);
        approx(pid.output(), 0.0);
    }

    /// `set_gains` changes the coefficients for subsequent updates.
    #[test]
    fn set_gains_takes_effect() {
        let mut pid = Pid::new(PidConfig {
            kp: 1.0,
            ki: 0.0,
            kd: 0.0,
            output_limit: 100.0,
            integral_limit: 100.0,
        });
        pid.set_gains(5.0, 0.0, 0.0);
        let u = pid.update(10.0, 8.0, 0.001);
        approx(u, 10.0); // 5.0 · (10 − 8) = 10
    }

    // ── Approx helper (within 1e-5) ─────────────────────────────────────
    fn approx(a: f32, b: f32) {
        let diff = (a - b).abs();
        assert!(diff < 1e-5, "expected {b}, got {a}");
    }
}
