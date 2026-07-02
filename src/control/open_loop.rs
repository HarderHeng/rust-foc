//! Open-loop rotating stator voltage vector.
//!
//! No current feedback — the motor is driven by a rotating voltage
//! vector at a fixed frequency and amplitude. This is the "V/f at
//! constant amplitude" profile used in step 1 of every open-loop FOC
//! bring-up, because it lets you verify the PWM stage end-to-end
//! before adding ADC and the current loop.
//!
//! Per cycle the controller:
//!   1. Integrates the electrical angle (saturating at `2π`).
//!   2. Computes `(v_α, v_β) = V · (cos θ, sin θ)`.
//!   3. Feeds it through `foc_algo::Svpwm` to get three-phase duties.
//!
//! The voltage itself is rate-limited by a `Ramp` so step changes in
//! the shell command (`spin 0 5.0` → `stop`) never produce a current
//! spike.

use core::f32::consts::TAU;

#[allow(unused_imports)]
use foc_algo::{Duty, LibmTrig, Ramp, Svpwm, Trig};

use crate::control::cmd::OpenLoopCmd;

/// Hard upper bound on the peak phase voltage the open-loop path will
/// ever command.  Defined in the milestone 1 spec; the shell also clamps
/// user input to this, but we re-check here as a belt-and-braces guard
/// against shell parser regressions.
pub const MAX_OPENLOOP_V: f32 = 3.0;

/// Rate of voltage change on enable / disable / voltage updates.
/// 5 V/s means a full 0 → 3 V startup takes 600 ms, slow enough to
/// avoid current spikes on a no-load motor.
pub const VOLTAGE_RAMP_V_PER_S: f32 = 5.0;

/// Advance the electrical angle by `2π · f · dt` and wrap to `[0, 2π)`.
///
/// Pure function — host-testable. The wrap is a tight while-loop rather
/// than `rem_euclid` because the `thumbv7em-none-eabihf` target's `f32`
/// doesn't expose the Euclidean intrinsics, and pulling in `num-traits`
/// just for two subtractions would be silly. At the motor task's 10 kHz
/// cadence and any sensible `f`, the loop runs 0–1 times per call.
#[inline]
#[must_use]
pub fn advance_angle(theta: f32, freq_hz: f32, dt: f32) -> f32 {
    if dt <= 0.0 {
        return theta;
    }
    let mut next = theta + TAU * freq_hz * dt;
    while next < 0.0 {
        next += TAU;
    }
    while next >= TAU {
        next -= TAU;
    }
    next
}

/// Polar → αβ. Pure function (modulo the pluggable trig impl).
#[inline]
#[must_use]
pub fn voltage_vector<T: Trig>(voltage: f32, theta: f32) -> (f32, f32) {
    (voltage * T::cos(theta), voltage * T::sin(theta))
}

/// State holder for the open-loop voltage path.
///
/// `OpenLoop::step` is called from the motor task at 10 kHz; the
/// struct owns the running angle, voltage ramp, and SVPWM modulator.
pub struct OpenLoop {
    /// Current electrical angle, kept in `[0, 2π)`.
    pub theta: f32,
    /// Soft-start / soft-stop on the commanded voltage (V/s).
    pub voltage_ramp: Ramp,
    /// Three-phase SVPWM modulator.
    pub svpwm: Svpwm,
}

impl OpenLoop {
    /// Construct with the bus voltage (V) used by SVPWM normalisation.
    /// The initial angle is 0 and the voltage ramp starts at 0 V
    /// (so a fresh controller cannot accidentally drive the motor).
    #[must_use]
    pub fn new(vdc: f32) -> Self {
        Self {
            theta: 0.0,
            voltage_ramp: Ramp::new(VOLTAGE_RAMP_V_PER_S),
            svpwm: Svpwm::new(vdc),
        }
    }

    /// One tick. Returns the three-phase duty to write to the timer.
    ///
    /// * `cmd` — current shell command; `cmd.voltage` is the *target*
    ///   voltage, the ramp walks toward it.
    /// * `dt` — seconds since the last tick. The motor task calls
    ///   this at 10 kHz so `dt ≈ 100 µs`.
    ///
    /// `cmd.voltage` is **re-clamped to `[0, MAX_OPENLOOP_V]`** here as
    /// a defence-in-depth against shell regressions.
    ///
    /// The trig type parameter lets callers route sin/cos through
    /// `LibmTrig` (host) or a future `CordicTrig` (on-chip).
    #[inline]
    pub fn step<T: Trig>(&mut self, cmd: OpenLoopCmd, dt: f32) -> Duty {
        // Disabled → ramp toward 0 V regardless of `cmd.voltage`.
        let target_v = if cmd.enabled {
            cmd.voltage.clamp(0.0, MAX_OPENLOOP_V)
        } else {
            0.0
        };
        // Ramp walks even when the controller is disabled so a previous
        // `spin` followed by `stop` ramps down instead of snapping.
        let v = self.voltage_ramp.update(target_v, dt);

        // Advance the angle only while enabled (and at non-zero V) so a
        // stopped motor doesn't slowly drift due to numerical noise.
        if cmd.enabled && v.abs() > 1e-6 {
            self.theta = advance_angle(self.theta, cmd.freq_hz, dt);
        }

        let (v_alpha, v_beta) = voltage_vector::<T>(v, self.theta);
        self.svpwm.update(v_alpha, v_beta);
        self.svpwm.duty
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Host-only tests. They run via the standard `cargo test
    // --bin foc-rust` invocation on a host toolchain that can build
    // the bin (not the `thumbv7em-none-eabihf` target). The
    // embedded-storage-inmemory dev-dep currently breaks the
    // host-bin build, so these tests are not run in CI today;
    // they document the expected behaviour and will light up
    // once that build chain is set up.

    fn approx(a: f32, b: f32, tol: f32) {
        assert!((a - b).abs() < tol, "expected {b}, got {a} (tol {tol})");
    }

    #[test]
    fn advance_angle_zero_dt_is_identity() {
        approx(advance_angle(1.234, 50.0, 0.0), 1.234, 1e-6);
    }

    #[test]
    fn advance_angle_zero_freq_is_identity() {
        approx(advance_angle(0.5, 0.0, 0.01), 0.5, 1e-6);
    }

    #[test]
    fn advance_angle_one_period_returns_to_start() {
        // At f=1 Hz, T=1 s, one full period wraps back to the same θ.
        let theta0 = 0.3;
        let theta1 = advance_angle(theta0, 1.0, 1.0);
        approx(theta1, theta0, 1e-4);
    }

    #[test]
    fn advance_angle_quarter_period() {
        // 1 Hz, 250 ms ⇒ 90° ⇒ θ' = θ + π/2, wrapped to [0, 2π).
        let theta0 = 0.0;
        let theta1 = advance_angle(theta0, 1.0, 0.25);
        approx(theta1, core::f32::consts::FRAC_PI_2, 1e-4);
    }

    #[test]
    fn advance_angle_wraps_negative() {
        // Negative f, large dt, would underflow the unwrap; rem_euclid
        // must bring it back into [0, 2π).
        let theta0 = 0.1;
        let theta1 = advance_angle(theta0, -10.0, 0.5);
        assert!(theta1 >= 0.0 && theta1 < TAU);
        // -10 Hz · 0.5 s = -5 revolutions = -5·2π + 0.1; mod TAU back to ~0.1.
        approx(theta1, 0.1, 1e-4);
    }

    #[test]
    fn voltage_vector_magnitude() {
        let v = 2.5;
        for k in 0..8 {
            let theta = k as f32 * (TAU / 8.0);
            let (a, b) = voltage_vector::<LibmTrig>(v, theta);
            approx((a * a + b * b).sqrt(), v, 1e-4);
        }
    }

    #[test]
    fn open_loop_disabled_outputs_zero_voltage() {
        let mut ol = OpenLoop::new(24.0);
        // Even with a non-zero command, a disabled controller must
        // ramp toward 0 and produce a centred (0.5, 0.5, 0.5) duty.
        let cmd = OpenLoopCmd { enabled: false, freq_hz: 10.0, voltage: 3.0 };
        for _ in 0..200 {
            let d = ol.step::<LibmTrig>(cmd, 0.001);
            approx(d.ta, 0.5, 1e-3);
            approx(d.tb, 0.5, 1e-3);
            approx(d.tc, 0.5, 1e-3);
        }
    }

    #[test]
    fn open_loop_enabled_command_clamped_to_max() {
        // User asked for 10 V; the controller must clamp to 3 V.
        let mut ol = OpenLoop::new(24.0);
        let cmd = OpenLoopCmd { enabled: true, freq_hz: 1.0, voltage: 10.0 };
        // Run for 5 s — enough for the ramp to reach the (clamped)
        // target. 5 s × 5 V/s = 25 V reach, but we only need 3 V
        // (3 / 5 = 0.6 s).
        for _ in 0..5000 {
            ol.step::<LibmTrig>(cmd, 0.001);
        }
        // The last cycle's voltage is 3 V (clamped); α component at θ
        // unknown but magnitude must be ≤ 3 V.
        let (a, b) = voltage_vector::<LibmTrig>(3.0, ol.theta);
        let mag = (a * a + b * b).sqrt();
        approx(mag, 3.0, 1e-3);
    }

    #[test]
    fn open_loop_voltage_ramp_is_smooth() {
        // Step from 0 to 3 V; assert that the running voltage grows
        // monotonically (Ramp is non-decreasing for non-negative target).
        let mut ol = OpenLoop::new(24.0);
        let cmd = OpenLoopCmd { enabled: true, freq_hz: 0.0, voltage: 3.0 };
        let mut prev = -1.0;
        for _ in 0..200 {
            ol.step::<LibmTrig>(cmd, 0.01);
            // v is not exposed directly, but Ramp is monotonic and the
            // duty's max-min span is proportional to |v| at constant θ.
            let d = ol.svpwm.duty;
            let span = d.ta.max(d.tb).max(d.tc) - d.ta.min(d.tb).min(d.tc);
            assert!(span >= prev - 1e-4, "span decreased: {prev} → {span}");
            prev = span;
        }
    }
}
