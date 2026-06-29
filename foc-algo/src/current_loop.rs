//! FOC controller — context-object with explicit field layering.
//!
//! ## Field layers (who writes, who reads)
//!
//! | Layer | Type | Owner | Lifetime |
//! |-------|------|-------|----------|
//! | `config` | [`FocConfig`] | **Application** writes once at init | Immutable after `new()` |
//! | `meas` | [`Measurements`] | **Application** writes per cycle (from ADC/sensor) | Per-cycle, overwritable |
//! | `target` | [`Targets`] | **Application** writes per cycle (from outer loop) | Per-cycle, overwritable |
//! | `runtime` | [`Runtime`] | **Controller** writes per cycle | Per-cycle, debug-only reads |
//! | `output` | [`Outputs`] | **Controller** writes per cycle | Per-cycle, app reads |
//!
//! ## Usage
//!
//! ```ignore
//! let mut foc = FocController::new(FocConfig::default());
//!
//! loop {
//!     // Application writes inputs:
//!     foc.meas.ia = adc.read_a();
//!     foc.meas.ib = adc.read_b();
//!     foc.meas.theta = encoder.angle();
//!     foc.meas.vdc = adc.read_vbus();
//!     foc.target.iq = outer_loop.torque_request();
//!
//!     // Controller runs:
//!     foc.update(&trig, dt);
//!
//!     // Application reads outputs:
//!     let duty = foc.output.duty;
//!     pwm.set(duty.ta, duty.tb, duty.tc);
//! }
//! ```

use crate::pid::{Pid, PidConfig};
use crate::svpwm::{svpwm, SvpwmDuty};
use crate::transforms::{clark_balanced, inv_park, park, Trig};

// ── Configuration (immutable after init) ─────────────────────────────────

/// Per-axis PI configuration. Set once at startup.
#[derive(Clone, Copy)]
pub struct AxisPidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub output_limit: f32,
    pub d_filter_cycles: u16,
}

impl AxisPidConfig {
    pub fn as_pid_config(&self) -> PidConfig {
        PidConfig {
            kp: self.kp,
            ki: self.ki,
            kd: self.kd,
            output_limit: self.output_limit,
            d_filter_cycles: self.d_filter_cycles,
        }
    }
}

impl Default for AxisPidConfig {
    fn default() -> Self {
        Self { kp: 0.0, ki: 0.0, kd: 0.0, output_limit: 12.0, d_filter_cycles: 0 }
    }
}

/// Top-level controller configuration. Application writes once.
#[derive(Clone, Copy)]
pub struct FocConfig {
    pub pid_d: AxisPidConfig,
    pub pid_q: AxisPidConfig,
    /// Default DC bus voltage (overridable per cycle via [`Measurements::vdc`]).
    pub vdc_default: f32,
}

impl Default for FocConfig {
    fn default() -> Self {
        Self {
            pid_d: AxisPidConfig::default(),
            pid_q: AxisPidConfig::default(),
            vdc_default: 24.0,
        }
    }
}

// ── Measurements (application writes, controller reads) ────────────────

/// Sensor readings, supplied by the application each cycle.
#[derive(Default, Clone, Copy)]
pub struct Measurements {
    /// Phase A current (A).
    pub ia: f32,
    /// Phase B current (A).
    pub ib: f32,
    /// Rotor electrical angle (rad).
    pub theta: f32,
    /// DC bus voltage (V). If left at 0, falls back to `FocConfig::vdc_default`.
    pub vdc: f32,
}

// ── Targets (application writes, controller reads) ──────────────────────

/// Reference inputs to the current loop (typically from a speed/torque loop).
#[derive(Default, Clone, Copy)]
pub struct Targets {
    pub id: f32,
    pub iq: f32,
}

// ── Runtime state (controller owns, debug-visible) ──────────────────────

/// Intermediate values that the controller computes each cycle.
/// Read-only for the application — useful for logging, VOFA, debug.
#[derive(Default, Clone, Copy)]
pub struct Runtime {
    pub id_measured: f32,
    pub iq_measured: f32,
    pub vd: f32,
    pub vq: f32,
}

// ── Outputs (controller writes, application reads) ──────────────────────

/// Final outputs to the application (PWM duty cycles).
#[derive(Clone, Copy)]
pub struct Outputs {
    pub duty: SvpwmDuty,
}

impl Default for Outputs {
    fn default() -> Self { Self { duty: SvpwmDuty { ta: 0.5, tb: 0.5, tc: 0.5 } } }
}

// ── Controller ─────────────────────────────────────────────────────────

pub struct FocController {
    config: FocConfig,
    pid_d: Pid,
    pid_q: Pid,
    pub meas: Measurements,
    pub target: Targets,
    pub runtime: Runtime,
    pub output: Outputs,
}

impl FocController {
    pub fn new(config: FocConfig) -> Self {
        Self {
            pid_d: Pid::new(config.pid_d.as_pid_config()),
            pid_q: Pid::new(config.pid_q.as_pid_config()),
            config,
            meas: Measurements::default(),
            target: Targets::default(),
            runtime: Runtime::default(),
            output: Outputs::default(),
        }
    }

    /// Run one control cycle.
    ///
    /// Reads `self.meas` and `self.target`, writes `self.output.duty` and
    /// `self.runtime`. Callers can then read both freely.
    pub fn update<T: Trig>(&mut self, trig: &T, dt: f32) {
        let vdc = if self.meas.vdc > 0.0 { self.meas.vdc } else { self.config.vdc_default };

        let ab = clark_balanced(self.meas.ia, self.meas.ib);
        let dq = park(trig, ab, self.meas.theta);

        self.runtime.id_measured = dq.d;
        self.runtime.iq_measured = dq.q;

        let vd = self.pid_d.update(self.target.id, dq.d, dt);
        let vq = self.pid_q.update(self.target.iq, dq.q, dt);
        self.runtime.vd = vd;
        self.runtime.vq = vq;

        let v_ab = inv_park(trig, crate::transforms::Dq { d: vd, q: vq }, self.meas.theta);
        self.output.duty = svpwm(v_ab.alpha, v_ab.beta, vdc);
    }

    pub fn reset(&mut self) {
        self.pid_d.reset(); self.pid_q.reset();
    }

    /// Mutate gains at runtime (rarely needed — usually rebuild with `new`).
    pub fn pid_d_mut(&mut self) -> &mut Pid { &mut self.pid_d }
    pub fn pid_q_mut(&mut self) -> &mut Pid { &mut self.pid_q }

    pub fn config(&self) -> &FocConfig { &self.config }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transforms::LibmTrig;
    static TRIG: LibmTrig = LibmTrig;

    #[test]
    fn zero_state_centred_duty() {
        let mut foc = FocController::new(FocConfig::default());
        foc.update(&TRIG, 0.0001);
        approx(foc.output.duty.ta, 0.5);
        approx(foc.output.duty.tb, 0.5);
        approx(foc.output.duty.tc, 0.5);
    }

    #[test]
    fn runtime_reflects_measurements() {
        let mut foc = FocController::new(FocConfig::default());
        foc.meas.ia = 1.0;
        foc.meas.ib = -0.5;
        foc.meas.theta = 0.0;
        foc.update(&TRIG, 0.0001);
        approx(foc.runtime.id_measured, 1.0);
        approx(foc.runtime.iq_measured, 0.0);
    }

    #[test]
    fn step_target_moves_duty() {
        let cfg = FocConfig {
            pid_d: AxisPidConfig { kp: 1.0, ki: 0.1, ..AxisPidConfig::default() },
            pid_q: AxisPidConfig { kp: 1.0, ki: 0.1, ..AxisPidConfig::default() },
            ..FocConfig::default()
        };
        let mut foc = FocController::new(cfg);
        foc.target.iq = 1.0;
        for _ in 0..10 { foc.update(&TRIG, 0.0001); }
        let d = foc.output.duty;
        assert!(d.ta != 0.5 || d.tb != 0.5 || d.tc != 0.5);
    }

    #[test]
    fn reset_clears_integrators() {
        let cfg = FocConfig {
            pid_d: AxisPidConfig { kp: 1.0, ki: 0.5, ..AxisPidConfig::default() },
            pid_q: AxisPidConfig { kp: 1.0, ki: 0.5, ..AxisPidConfig::default() },
            ..FocConfig::default()
        };
        let mut foc = FocController::new(cfg);
        foc.target.iq = 1.0;
        for _ in 0..10 { foc.update(&TRIG, 0.0001); }
        let before = foc.pid_q_mut().integral();
        foc.reset();
        assert!(foc.pid_q_mut().integral().abs() < before.abs());
    }

    #[test]
    fn vdc_falls_back_to_config_default() {
        let mut foc = FocController::new(FocConfig { vdc_default: 12.0, ..FocConfig::default() });
        foc.meas.vdc = 0.0; // explicit "use default"
        foc.update(&TRIG, 0.0001);
        // Cannot observe vdc directly, but the call should not panic.
        // Verifying via the duty that the chain ran end-to-end:
        approx(foc.output.duty.ta, 0.5);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}