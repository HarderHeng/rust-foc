//! Cascaded FOC controller — composes speed + current loop into one step.
//!
//! ## Architecture
//!
//! ```text
//! target.speed_ref          meas.accel  meas.speed
//!       │                       │          │
//!       ▼                       ▼          ▼
//!   ┌──────────────────────────────────────────┐
//!   │ SpeedLoopController                       │
//!   │   iq_target ──────────────────────┐      │
//!   │   runtime.*  (debug)              │      │
//!   └────────────────────────────────────│──────┘
//!                                        │
//!                                        ▼
//!   ┌──────────────────────────────────────────┐
//!   │ CurrentLoopController                    │
//!   │   target.iq  ← iq_target                │
//!   │   meas.ia, ib, theta                     │
//!   │   duty → PWM                             │
//!   └──────────────────────────────────────────┘
//! ```
//!
//! ## Field layering
//!
//! | Layer | Type | Owner |
//! |-------|------|-------|
//! | `target` | [`CascadeTarget`] | application writes per cycle |
//! | `meas` | [`CascadeMeasurements`] | application writes per cycle |
//! | `speed`, `current` | inner controllers | composed |
//! | `runtime` | [`CascadeRuntime`] | controllers write per cycle (debug) |
//!
//! ## Usage
//!
//! ```ignore
//! let mut cascade = FocCascade::new();
//! cascade.target.speed_ref = setpoint;
//! cascade.meas.speed = encoder.speed();
//! cascade.meas.accel = encoder.acceleration();
//! cascade.meas.ia = adc.read_a();
//! cascade.meas.ib = adc.read_b();
//! cascade.meas.theta = encoder.angle();
//! cascade.meas.vdc = bus_voltage;
//!
//! cascade.update(&trig, dt);
//!
//! pwm.set(cascade.current.duty);
//! ```

use crate::current_loop_controller::CurrentLoopController;
use crate::pid::Pid;
use crate::speed_loop_controller::SpeedLoopController;
use crate::transforms::Trig;

/// Top-level references the application supplies each cycle.
#[derive(Default, Clone, Copy)]
pub struct CascadeTarget {
    /// Mechanical speed setpoint (rad/s).
    pub speed_ref: f32,
    /// Optional Id reference (e.g. for flux weakening).  Defaults to 0.
    pub id_ref: f32,
}

/// All sensor inputs collected at one place.
#[derive(Default, Clone, Copy)]
pub struct CascadeMeasurements {
    pub speed: f32,
    pub accel: f32,
    pub ia: f32,
    pub ib: f32,
    pub theta: f32,
    pub vdc: f32,
}

/// Aggregated runtime values for logging.
#[derive(Default, Clone, Copy)]
pub struct CascadeRuntime {
    pub iq_target: f32,
    pub id_measured: f32,
    pub iq_measured: f32,
    pub vd: f32,
    pub vq: f32,
}

/// Two-loop cascaded controller.  Application writes `target` + `meas`,
/// calls `update()`, reads `current.duty` for the PWM.
pub struct FocCascade {
    pub target: CascadeTarget,
    pub meas: CascadeMeasurements,
    pub speed: SpeedLoopController,
    pub current: CurrentLoopController,
    pub runtime: CascadeRuntime,
}

impl Default for FocCascade {
    fn default() -> Self {
        Self {
            target: CascadeTarget::default(),
            meas: CascadeMeasurements::default(),
            speed: SpeedLoopController::default(),
            current: CurrentLoopController::default(),
            runtime: CascadeRuntime::default(),
        }
    }
}

impl FocCascade {
    /// Single entry point — runs speed loop, hands Iq to current loop,
    /// updates SVPWM duty.
    pub fn update<T: Trig>(&mut self, dt: f32) {
        // Speed loop → Iq target
        self.speed.meas.speed = self.meas.speed;
        self.speed.meas.accel = self.meas.accel;
        self.speed.target.speed_ref = self.target.speed_ref;
        self.speed.update(dt);

        // Hand off
        self.current.target.iq = self.speed.iq_target;
        self.current.target.id = self.target.id_ref;

        // Current loop measurements
        self.current.meas.ia = self.meas.ia;
        self.current.meas.ib = self.meas.ib;
        self.current.meas.theta = self.meas.theta;
        self.current.svpwm.vdc = self.meas.vdc;
        self.current.update::<T>(dt);

        // Aggregate debug
        self.runtime.iq_target = self.speed.iq_target;
        self.runtime.id_measured = self.current.runtime.id_measured;
        self.runtime.iq_measured = self.current.runtime.iq_measured;
        self.runtime.vd = self.current.runtime.vd;
        self.runtime.vq = self.current.runtime.vq;
    }

    pub fn reset(&mut self) {
        self.speed.reset();
        self.current.reset();
    }

    /// Direct access to the current loop's PI for tuning.
    pub fn current_pid_d(&mut self) -> &mut Pid { &mut self.current.pid_d }
    pub fn current_pid_q(&mut self) -> &mut Pid { &mut self.current.pid_q }
    pub fn speed_pid(&mut self) -> &mut Pid { &mut self.speed.pid }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transforms::LibmTrig;

    #[test]
    fn default_state_no_panic() {
        let mut c = FocCascade::default();
        c.meas.vdc = 24.0;
        c.update::<LibmTrig>(0.0001);
        // vdc=24 + zero voltage vector → centred 0.5 duty
        approx(c.current.duty.ta, 0.5);
    }

    #[test]
    fn speed_target_propagates_to_iq() {
        let mut c = FocCascade::default();
        c.speed.pid.kp = 1.0;
        c.target.speed_ref = 2.0;
        c.meas.speed = 0.0;
        c.update::<LibmTrig>(0.0001);
        // P-only: kp × (2 − 0) = 2.0
        approx(c.runtime.iq_target, 2.0);
    }

    #[test]
    fn current_loop_runs_after_speed() {
        let mut c = FocCascade::default();
        c.speed.pid.kp = 1.0;
        c.target.speed_ref = 1.0;
        c.meas.speed = 0.0;
        c.meas.ia = 1.0;
        c.meas.ib = -0.5;
        c.meas.theta = 0.0;
        c.update::<LibmTrig>(0.0001);
        // The Id measurement should match what we fed in.
        approx(c.runtime.id_measured, 1.0);
    }

    #[test]
    fn feedforward_disabled_blocks_terms() {
        let mut c = FocCascade::default();
        c.speed.pid.kp = 1.0;
        c.speed.feedforward.enabled = false;
        c.speed.feedforward.inertia_gain = 100.0;
        c.target.speed_ref = 2.0;
        c.meas.speed = 0.0;
        c.update::<LibmTrig>(0.0001);
        // Pure P, FF disabled → iq_target = kp × (2 − 0) = 2.0
        approx(c.runtime.iq_target, 2.0);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}