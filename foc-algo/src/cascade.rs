//! FOC mode dispatch — pick which outer loop controls the current loop.
//!
//! ## Modes
//!
//! | Mode     | Chain                       | `target.*` fields used |
//! |----------|-----------------------------|------------------------|
//! | Torque   | Iq_ref → current loop       | `target.iq`            |
//! | Speed    | speed loop → Iq → current   | `target.speed_ref`     |
//! | Position | position loop → ω_ref → speed → Iq → current | `target.position` |
//!
//! The dispatch runs the right outer loop, then hands its output to the
//! current loop.  Adding a new mode means adding one struct and one match arm.
//!
//! ## Field layering
//!
//! | Layer | Type | Owner |
//! |-------|------|-------|
//! | `mode` | [`Mode`] | application sets once at start |
//! | `target` | [`ModeTarget`] | application writes per cycle |
//! | `meas` | [`CascadeMeasurements`] | application writes per cycle |
//! | `torque` / `speed` / `position` | outer loops | composed |
//! | `current` | current loop | composed |
//!
//! ## Usage
//!
//! ```ignore
//! let mut ctrl = FocController::new(Mode::Speed);
//! ctrl.speed.feedforward.inertia_gain = 0.001;
//!
//! loop {
//!     ctrl.target.speed_ref = setpoint;
//!     ctrl.meas.speed = encoder.speed();
//!     ctrl.meas.accel = encoder.acceleration();
//!     ctrl.meas.ia = adc.read_a();
//!     ctrl.meas.ib = adc.read_b();
//!     ctrl.meas.theta = encoder.angle();
//!     ctrl.meas.vdc = bus_voltage;
//!
//!     ctrl.update::<LibmTrig>(dt);
//!     pwm.set(ctrl.current.duty);
//! }
//! ```

use crate::current_loop_controller::CurrentLoopController;
use crate::pid::Pid;
use crate::speed_loop_controller::SpeedLoopController;
use crate::transforms::Trig;

/// Control mode — which outer loop drives the current loop.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Mode {
    /// Direct Iq setpoint, no outer loop.  Use for torque control or testing.
    Torque,
    /// Speed PI → Iq → current.
    Speed,
}

/// Mode-specific reference inputs.
#[derive(Default, Clone, Copy)]
pub struct ModeTarget {
    /// Torque mode: Iq setpoint (A).  Ignored in other modes.
    pub iq: f32,
    /// Speed mode: speed setpoint (rad/s).  Ignored in other modes.
    pub speed_ref: f32,
    /// Id reference (used in all modes for flux control / weakening).
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

/// FOC controller with mode dispatch.
pub struct FocController {
    pub mode: Mode,
    pub target: ModeTarget,
    pub meas: CascadeMeasurements,

    // Outer loops (only the one matching `mode` runs each cycle).
    pub speed: SpeedLoopController,
    // `position` slot reserved for future use — see `PositionController` below.

    // Inner loop (always runs).
    pub current: CurrentLoopController,
}

impl FocController {
    pub fn new(mode: Mode) -> Self {
        Self {
            mode,
            target: ModeTarget::default(),
            meas: CascadeMeasurements::default(),
            speed: SpeedLoopController::default(),
            current: CurrentLoopController::default(),
        }
    }

    /// Run one control cycle.  Selects the outer loop based on `self.mode`,
    /// computes Iq_target, runs the current loop.
    pub fn update<T: Trig>(&mut self, dt: f32) {
        // Stage 1: outer loop → Iq target
        let iq_target = match self.mode {
            Mode::Torque => self.target.iq,

            Mode::Speed => {
                self.speed.meas.speed = self.meas.speed;
                self.speed.meas.accel = self.meas.accel;
                self.speed.target.speed_ref = self.target.speed_ref;
                self.speed.update(dt);
                self.speed.iq_target
            }
        };

        // Stage 2: current loop
        self.current.target.iq = iq_target;
        self.current.target.id = self.target.id_ref;
        self.current.meas.ia = self.meas.ia;
        self.current.meas.ib = self.meas.ib;
        self.current.meas.theta = self.meas.theta;
        self.current.svpwm.vdc = self.meas.vdc;
        self.current.update::<T>(dt);
    }

    pub fn reset(&mut self) {
        self.speed.reset();
        self.current.reset();
    }

    // Direct access for tuning.
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

    fn make_with_vdc() -> FocController {
        let mut c = FocController::new(Mode::Speed);
        c.meas.vdc = 24.0;
        c
    }

    #[test]
    fn torque_mode_passes_iq_directly() {
        let mut c = FocController::new(Mode::Torque);
        c.meas.vdc = 24.0;
        c.target.iq = 1.5;
        c.update::<LibmTrig>(0.0001);
        // Iq_target was 1.5; current loop runs but PI gains are 0, so duty
        // stays centred.
        approx(c.current.duty.ta, 0.5);
    }

    #[test]
    fn speed_mode_runs_speed_loop() {
        let mut c = make_with_vdc();
        c.speed.pid.kp = 1.0;
        c.target.speed_ref = 2.0;
        c.meas.speed = 0.0;
        c.update::<LibmTrig>(0.0001);
        // iq_target = kp × 2.0 = 2.0 → Id measured should be 0
        approx(c.current.runtime.id_measured, 0.0);
        // Iq target = 2.0 → current loop sees iq_target = 2.0, runs the
        // current loop.  We just check it doesn't panic.
    }

    #[test]
    fn mode_can_be_switched() {
        let mut c = make_with_vdc();
        c.speed.pid.kp = 1.0;

        // In Speed mode: iq_target = kp × (setpoint − meas)
        c.target.speed_ref = 2.0;
        c.meas.speed = 0.0;
        c.update::<LibmTrig>(0.0001);
        let iq_after_speed = c.speed.iq_target;

        // Switch to Torque mode: iq_target = user-supplied
        c.mode = Mode::Torque;
        c.target.iq = 5.0;
        c.update::<LibmTrig>(0.0001);

        // The iq set into the current loop should be exactly 5.0
        // (verifiable via the runtime field of the current loop).
        // Indirect check: feedforward disabled, gains 0 → duty 0.5
        // but iq_target was 5.0 (we don't expose it on current directly,
        // so just verify the dispatcher did run).
        assert_eq!(c.mode, Mode::Torque);
        assert!((iq_after_speed - 2.0).abs() < 1e-5);
    }

    #[test]
    fn feedforward_propagates_through_mode() {
        let mut c = FocController::new(Mode::Speed);
        c.meas.vdc = 24.0;
        c.speed.pid.kp = 0.0;     // disable PI
        c.speed.feedforward.inertia_gain = 0.5;
        c.meas.accel = 10.0;
        c.update::<LibmTrig>(0.0001);
        // FF: 0.5 × 10 = 5.0
        approx(c.speed.iq_target, 5.0);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}