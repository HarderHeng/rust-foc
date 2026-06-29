//! FOC controller — mode dispatch over the cascade.
//!
//! | Mode       | Chain                                     | `target.*` field used |
//! |------------|-------------------------------------------|------------------------|
//! | `Off`      | none                                      | (none)                 |
//! | `Torque`   | current                                   | `target.iq`            |
//! | `Speed`    | speed → current                           | `target.speed_ref`     |
//! | `Position` | position → speed → current                | `target.position`      |
//!
//! Default mode is `Off`.  Current loop is never exposed.

use crate::current_loop_controller::CurrentLoopController;
use crate::pid::Pid;
use crate::position_loop_controller::PositionLoopController;
use crate::speed_loop_controller::SpeedLoopController;
use crate::svpwm::Duty;
use crate::transforms::Trig;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Mode {
    Off,
    Torque,
    Speed,
    Position,
}

impl Default for Mode {
    fn default() -> Self { Mode::Off }
}

/// Reference inputs.  Only the field matching `mode` is read.
#[derive(Default, Clone, Copy)]
pub struct Target {
    pub iq: f32,
    pub speed_ref: f32,
    pub position: f32,
    pub id_ref: f32,
}

/// Sensor inputs.
#[derive(Default, Clone, Copy)]
pub struct Meas {
    pub position: f32,
    pub speed: f32,
    pub accel: f32,
    pub ia: f32,
    pub ib: f32,
    pub theta: f32,
    pub vdc: f32,
}

/// Debug-only output values.
#[derive(Default, Clone, Copy)]
pub struct Debug {
    pub iq_target: f32,
    pub speed_target: f32,
    pub id_measured: f32,
    pub iq_measured: f32,
    pub vd: f32,
    pub vq: f32,
}

pub struct FocController {
    pub mode: Mode,
    pub target: Target,
    pub meas: Meas,
    pub position: PositionLoopController,
    pub speed: SpeedLoopController,
    pub current: CurrentLoopController,
    pub debug: Debug,
}

impl Default for FocController {
    fn default() -> Self { Self::new(Mode::Off) }
}

impl FocController {
    pub fn new(mode: Mode) -> Self {
        Self {
            mode,
            target: Target::default(),
            meas: Meas::default(),
            position: PositionLoopController::default(),
            speed: SpeedLoopController::default(),
            current: CurrentLoopController::default(),
            debug: Debug::default(),
        }
    }

    /// Run one control cycle.  Reads `target` + `meas`, writes `current.duty`
    /// and `debug`.
    pub fn update<T: Trig>(&mut self, dt: f32) {
        let iq_target = match self.mode {
            Mode::Off => {
                self.current.duty = Duty { ta: 0.0, tb: 0.0, tc: 0.0 };
                self.debug.iq_target = 0.0;
                self.debug.speed_target = 0.0;
                return;
            }
            Mode::Torque => {
                self.debug.speed_target = 0.0;
                self.target.iq
            }
            Mode::Speed => self.run_speed(dt),
            Mode::Position => self.run_position(dt),
        };
        self.debug.iq_target = iq_target;
        self.copy_meas_to_current();
        self.current.target.iq = iq_target;
        self.current.target.id = self.target.id_ref;
        self.current.update::<T>(dt);
        let r = &self.current.runtime;
        self.debug.id_measured = r.id_measured;
        self.debug.iq_measured = r.iq_measured;
        self.debug.vd = r.vd;
        self.debug.vq = r.vq;
    }

    pub fn reset(&mut self) {
        self.position.reset();
        self.speed.reset();
        self.current.reset();
    }

    pub fn current_pid_d(&mut self) -> &mut Pid { &mut self.current.pid_d }
    pub fn current_pid_q(&mut self) -> &mut Pid { &mut self.current.pid_q }
    pub fn speed_pid(&mut self) -> &mut Pid { &mut self.speed.pid }
    pub fn position_pid(&mut self) -> &mut Pid { &mut self.position.pid }

    fn run_speed(&mut self, dt: f32) -> f32 {
        self.speed.meas.speed = self.meas.speed;
        self.speed.meas.accel = self.meas.accel;
        self.speed.target.speed_ref = self.target.speed_ref;
        self.speed.update(dt);
        self.debug.speed_target = self.target.speed_ref;
        self.speed.iq_target
    }

    fn run_position(&mut self, dt: f32) -> f32 {
        self.position.meas.position = self.meas.position;
        self.position.meas.velocity = self.meas.speed;
        self.position.meas.accel = self.meas.accel;
        self.position.target.position_ref = self.target.position;
        self.position.update(dt);
        let omega_ref = self.position.omega_ref;
        self.debug.speed_target = omega_ref;
        self.speed.meas.speed = self.meas.speed;
        self.speed.meas.accel = self.meas.accel;
        self.speed.target.speed_ref = omega_ref;
        self.speed.update(dt);
        self.speed.iq_target
    }

    fn copy_meas_to_current(&mut self) {
        self.current.meas.ia = self.meas.ia;
        self.current.meas.ib = self.meas.ib;
        self.current.meas.theta = self.meas.theta;
        self.current.svpwm.vdc = self.meas.vdc;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transforms::LibmTrig;

    fn make(mode: Mode) -> FocController {
        let mut c = FocController::new(mode);
        c.meas.vdc = 24.0;
        c
    }

    #[test] fn default_mode_is_off() {
        assert_eq!(FocController::default().mode, Mode::Off);
    }

    #[test] fn off_mode_zero_duty() {
        let mut c = make(Mode::Off);
        c.update::<LibmTrig>(0.0001);
        approx(c.current.duty.ta, 0.0);
        approx(c.debug.iq_target, 0.0);
    }

    #[test] fn torque_mode_passes_iq() {
        let mut c = make(Mode::Torque);
        c.target.iq = 1.5;
        c.update::<LibmTrig>(0.0001);
        approx(c.debug.iq_target, 1.5);
    }

    #[test] fn speed_mode_runs_speed_loop() {
        let mut c = make(Mode::Speed);
        c.speed.pid.kp = 1.0;
        c.target.speed_ref = 2.0;
        c.update::<LibmTrig>(0.0001);
        approx(c.debug.iq_target, 2.0);
    }

    #[test] fn position_mode_runs_both_loops() {
        let mut c = make(Mode::Position);
        c.position.pid.kp = 1.0;
        c.speed.pid.kp = 1.0;
        c.target.position = 1.0;
        c.update::<LibmTrig>(0.0001);
        approx(c.debug.speed_target, 1.0);
        approx(c.debug.iq_target, 1.0);
    }

    #[test] fn mode_can_be_switched() {
        let mut c = make(Mode::Speed);
        c.speed.pid.kp = 1.0;
        c.target.speed_ref = 2.0;
        c.update::<LibmTrig>(0.0001);
        approx(c.debug.iq_target, 2.0);
        c.mode = Mode::Off;
        c.update::<LibmTrig>(0.0001);
        approx(c.debug.iq_target, 0.0);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}