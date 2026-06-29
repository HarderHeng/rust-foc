//! FOC controller — mode dispatch over the cascade.
//!
//! | Mode       | Chain                       | Reference field     |
//! |------------|------------------------------|---------------------|
//! | `Off`      | none                         | (none)              |
//! | `Torque`   | current                      | `target.iq`         |
//! | `Speed`    | speed → current              | `target.speed_ref`  |
//! | `Position` | position → speed → current   | `target.position`   |
//!
//! ## Field layers
//!
//! The controller owns ONE flat set of meas / target / runtime / duty fields.
//! Inner loops (position, speed) are private and only expose their PID gains
//! and feedforward through typed accessors for tuning.

use crate::current_loop_controller::CurrentLoop;
use crate::pid::Pid;
use crate::position_loop_controller::PositionLoopController;
use crate::speed_loop_controller::SpeedLoopController;
use crate::svpwm::Duty;
use crate::transforms::Trig;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Mode { Off, Torque, Speed, Position }

impl Default for Mode { fn default() -> Self { Mode::Off } }

#[derive(Default, Clone, Copy)]
pub struct Target {
    pub iq: f32,
    pub speed_ref: f32,
    pub position: f32,
    pub id_ref: f32,
}

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

#[derive(Default, Clone, Copy)]
pub struct Runtime {
    pub iq_target: f32,
    pub speed_target: f32,
}

pub struct FocController {
    pub mode: Mode,
    pub target: Target,
    pub meas: Meas,
    pub runtime: Runtime,
    pub duty: Duty,
    position: PositionLoopController,
    speed: SpeedLoopController,
    current: CurrentLoop,
}

impl Default for FocController { fn default() -> Self { Self::new(Mode::Off) } }

impl FocController {
    pub fn new(mode: Mode) -> Self {
        Self {
            mode,
            target: Target::default(),
            meas: Meas::default(),
            runtime: Runtime::default(),
            duty: Duty::default(),
            position: PositionLoopController::default(),
            speed: SpeedLoopController::default(),
            current: CurrentLoop::new(),
        }
    }

    /// One control cycle.  Picks the chain based on `mode`, then runs the
    /// current loop to update `self.duty`.
    pub fn update<T: Trig>(&mut self, dt: f32) {
        if self.mode == Mode::Off {
            self.runtime.iq_target = 0.0;
            self.runtime.speed_target = 0.0;
            self.duty = Duty { ta: 0.0, tb: 0.0, tc: 0.0 };
            return;
        }

        let (iq_target, speed_target) = match self.mode {
            Mode::Off => unreachable!(),  // handled above
            Mode::Torque => (self.target.iq, 0.0),
            Mode::Speed => {
                self.speed.meas.speed = self.meas.speed;
                self.speed.meas.accel = self.meas.accel;
                self.speed.target.speed_ref = self.target.speed_ref;
                self.speed.update(dt);
                (self.speed.iq_target, self.target.speed_ref)
            }
            Mode::Position => {
                self.position.meas.position = self.meas.position;
                self.position.meas.velocity = self.meas.speed;
                self.position.meas.accel = self.meas.accel;
                self.position.target.position_ref = self.target.position;
                self.position.update(dt);
                let omega_ref = self.position.omega_ref;
                self.speed.meas.speed = self.meas.speed;
                self.speed.meas.accel = self.meas.accel;
                self.speed.target.speed_ref = omega_ref;
                self.speed.update(dt);
                (self.speed.iq_target, omega_ref)
            }
        };
        self.runtime.iq_target = iq_target;
        self.runtime.speed_target = speed_target;

        self.duty = self.current.update::<T>(
            self.meas.ia, self.meas.ib, self.meas.theta,
            self.target.id_ref, iq_target,
            dt,
        );
    }

    pub fn reset(&mut self) {
        self.position.reset();
        self.speed.reset();
        self.current.d.pid.reset();
        self.current.q.pid.reset();
    }

    // ── Tuning accessors ──

    pub fn position_pid(&mut self) -> &mut Pid { &mut self.position.pid }
    pub fn position_feedforward(&mut self) -> &mut crate::position_loop_controller::Feedforward {
        &mut self.position.feedforward
    }
    pub fn speed_pid(&mut self) -> &mut Pid { &mut self.speed.pid }
    pub fn speed_feedforward(&mut self) -> &mut crate::speed_loop_controller::Feedforward {
        &mut self.speed.feedforward
    }
    pub fn current_pid_d(&mut self) -> &mut Pid { &mut self.current.d.pid }
    pub fn current_pid_q(&mut self) -> &mut Pid { &mut self.current.q.pid }
    pub fn current_vdc(&mut self) -> &mut f32 { &mut self.current.svpwm.vdc }
}

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
        approx(c.duty.ta, 0.0);
        approx(c.runtime.iq_target, 0.0);
    }

    #[test] fn torque_mode_passes_iq() {
        let mut c = make(Mode::Torque);
        c.target.iq = 1.5;
        c.update::<LibmTrig>(0.0001);
        approx(c.runtime.iq_target, 1.5);
    }

    #[test] fn speed_mode_runs_speed_loop() {
        let mut c = make(Mode::Speed);
        c.speed_pid().kp = 1.0;
        c.target.speed_ref = 2.0;
        c.update::<LibmTrig>(0.0001);
        approx(c.runtime.iq_target, 2.0);
    }

    #[test] fn position_mode_runs_both_loops() {
        let mut c = make(Mode::Position);
        c.position_pid().kp = 1.0;
        c.speed_pid().kp = 1.0;
        c.target.position = 1.0;
        c.update::<LibmTrig>(0.0001);
        approx(c.runtime.speed_target, 1.0);
        approx(c.runtime.iq_target, 1.0);
    }

    #[test] fn mode_can_be_switched() {
        let mut c = make(Mode::Speed);
        c.speed_pid().kp = 1.0;
        c.target.speed_ref = 2.0;
        c.update::<LibmTrig>(0.0001);
        approx(c.runtime.iq_target, 2.0);
        c.mode = Mode::Off;
        c.update::<LibmTrig>(0.0001);
        approx(c.duty.ta, 0.0);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}