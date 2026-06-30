//! FOC controller — mode dispatch over the cascade.
//!
//! | Mode       | Chain                       | Reference field     |
//! |------------|------------------------------|---------------------|
//! | `Off`      | none                         | (none)              |
//! | `Torque`   | `iq_ramp` → current            | `target.iq`         |
//! | `Speed`    | `speed_ramp` → speed → `iq_ramp` → current | `target.speed_ref` |
//! | `Position` | `pos_ramp` → pos → `speed_ramp` → speed → `iq_ramp` → current | `target.position` |
//!
//! ## Layering
//!
//! The controller owns ONE flat set of `Meas` / `Target` / `Runtime` / `Duty`
//! fields — the single source of truth each cycle.  Inner loops (position,
//! speed, current) are stateless math blocks beyond their PID integrators and
//! SVPWM modulator.  Measurements flow **down** as function parameters; the
//! duty flows **up** as the return value.
//!
//! Three optional [`Ramp`](crate::ramp::Ramp) rate limiters sit between
//! reference sources and their loops.  Each defaults to `rate_limit = 0`
//! (disabled — instant tracking).  Set a positive rate to soften step changes.

use crate::current_loop_controller::CurrentLoop;
use crate::pid::Pid;
use crate::position_loop_controller::{PositionFfFn, PositionLoopController};
use crate::ramp::Ramp;
use crate::speed_loop_controller::{SpeedFfFn, SpeedLoopController};
use crate::svpwm::Duty;
use crate::transforms::Trig;

#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub enum Mode { #[default] Off, Torque, Speed, Position }

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

    /// Rate limiter on the position reference (rad/s²).
    pub pos_ramp: Ramp,
    /// Rate limiter on the speed reference (rad/s²).
    pub speed_ramp: Ramp,
    /// Rate limiter on the Iq reference (A/s).  Applied last, before the
    /// current loop, in every non-Off mode.
    pub iq_ramp: Ramp,

    position: PositionLoopController,
    speed: SpeedLoopController,
    current: CurrentLoop,

    /// Tracks the previous mode so mode-switch ramp seeding can trigger
    /// exactly once at the transition boundary.
    prev_mode: Mode,
}

impl Default for FocController { fn default() -> Self { Self::new(Mode::Off) } }

impl FocController {
    #[must_use]
    pub fn new(mode: Mode) -> Self {
        Self {
            mode,
            target: Target::default(),
            meas: Meas::default(),
            runtime: Runtime::default(),
            duty: Duty::default(),
            pos_ramp: Ramp::default(),
            speed_ramp: Ramp::default(),
            iq_ramp: Ramp::default(),
            position: PositionLoopController::default(),
            speed: SpeedLoopController::default(),
            current: CurrentLoop::new(),
            prev_mode: Mode::Off,
        }
    }

    /// One control cycle.  Picks the chain based on `mode`, then runs the
    /// current loop to update `self.duty`.
    ///
    /// When `dt ≤ 0` the call is a no-op — duty and integrators are frozen.
    /// On mode change, ramps are seeded from current measurements for
    /// bumpless transfer.
    pub fn update<T: Trig>(&mut self, dt: f32) {
        if dt <= 0.0 {
            return;
        }

        // ── Bumpless mode-switch: seed ramps from measurements ──────────
        if self.mode != self.prev_mode {
            match self.mode {
                Mode::Speed => self.speed_ramp.set(self.meas.speed),
                Mode::Position => {
                    self.speed_ramp.set(self.meas.speed);
                    self.pos_ramp.set(self.meas.position);
                }
                Mode::Torque | Mode::Off => {}
            }
        }

        if self.mode == Mode::Off {
            self.runtime.iq_target = 0.0;
            self.runtime.speed_target = 0.0;
            self.duty = Duty { ta: 0.0, tb: 0.0, tc: 0.0 };
            self.reset();
            self.prev_mode = Mode::Off;
            return;
        }

        let (iq_target, speed_target) = match self.mode {
            Mode::Off => unreachable!(),
            Mode::Torque => {
                let iq = self.iq_ramp.update(self.target.iq, dt);
                (iq, 0.0)
            }
            Mode::Speed => {
                let speed_ref = self.speed_ramp.update(self.target.speed_ref, dt);
                let iq = self.speed.update(speed_ref, self.meas.speed, self.meas.accel, dt);
                let iq_final = self.iq_ramp.update(iq, dt);
                (iq_final, speed_ref)
            }
            Mode::Position => {
                let pos_ref = self.pos_ramp.update(self.target.position, dt);
                let omega = self.position.update(
                    pos_ref, self.meas.position,
                    self.meas.speed, self.meas.accel,
                    dt,
                );
                let speed_ref = self.speed_ramp.update(omega, dt);
                let iq = self.speed.update(speed_ref, self.meas.speed, self.meas.accel, dt);
                let iq_final = self.iq_ramp.update(iq, dt);
                (iq_final, speed_ref)
            }
        };
        self.runtime.iq_target = iq_target;
        self.runtime.speed_target = speed_target;
        self.prev_mode = self.mode;

        self.current.set_vdc(self.meas.vdc);
        self.duty = self.current.update::<T>(
            self.meas.ia, self.meas.ib, self.meas.theta,
            self.target.id_ref, iq_target,
            dt,
        );
    }

    pub fn reset(&mut self) {
        self.position.reset();
        self.speed.reset();
        self.current.d_pid.reset();
        self.current.q_pid.reset();
        self.pos_ramp.reset();
        self.speed_ramp.reset();
        self.iq_ramp.reset();
    }

    // ── Tuning accessors ──

    pub fn position_pid(&mut self) -> &mut Pid { &mut self.position.pid }
    pub fn speed_pid(&mut self) -> &mut Pid { &mut self.speed.pid }
    pub fn current_pid_d(&mut self) -> &mut Pid { &mut self.current.d_pid }
    pub fn current_pid_q(&mut self) -> &mut Pid { &mut self.current.q_pid }

    /// Set the speed-loop feedforward callback.  `None` disables feedforward.
    pub fn set_speed_ff(&mut self, cb: Option<SpeedFfFn>) {
        self.speed.ff_callback = cb;
    }

    /// Set the position-loop feedforward callback.  `None` disables feedforward.
    pub fn set_position_ff(&mut self, cb: Option<PositionFfFn>) {
        self.position.ff_callback = cb;
    }
}

#[cfg(all(test, feature = "libm-trig"))]
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

    #[test] fn off_mode_clears_integrators() {
        let mut c = make(Mode::Speed);
        c.speed_pid().ki = 1.0;
        c.target.speed_ref = 1.0;
        c.update::<LibmTrig>(0.1);
        assert!(c.speed.pid.integral != 0.0);
        c.mode = Mode::Off;
        c.update::<LibmTrig>(0.0001);
        approx(c.speed.pid.integral, 0.0);
        approx(c.current.d_pid.integral, 0.0);
        approx(c.current.q_pid.integral, 0.0);
        approx(c.position.pid.integral, 0.0);
    }

    #[test] fn vdc_wired_from_meas_to_svpwm() {
        let mut c = make(Mode::Torque);
        c.meas.vdc = 12.0;
        c.target.iq = 0.0;
        c.update::<LibmTrig>(0.0001);
        approx(c.duty.ta, 0.5);
        c.meas.vdc = 0.0;
        c.update::<LibmTrig>(0.0001);
        approx(c.duty.ta, 0.0);
    }

    #[test] fn iq_ramp_limits_torque_mode() {
        let mut c = make(Mode::Torque);
        c.iq_ramp.rate_limit = 10.0; // 10 A/s
        c.iq_ramp.set(0.0);
        c.target.iq = 100.0;
        c.update::<LibmTrig>(0.001);  // max step = 0.01 A
        assert!(c.runtime.iq_target < 0.02);
    }

    #[test] fn speed_ramp_softens_step() {
        let mut c = make(Mode::Speed);
        c.speed_pid().kp = 1.0;
        c.speed_ramp.rate_limit = 20.0;  // 20 rad/s²
        c.speed_ramp.set(0.0);
        c.target.speed_ref = 100.0;
        c.update::<LibmTrig>(0.001);  // max step = 0.02 rad/s
        assert!(c.runtime.speed_target < 0.03);
    }

    #[test] fn pos_ramp_softens_position_step() {
        let mut c = make(Mode::Position);
        c.position_pid().kp = 1.0;
        c.speed_pid().kp = 1.0;
        c.pos_ramp.rate_limit = 5.0;   // 5 rad/s
        c.pos_ramp.set(0.0);
        c.target.position = 10.0;
        c.update::<LibmTrig>(0.001);   // max step = 0.005 rad
        // omega_ref comes from position loop which sees ramped pos_ref
        // speed_target is omega_ref → speed_ramp → ramped
        assert!(c.runtime.speed_target < 0.006);
    }

    #[test] fn ramps_reset_with_controller() {
        let mut c = make(Mode::Speed);
        c.speed_ramp.rate_limit = 100.0;
        c.speed_ramp.set(50.0);
        c.reset();
        c.mode = Mode::Speed;
        c.speed_pid().kp = 1.0;
        c.target.speed_ref = 0.0;
        c.update::<LibmTrig>(0.001);
        approx(c.runtime.iq_target, 0.0);
    }

    #[test] fn mode_switch_seeds_speed_ramp() {
        let mut c = make(Mode::Torque);
        c.meas.speed = 50.0;
        c.mode = Mode::Speed;
        c.speed_pid().kp = 1.0;
        c.speed_ramp.rate_limit = 10.0;
        c.target.speed_ref = 50.0;
        c.update::<LibmTrig>(0.001);
        assert!(c.runtime.speed_target > 49.0);
    }

    #[test] fn dt_zero_is_noop() {
        let mut c = make(Mode::Speed);
        c.speed_pid().kp = 1.0;
        c.speed_pid().ki = 0.1;
        c.target.speed_ref = 10.0;
        c.update::<LibmTrig>(0.001);
        let duty_before = c.duty;
        let integral_before = c.speed.pid.integral;
        c.update::<LibmTrig>(0.0);
        approx(c.duty.ta, duty_before.ta);
        approx(c.speed.pid.integral, integral_before);
    }

    #[test] fn dt_negative_is_noop() {
        let mut c = make(Mode::Torque);
        c.target.iq = 1.0;
        c.update::<LibmTrig>(0.001);
        let duty_before = c.duty;
        c.update::<LibmTrig>(-0.001);
        approx(c.duty.ta, duty_before.ta);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
