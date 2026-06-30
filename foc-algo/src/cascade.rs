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
//! The controller owns ONE flat set of `Meas` / `Target` / `ControllerState` /
//! `Duty` fields — the single source of truth each cycle.  Inner loops
//! (position, speed, current) are stateless math blocks beyond their PID
//! integrators and SVPWM modulator.  Measurements flow **down** as function
//! parameters; the duty flows **up** as the return value.
//!
//! Three optional [`Ramp`](crate::math::ramp::Ramp) rate limiters sit between
//! reference sources and their loops.  Each defaults to `rate_limit = 0`
//! (disabled — instant tracking).  Set a positive rate to soften step changes.

use crate::loops::current::CurrentLoop;
use crate::loops::position::{PositionFfFn, PositionLoopController};
use crate::loops::speed::{SpeedFfFn, SpeedLoopController};
use crate::math::{circle_limitation, Duty, Pid, Ramp, Trig};
use crate::state::{ControllerState, Meas, Target};

#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub enum Mode { #[default] Off, Torque, Speed, Position }

pub struct FocController {
    mode: Mode,
    pub target: Target,
    pub meas: Meas,
    pub runtime: ControllerState,
    pub duty: Duty,

    /// Rate limiter on the position reference.  `rate_limit` is in rad/s
    /// (max change per second).
    pub pos_ramp: Ramp,
    /// Rate limiter on the speed reference.  `rate_limit` is in rad/s²
    /// (max change per second).
    pub speed_ramp: Ramp,
    /// Rate limiter on the Iq reference.  `rate_limit` is in A/s
    /// (max change per second).  Applied last, before the current loop,
    /// in every non-Off mode.  Default: 1000 A/s — safe for small PMSMs
    /// (≈1 A per cycle at 1 kHz control).
    pub iq_ramp: Ramp,

    /// Maximum allowed current vector magnitude (A).  0 = no limit.  When > 0,
    /// the controller applies circle limitation to (id, iq) before the
    /// current loop, ensuring `id² + iq² ≤ current_limit²`.
    current_limit: f32,

    /// When `true`, switching to `Mode::Off` clears all PI integrators and
    /// ramp state — the controller starts "fresh" on the next non-Off
    /// transition.
    ///
    /// When `false` (default), `Mode::Off` only zeros the duty output; PI
    /// integrators, ramp state, and previous mode are preserved.  This
    /// gives **bumpless fault recovery** — when the controller exits Off
    /// (e.g. after a transient overcurrent), the loops continue from
    /// their last state without a wind-up spike.
    ///
    /// Use `true` for hard stops (e.g. E-stop).  Use `false` for normal
    /// enable/disable.
    reset_on_off: bool,

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
            runtime: ControllerState::default(),
            duty: Duty::default(),
            pos_ramp: Ramp::default(),
            speed_ramp: Ramp::default(),
            // iq_ramp defaults to 1000 A/s — a safe slew rate for small
            // PMSMs that prevents mechanical shock from instant torque
            // step changes.  Override with `ctrl.iq_ramp.rate_limit = ...`.
            iq_ramp: Ramp::new(1000.0),
            current_limit: 0.0,
            reset_on_off: false,
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

        // 1. Bumpless mode-switch
        if self.mode != self.prev_mode {
            self.seed_ramps_on_mode_change();
        }

        // 2. Off branch
        if self.mode == Mode::Off {
            self.apply_off_state();
            return;
        }

        // 3. Mode-specific loop chain
        let (iq_target, speed_target) = self.compute_loop_output(dt);
        self.runtime.iq_target = iq_target;
        self.runtime.speed_target = speed_target;
        self.prev_mode = self.mode;

        // 4. Circle limitation
        let (id_cmd, iq_cmd, limited) = self.apply_circle_limit(iq_target);
        self.runtime.id_command = id_cmd;
        self.runtime.iq_command = iq_cmd;
        self.runtime.current_limited = limited;

        // 5. Current loop → PWM duty
        self.current.set_vdc(self.meas.vdc);
        self.duty = self.current.update::<T>(
            self.meas.ia, self.meas.ib, self.meas.theta,
            id_cmd, iq_cmd,
            dt,
        );
    }

    /// Seed ramps from current measurements on mode transitions.
    fn seed_ramps_on_mode_change(&mut self) {
        match self.mode {
            Mode::Speed => self.speed_ramp.set(self.meas.speed),
            Mode::Position => {
                self.speed_ramp.set(self.meas.speed);
                self.pos_ramp.set(self.meas.position);
            }
            Mode::Torque | Mode::Off => {}
        }
    }

    /// Apply Off-mode state: zero duty, optionally reset integrators.
    fn apply_off_state(&mut self) {
        self.runtime.iq_target = 0.0;
        self.runtime.iq_command = 0.0;
        self.runtime.id_command = 0.0;
        self.runtime.speed_target = 0.0;
        self.runtime.current_limited = false;
        self.duty = Duty { ta: 0.0, tb: 0.0, tc: 0.0 };
        if self.reset_on_off {
            self.reset();
        }
        self.prev_mode = Mode::Off;
    }

    /// Run the mode-specific loop chain, returning `(iq_target, speed_target)`.
    fn compute_loop_output(&mut self, dt: f32) -> (f32, f32) {
        match self.mode {
            Mode::Off => unreachable!("Off handled by apply_off_state"),
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
        }
    }

    /// Apply circle limitation to the (id, iq) pair.  Returns the
    /// post-limit `(id, iq)` and a flag indicating whether the limiter
    /// actually reduced the vector.
    ///
    /// When `current_limit ≤ 0`, this is a pass-through.
    fn apply_circle_limit(&self, iq_target: f32) -> (f32, f32, bool) {
        if self.current_limit <= 0.0 {
            return (self.target.id_ref, iq_target, false);
        }
        let (id, iq) = circle_limitation(
            self.target.id_ref, iq_target, self.current_limit,
        );
        // Relative epsilon: 0.01% of the limit.  This swallows the sqrt
        // rounding in `circle_limitation` (a few ULPs) and scales correctly
        // with the current range — a 1 A limit triggers at 1.0001 A, a
        // 100 A limit triggers at 100.01 A.
        let eps = 1e-4 * self.current_limit;
        let was_limited =
            (id - self.target.id_ref).abs() > eps || (iq - iq_target).abs() > eps;
        (id, iq, was_limited)
    }

    /// Clear all PI integrators, ramp state, and observer seed values.
    /// Most users should prefer the soft-off behaviour of `Mode::Off`
    /// with `reset_on_off = true` instead of calling this directly.
    pub(crate) fn reset(&mut self) {
        self.position.reset();
        self.speed.reset();
        self.current.d_pid.reset();
        self.current.q_pid.reset();
        self.pos_ramp.reset();
        self.speed_ramp.reset();
        self.iq_ramp.reset();
    }

    // ── Tuning accessors ──

    /// Set the maximum allowed current vector magnitude (A).  0 disables
    /// circle limitation.
    pub fn set_current_limit(&mut self, amps: f32) {
        self.current_limit = amps;
    }

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

    // ── Configuration accessors (read-only) ──

    /// Current operating mode.
    #[must_use]
    pub fn mode(&self) -> Mode { self.mode }

    /// Switch operating mode.  Ramps are seeded from current measurements
    /// on the next `update()` call for bumpless transfer.
    pub fn set_mode(&mut self, mode: Mode) {
        self.mode = mode;
    }

    /// Current circle-limitation threshold (A).  0 = disabled.
    #[must_use]
    pub fn current_limit(&self) -> f32 { self.current_limit }

    /// True when `Mode::Off` clears PI state (hard stop).  False = soft off
    /// (preserve integrators for fault recovery).
    #[must_use]
    pub fn reset_on_off(&self) -> bool { self.reset_on_off }

    /// Set whether `Mode::Off` clears PI state.  See [`reset_on_off`].
    pub fn set_reset_on_off(&mut self, reset: bool) {
        self.reset_on_off = reset;
    }

    // ── Runtime diagnostics (read-only) ──

    /// Speed-loop per-cycle diagnostics: PI output, feedforward, measurement.
    pub fn speed_runtime(&self) -> &crate::loops::speed::Runtime {
        &self.speed.runtime
    }

    /// Position-loop per-cycle diagnostics: PI output, feedforward, measurement.
    pub fn position_runtime(&self) -> &crate::loops::position::Runtime {
        &self.position.runtime
    }

    /// Current-loop per-cycle diagnostics: d/q currents, voltages, duty.
    pub fn current_runtime(&self) -> &crate::loops::current::Runtime {
        &self.current.runtime
    }
}

#[cfg(all(test, feature = "libm-trig"))]
mod tests {
    use super::*;
    use crate::math::transforms::LibmTrig;

    fn make(mode: Mode) -> FocController {
        let mut c = FocController::new(mode);
        c.meas.vdc = 24.0;
        // Disable iq_ramp by default — most tests don't care about slew
        // limiting.  Specific ramp tests override below.
        c.iq_ramp.rate_limit = 0.0;
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
        c.set_mode(Mode::Off);
        c.update::<LibmTrig>(0.0001);
        approx(c.duty.ta, 0.0);
    }

    #[test] fn off_mode_clears_integrators_when_reset_on_off() {
        let mut c = make(Mode::Speed);
        c.reset_on_off = true;
        c.speed_pid().ki = 1.0;
        c.target.speed_ref = 1.0;
        c.update::<LibmTrig>(0.1);
        assert!(c.speed.pid.integral != 0.0);
        c.set_mode(Mode::Off);
        c.update::<LibmTrig>(0.0001);
        approx(c.speed.pid.integral, 0.0);
        approx(c.current.d_pid.integral, 0.0);
        approx(c.current.q_pid.integral, 0.0);
        approx(c.position.pid.integral, 0.0);
    }

    #[test] fn off_mode_preserves_integrators_by_default() {
        let mut c = make(Mode::Speed);
        c.speed_pid().ki = 1.0;
        c.target.speed_ref = 1.0;
        c.update::<LibmTrig>(0.1);
        let integral_before = c.speed.pid.integral;
        assert!(integral_before != 0.0);
        c.set_mode(Mode::Off);
        c.update::<LibmTrig>(0.0001);
        approx(c.duty.ta, 0.0);
        approx(c.speed.pid.integral, integral_before);
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
        c.iq_ramp.rate_limit = 10.0;
        c.iq_ramp.set(0.0);
        c.target.iq = 100.0;
        c.update::<LibmTrig>(0.001);
        assert!(c.runtime.iq_target < 0.02);
    }

    #[test] fn speed_ramp_softens_step() {
        let mut c = make(Mode::Speed);
        c.speed_pid().kp = 1.0;
        c.speed_ramp.rate_limit = 20.0;
        c.speed_ramp.set(0.0);
        c.target.speed_ref = 100.0;
        c.update::<LibmTrig>(0.001);
        assert!(c.runtime.speed_target < 0.03);
    }

    #[test] fn pos_ramp_softens_position_step() {
        let mut c = make(Mode::Position);
        c.position_pid().kp = 1.0;
        c.speed_pid().kp = 1.0;
        c.pos_ramp.rate_limit = 5.0;
        c.pos_ramp.set(0.0);
        c.target.position = 10.0;
        c.update::<LibmTrig>(0.001);
        assert!(c.runtime.speed_target < 0.006);
    }

    #[test] fn ramps_reset_with_controller() {
        let mut c = make(Mode::Speed);
        c.speed_ramp.rate_limit = 100.0;
        c.speed_ramp.set(50.0);
        c.reset();
        c.set_mode(Mode::Speed);
        c.speed_pid().kp = 1.0;
        c.target.speed_ref = 0.0;
        c.update::<LibmTrig>(0.001);
        approx(c.runtime.iq_target, 0.0);
    }

    #[test] fn mode_switch_seeds_speed_ramp() {
        let mut c = make(Mode::Torque);
        c.meas.speed = 50.0;
        c.set_mode(Mode::Speed);
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

    // ── Circle limitation ──

    #[test] fn circle_limit_disabled_passes_through() {
        let mut c = make(Mode::Torque);
        c.iq_ramp.rate_limit = 0.0;  // disable ramp for this test
        c.target.iq = 100.0;
        c.update::<LibmTrig>(0.001);
        assert!(!c.runtime.current_limited);
        approx(c.runtime.iq_command, c.runtime.iq_target);
        approx(c.runtime.iq_command, 100.0);
    }

    #[test] fn circle_limit_within_circle_unchanged() {
        let mut c = make(Mode::Torque);
        c.set_current_limit(10.0);
        c.target.iq = 5.0;
        c.update::<LibmTrig>(0.001);
        assert!(!c.runtime.current_limited);
        approx(c.runtime.iq_command, 5.0);
    }

    #[test] fn circle_limit_clips_iq_above_max() {
        let mut c = make(Mode::Torque);
        c.set_current_limit(5.0);
        c.target.iq = 10.0;
        c.update::<LibmTrig>(0.001);
        assert!(c.runtime.current_limited);
        approx(c.runtime.iq_command, 5.0);
    }

    #[test] fn circle_limit_respects_d_axis() {
        let mut c = make(Mode::Torque);
        c.set_current_limit(2.5);
        c.target.id_ref = -3.0;
        c.target.iq = 4.0;
        c.update::<LibmTrig>(0.001);
        assert!(c.runtime.current_limited);
        approx(c.runtime.id_command, -1.5);
        approx(c.runtime.iq_command, 2.0);
    }

    #[test] fn circle_limit_off_mode_clears_flag() {
        let mut c = make(Mode::Torque);
        c.set_current_limit(5.0);
        c.target.iq = 10.0;
        c.update::<LibmTrig>(0.001);
        assert!(c.runtime.current_limited);
        c.set_mode(Mode::Off);
        c.update::<LibmTrig>(0.001);
        assert!(!c.runtime.current_limited);
    }

    #[test] fn circle_limit_does_not_mutate_target() {
        // The user's setpoint must survive — only the runtime/command reflects limit.
        let mut c = make(Mode::Torque);
        c.set_current_limit(2.0);
        c.target.id_ref = -1.5;
        c.target.iq = 3.0;
        c.update::<LibmTrig>(0.001);
        approx(c.target.id_ref, -1.5);  // untouched
        approx(c.target.iq, 3.0);
    }

    #[test] fn current_runtime_accessible() {
        let mut c = make(Mode::Torque);
        c.update::<LibmTrig>(0.001);
        // Just verify it returns and has expected fields
        let rt = c.current_runtime();
        let _: f32 = rt.id;
        let _: f32 = rt.iq;
        let _: crate::math::svpwm::Duty = rt.duty;
    }

    // ── End-to-end smoke test ─────────────────────────────────────────
    //
    // Exercises every public API in one chain to catch import-path and
    // API-consistency issues that unit tests miss.

    #[test]
    #[cfg(feature = "libm-trig")]
    fn full_chain_smoke() {
        use crate::motor::MotorParams;
        use crate::observer::{SmoConfig, SmoObserver};
        use crate::math::circle_limitation;
        use crate::{field_weakening, pll_pi_gains};

        let motor = MotorParams {
            r: 0.3, ld: 0.0005, lq: 0.0005,
            flux_linkage: 0.01, pole_pairs: 7,
            continuous_current: 5.0, inertia: 1e-5,
        };

        let (kp_i, ki_i) = motor.current_pi_gains(1000.0);
        let (kp_s, ki_s) = motor.speed_pi_gains(50.0, 1000.0);
        let k_slide = motor.smo_slide_gain(1500.0, 1.5);
        let (kp_pll, ki_pll) = pll_pi_gains(20.0);

        let mut ctrl = FocController::new(Mode::Speed);
        ctrl.set_current_limit(motor.continuous_current);
        ctrl.iq_ramp.rate_limit = 0.0;  // disable ramp for smoke test
        ctrl.speed_pid().kp = kp_s;
        ctrl.speed_pid().ki = ki_s;
        ctrl.current_pid_d().kp = kp_i;
        ctrl.current_pid_d().ki = ki_i;
        ctrl.current_pid_q().kp = kp_i;
        ctrl.current_pid_q().ki = ki_i;

        let mut smo = SmoObserver::new(SmoConfig {
            rs: motor.r, ls: motor.ld,
            k_slide, emf_cutoff: 200.0,
            pll_kp: kp_pll, pll_ki: ki_pll,
        });
        smo.set_angle(0.0);

        let mut angle_seed: f32 = 0.0;
        let dt: f32 = 1.0 / 20_000.0;

        for _ in 0..50 {
            ctrl.meas.ia = 0.1;
            ctrl.meas.ib = -0.05;
            ctrl.meas.theta = smo.theta_hat();
            ctrl.meas.speed = smo.omega_hat();
            ctrl.meas.accel = 0.0;
            ctrl.meas.vdc = 24.0;

            ctrl.target.speed_ref = 100.0;
            ctrl.target.id_ref = field_weakening(
                ctrl.meas.vdc, smo.omega_hat(),
                motor.flux_linkage, motor.ld,
            );

            let (id_c, iq_c) = circle_limitation(
                ctrl.target.id_ref, ctrl.target.iq,
                motor.continuous_current,
            );
            ctrl.target.id_ref = id_c;
            ctrl.target.iq = iq_c;

            ctrl.update::<LibmTrig>(dt);

            let vdq = ctrl.current_runtime();
            smo.update_dq::<LibmTrig>(
                vdq.vd, vdq.vq, angle_seed,
                vdq.id, vdq.iq, dt,
            );
            angle_seed = smo.theta_hat();

            let _ = ctrl.duty
                .apply_dead_time(200, 50_000, ctrl.meas.ia, ctrl.meas.ib,
                                 -(ctrl.meas.ia + ctrl.meas.ib))
                .to_timer_counts(7199);
        }

        // Sanity: duty in [0, 1].
        let d = ctrl.duty;
        assert!(d.ta >= 0.0 && d.ta <= 1.0);
        assert!(d.tb >= 0.0 && d.tb <= 1.0);
        assert!(d.tc >= 0.0 && d.tc <= 1.0);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
