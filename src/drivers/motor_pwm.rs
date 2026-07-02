//! Thin wrapper around `embassy_stm32::timer::complementary_pwm::ComplementaryPwm`
//! that drives the three-phase inverter on TIM1 with a 0..1 duty in
//! `foc_algo::Duty` units.
//!
//! Owns the embassy `ComplementaryPwm` instance so that the BSP can hand
//! out a `MotorPwm` by value (no lifetime gymnastics) and the motor
//! task can hold the only `&mut` for the lifetime of the executor.
//!
//! All safety-critical gating goes through the `enable` / `disable`
//! helpers which set/clear the timer master output enable (MOE) bit.
//! The complementary PWM driver is constructed with MOE already 0
//! in [`new`](Self::new) so a brand-new handle is safe to drop on the
//! floor: a stray `apply(Duty)` on a disabled handle drives the duty
//! registers but the outputs stay Hi-Z, so nothing reaches the
//! motor.

// `MotorPwm` is constructed in `bsp::board_init`; the dead_code
// warnings here are expected during the rest of the milestone-1
// wiring and disappear once main.rs spawns the motor task.
#![allow(dead_code)]

use embassy_stm32::timer::complementary_pwm::ComplementaryPwm;
use embassy_stm32::timer::Channel;
use foc_algo::Duty;

/// TIM1 → 3-phase inverter wrapper.
///
/// 20 kHz center-aligned, 750 ns dead-time, MOE-controlled output
/// enable. See `src/bsp.rs` for the timer / pin / frequency setup.
pub struct MotorPwm<'d> {
    inner: ComplementaryPwm<'d, embassy_stm32::peripherals::TIM1>,
    /// Cached so `apply(Duty)` doesn't have to walk into the timer
    /// every tick — `get_max_duty` is a register read on the embassy
    /// driver and we call it at 10 kHz.
    max_duty: u32,
}

impl<'d> MotorPwm<'d> {
    /// Wrap an already-configured `ComplementaryPwm`. The handle is
    /// returned in the **MOE = 0** state so the motor cannot start
    /// turning until the motor task calls [`enable`](Self::enable).
    pub fn new(inner: ComplementaryPwm<'d, embassy_stm32::peripherals::TIM1>) -> Self {
        let max_duty = inner.get_max_duty();
        Self { inner, max_duty }
    }

    /// Master Output Enable = 1 → the three phase legs are connected
    /// to the timer outputs. Idempotent.
    pub fn enable(&mut self) {
        self.inner.set_master_output_enable(true);
    }

    /// Master Output Enable = 0 → outputs go to the idle state
    /// (OIS-active, per the BSP init). Idempotent. The motor task
    /// ramps the duty to 0 *before* calling this so the bridge
    /// transitions through a known-zero state.
    pub fn disable(&mut self) {
        self.inner.set_master_output_enable(false);
    }

    /// True when the timer's MOE bit is set.
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        self.inner.get_master_output_enable()
    }

    /// Write the three-phase duty. `Duty` components are in `[0, 1]`;
    /// they are scaled to the timer's compare range.
    ///
    /// Clamps to the timer's `[0, max_duty]` range to defend against
    /// a future caller that miscomputes the duty.
    pub fn apply(&mut self, duty: Duty) {
        let max = self.max_duty;
        let ta = scale_duty(duty.ta, max);
        let tb = scale_duty(duty.tb, max);
        let tc = scale_duty(duty.tc, max);
        self.inner.set_duty(Channel::Ch1, ta);
        self.inner.set_duty(Channel::Ch2, tb);
        self.inner.set_duty(Channel::Ch3, tc);
    }

    /// The timer's auto-reload value (= max compare value + 1 in
    /// center-aligned mode). Surfaced so the motor task can match
    /// `Duty::to_timer_counts(period)` if it ever needs to.
    #[must_use]
    pub fn arr(&self) -> u16 {
        // max_duty is `max_compare_value`; in center-aligned mode the
        // ARR = max_compare_value (the counter goes 0..ARR..0). For
        // edge-aligned it would be max_duty + 1. The BSP uses
        // center-aligned so this is exact.
        self.max_duty as u16
    }
}

#[inline]
fn scale_duty(d: f32, max: u32) -> u32 {
    if d <= 0.0 {
        0
    } else if d >= 1.0 {
        max
    } else {
        (d * (max as f32)) as u32
    }
}
