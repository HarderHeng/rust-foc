//! Motor control task — drives TIM1 from the shared `OPEN_LOOP_CMD`.
//!
//! Runs at 10 kHz on an `embassy_time::Ticker`. Each tick:
//!   1. Snapshots the current shell command (non-blocking lock).
//!   2. Steps the open-loop voltage generator (angle advance, ramp,
//!      SVPWM) → three-phase duty.
//!   3. Writes the duty to TIM1 via `MotorPwm::apply`.
//!   4. Detects enabled-edge transitions to gate the bridge:
//!      - false → true: ramp from 0 V; once the ramp reaches the
//!        target voltage, set MOE = 1.
//!      - true → false: clear MOE = 1 immediately (so the bridge
//!        stops at the next PWM cycle); keep stepping the ramp toward
//!        0 so the duty registers reflect the soft-stop on the way
//!        down, and the next `apply` after the ramp settles at 0 V
//!        leaves a centred duty with the bridge gated off.
//!
//! The motor task is the only `&mut MotorPwm` consumer, so it owns
//! the `MotorPwm` for the lifetime of the executor.

use defmt::info;
use embassy_time::Ticker;

use crate::control::cmd::{OpenLoopCmd, OPEN_LOOP_CMD};
use crate::control::open_loop::OpenLoop;
use crate::drivers::motor_pwm::MotorPwm;
use foc_algo::LibmTrig;

/// Task tick rate. Spec: 10 kHz (100 µs period).
///
/// A `Ticker` (not a `Timer`) so a single long pause — e.g. the
/// shell's blocking write of a long line — can't make us lose
/// accumulated drift: the next tick fires the configured period
/// after the *previous* one, not after the wake.
const MOTOR_TICK_US: u64 = 100;

/// Bus voltage used for SVPWM normalisation.
///
/// Hardcoded at 24 V because the B-G431B-ESC1 board runs on a
/// nominal 24 V supply and we don't yet have an ADC reading for Vbus
/// (ADC comes in a later milestone). A future revision will
/// thread `meas.vdc` through the open-loop state.
const VDC_NOMINAL: f32 = 24.0;

/// How often to emit a defmt trace with the running angle and duty.
/// At 10 kHz, 2000 ticks = 5 Hz — fast enough to spot motion, slow
/// enough to not flood RTT.
const TRACE_EVERY_N_TICKS: u32 = 2000;

#[embassy_executor::task]
pub async fn motor_task(mut pwm: MotorPwm<'static>) {
    info!("motor task started");

    let mut open_loop = OpenLoop::new(VDC_NOMINAL);

    // Cached enabled flag from the previous tick so we can detect
    // false→true (rising) and true→false (falling) edges for the
    // MOE gate. Initialise to "disabled" so the first spin is
    // correctly seen as a rising edge.
    let mut prev_enabled: bool = false;

    let mut tick: u32 = 0;

    // 100 µs ticker. The `.tick()` future is created *once* outside
    // the loop so each iteration awaits the next tick of the same
    // ticker; this matches the spec's note that dt = 100 µs exactly.
    let mut ticker = Ticker::every(embassy_time::Duration::from_micros(MOTOR_TICK_US));

    loop {
        // 1. Snapshot the shell command. The embassy blocking mutex
        //    uses a critical section: this is non-async and never
        //    blocks, so it is safe to call inside the tick loop.
        let cmd: OpenLoopCmd = OPEN_LOOP_CMD.lock(|c| c.get());

        // 2. Edge detection on `enabled`.
        if cmd.enabled && !prev_enabled {
            // Rising edge: MOE stays 0 for now — the OpenLoop ramp
            // will walk voltage from 0 toward the target, and we
            // enable the bridge once the ramp crosses a small
            // threshold so the first powered cycle has a
            // non-trivial duty.
            info!("motor: enable requested (f={} Hz, v={} V)", cmd.freq_hz, cmd.voltage);
        } else if !cmd.enabled && prev_enabled {
            // Falling edge: gate the bridge immediately. The
            // OpenLoop ramp will then walk the duty back to the
            // 50/50 centred "off" state in software.
            info!("motor: stop requested");
            pwm.disable();
        }
        prev_enabled = cmd.enabled;

        // 3. Step the open-loop generator.
        //    dt is the tick period. We pass it in seconds for the
        //    foc-algo API, which expects SI units throughout.
        let duty = open_loop.step::<LibmTrig>(cmd, MOTOR_TICK_US as f32 / 1e6);

        // 4. Write the duty. If we just rose into enabled and the
        //    ramp is still under threshold, MOE stays 0 and the
        //    duty registers are written harmlessly to the bridge
        //    (which is gated off).
        pwm.apply(duty);

        // 5. Rising-edge bridge enable, gated on a small duty so we
        //    don't power up at exactly 50/50 (centred) for one cycle.
        if cmd.enabled && !pwm.is_enabled() {
            // 0.5 ± small = "almost centred" — wait until the duty
            // has visibly departed from 0.5 on at least one phase.
            let max_deviation = (duty.ta - 0.5).abs()
                .max((duty.tb - 0.5).abs())
                .max((duty.tc - 0.5).abs());
            if max_deviation > 0.02 {
                pwm.enable();
                info!("motor: MOE=1 (ramp engaged)");
            }
        }

        // 6. Periodic defmt trace for human-visible motion.
        tick = tick.wrapping_add(1);
        if tick % TRACE_EVERY_N_TICKS == 0 {
            info!(
                "motor: θ={=f32} duty=({=f32},{=f32},{=f32}) moe={}",
                open_loop.theta,
                duty.ta, duty.tb, duty.tc,
                pwm.is_enabled(),
            );
        }

        ticker.next().await;
    }
}
