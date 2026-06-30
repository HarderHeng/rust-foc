//! Feedforward compensation formulas for speed and position loops.
//!
//! Each loop controller accepts an optional feedforward callback
//! (`Option<fn(...) -> f32>`).  This module provides the common physical
//! models that callbacks can delegate to.
//!
//! For a custom model, write your own function matching the callback
//! signature — no trait or struct needed.

/// Standard inertia + viscous friction feedforward.
///
/// ```text
/// ff = inertia_gain · accel + viscous_gain · velocity
/// ```
///
/// # Example
///
/// ```ignore
/// fn my_speed_ff(_ref: f32, speed: f32, accel: f32) -> f32 {
///     inertia_viscous(0.001, 0.0005, accel, speed)
/// }
/// ```
#[inline]
#[must_use]
pub fn inertia_viscous(
    inertia_gain: f32,
    viscous_gain: f32,
    accel: f32,
    velocity: f32,
) -> f32 {
    inertia_gain * accel + viscous_gain * velocity
}

/// Coulomb (dry) friction feedforward.
///
/// Compensates for constant friction that opposes motion, independent of
/// speed.  Common in geared actuators and sliding contacts.
///
/// ```text
/// ff = +coulomb_gain   if velocity > 0
///    = −coulomb_gain   if velocity < 0
///    = 0               if velocity = 0
/// ```
#[inline]
#[must_use]
pub fn coulomb_friction(gain: f32, velocity: f32) -> f32 {
    if velocity > 0.0 { gain } else if velocity < 0.0 { -gain } else { 0.0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }

    #[test]
    fn inertia_only() {
        approx(inertia_viscous(0.5, 0.0, 10.0, 0.0), 5.0);
    }

    #[test]
    fn viscous_only() {
        approx(inertia_viscous(0.0, 0.1, 0.0, 50.0), 5.0);
    }

    #[test]
    fn inertia_viscous_combined() {
        // 0.5 * 10 + 0.1 * 20 = 5 + 2 = 7
        approx(inertia_viscous(0.5, 0.1, 10.0, 20.0), 7.0);
    }

    #[test]
    fn coulomb_positive() {
        approx(coulomb_friction(2.0, 1.0), 2.0);
    }

    #[test]
    fn coulomb_negative() {
        approx(coulomb_friction(2.0, -1.0), -2.0);
    }

    #[test]
    fn coulomb_zero_at_rest() {
        approx(coulomb_friction(2.0, 0.0), 0.0);
    }
}
