//! Feedforward utilities — standard compensation formulas.
//!
//! Each loop controller accepts an optional feedforward callback
//! (`Option<fn(...) -> f32>`).  This module provides the common
//! inertia + viscous friction model as a pure function that callbacks
//! can delegate to.
//!
//! For a custom model, write your own function matching the callback
//! signature — no trait or struct needed.

/// Standard inertia + viscous friction feedforward.
///
/// ```text
/// ff = inertia_gain · accel + viscous_gain · velocity
/// ```
///
/// Call from within a loop's feedforward callback:
///
/// ```ignore
/// fn my_speed_ff(_ref: f32, speed: f32, accel: f32) -> f32 {
///     inertia_viscous(0.001, 0.0005, accel, speed)
/// }
/// ```
#[must_use]
pub fn inertia_viscous(inertia_gain: f32, viscous_gain: f32, accel: f32, velocity: f32) -> f32 {
    inertia_gain * accel + viscous_gain * velocity
}
