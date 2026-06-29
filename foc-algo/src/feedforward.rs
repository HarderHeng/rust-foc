//! Feedforward — shared inertia + viscous compensation for cascade stages.
//!
//! Both the speed loop and position loop use the same feedforward structure:
//! an inertia term proportional to acceleration and a viscous term proportional
//! to velocity.  This module provides the single shared definition.
//!
//! ```text
//! ff = inertia_gain · accel + viscous_gain · velocity
//! ```
//!
//! Set `enabled = false` to bypass all terms without touching individual gains.

/// Configurable feedforward gains shared by speed and position loops.
#[derive(Clone, Copy)]
pub struct Feedforward {
    /// Inertia compensation gain.
    pub inertia_gain: f32,
    /// Viscous friction compensation gain.
    pub viscous_gain: f32,
    /// Master enable — when `false`, `compute()` always returns zeros.
    pub enabled: bool,
}

impl Default for Feedforward {
    fn default() -> Self {
        Self { inertia_gain: 0.0, viscous_gain: 0.0, enabled: true }
    }
}

impl Feedforward {
    /// Evaluate the feedforward terms.
    ///
    /// Returns `(inertia_term, viscous_term, total)`.
    /// All terms are zero when `enabled` is `false`.
    pub fn compute(&self, accel: f32, velocity: f32) -> (f32, f32, f32) {
        if self.enabled {
            let fi = self.inertia_gain * accel;
            let fv = self.viscous_gain * velocity;
            (fi, fv, fi + fv)
        } else {
            (0.0, 0.0, 0.0)
        }
    }
}
