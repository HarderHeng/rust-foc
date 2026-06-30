//! Circle limitation — vector (id, iq) amplitude cap.
//!
//! When the squared current vector `id² + iq²` exceeds `I_max²`, the entire
//! (id, iq) pair is scaled down to the circle, **preserving phase**.  This is
//! the correct behaviour for FOC: the current magnitude is the true limit
//! (set by motor thermal rating or inverter peak), and shrinking iq alone (the
//! "rectangular" limit) is overly pessimistic.
//!
//! # Why not just clamp iq?
//!
//! A pure `|iq| ≤ I_max` clamp ignores the d-axis.  When the speed loop asks
//! for `iq = 8 A` while field-weakening demands `id = -4 A`, the vector length
//! is `√(64+16) ≈ 8.94 A` — the rectangular clamp lets this through.  Worse,
//! at `iq = 10, id = -10` the vector is `14.14 A` (over limit) but a
//! rectangular clamp would let it through and trip the inverter.
//!
//! ```text
//!     q
//!     │     ╱ I_max
//!     │    ╱
//!     │   ╱· (q,d) inside
//!     │  ╱
//!     │ ╱· (q,d) clamped to circle
//!     │╱___d
//! ```
//!
//! # Usage
//!
//! Insert between speed-loop output and the d/q current references:
//!
//! ```ignore
//! let iq_raw = speed.update(speed_ref, meas_speed, meas_accel, dt);
//! let (id_ref, iq_ref) = circle_limitation(ctrl.target.id_ref, iq_raw, motor.rated_current);
//! ctrl.target.id_ref = id_ref;
//! ctrl.target.iq = iq_ref;
//! ```

/// Limit a 2D current vector to a circle of radius `i_max`.
///
/// Returns `(id, iq)` where the vector magnitude is at most `i_max`,
/// preserving the input phase.  When the input is already inside the circle
/// it is returned unchanged (modulo float rounding).
///
/// # Arguments
/// * `id` — d-axis current (A).  Often negative for flux-weakening/MTPA.
/// * `iq` — q-axis current (A).  Positive for motoring torque.
/// * `i_max` — maximum allowed vector magnitude (A).  Must be > 0.
///
/// # Returns
/// `(id_clamped, iq_clamped)` such that `|id|² + |iq|² ≤ i_max²`.
///
/// When `i_max ≤ 0`, returns `(0, 0)` (safe default).
#[inline]
#[must_use]
pub fn circle_limitation(id: f32, iq: f32, i_max: f32) -> (f32, f32) {
    if i_max <= 0.0 {
        return (0.0, 0.0);
    }

    let square = id * id + iq * iq;
    let max_sq = i_max * i_max;

    if square <= max_sq {
        return (id, iq);
    }

    // Outside the circle — scale to the boundary.  `k = i_max / |v|`
    // preserves phase.  Guard against the (theoretical) zero-magnitude input.
    let magnitude = libm::sqrtf(square);
    if magnitude <= 0.0 {
        return (0.0, 0.0);
    }
    let k = i_max / magnitude;
    (id * k, iq * k)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-5;

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < EPS, "expected {b}, got {a}");
    }

    #[test]
    fn inside_circle_unchanged() {
        let (id, iq) = circle_limitation(2.0, 3.0, 10.0);
        // √(4+9) ≈ 3.6 < 10, no change
        approx(id, 2.0);
        approx(iq, 3.0);
    }

    #[test]
    fn exactly_on_circle_unchanged() {
        let (id, iq) = circle_limitation(3.0, 4.0, 5.0);
        approx(id, 3.0);
        approx(iq, 4.0);
    }

    #[test]
    fn outside_circle_scales_to_boundary() {
        // (3, 4) magnitude 5, i_max=2.5 → scale by 0.5
        let (id, iq) = circle_limitation(3.0, 4.0, 2.5);
        approx(id, 1.5);
        approx(iq, 2.0);
    }

    #[test]
    fn preserves_phase_after_clamp() {
        // Input phase: atan2(4, 3) ≈ 53.13°
        // After clamp to 2.5: (1.5, 2.0), phase should be the same
        let (id, iq) = circle_limitation(3.0, 4.0, 2.5);
        let phase_in = (4.0_f32).atan2(3.0);
        let phase_out = iq.atan2(id);
        approx(phase_in, phase_out);
    }

    #[test]
    fn pure_d_axis_clamps_d() {
        // (10, 0) with i_max=3 → (3, 0)
        let (id, iq) = circle_limitation(10.0, 0.0, 3.0);
        approx(id, 3.0);
        approx(iq, 0.0);
    }

    #[test]
    fn pure_q_axis_clamps_q() {
        // (0, 10) with i_max=3 → (0, 3)
        let (id, iq) = circle_limitation(0.0, 10.0, 3.0);
        approx(id, 0.0);
        approx(iq, 3.0);
    }

    #[test]
    fn negative_d_id_kept() {
        // (-6, 8) magnitude 10, i_max=5 → (-3, 4)
        let (id, iq) = circle_limitation(-6.0, 8.0, 5.0);
        approx(id, -3.0);
        approx(iq, 4.0);
    }

    #[test]
    fn negative_idq_preserved() {
        // Reverse direction: (-3, -4) magnitude 5, i_max=2 → (-1.2, -1.6)
        let (id, iq) = circle_limitation(-3.0, -4.0, 2.0);
        approx(id, -1.2);
        approx(iq, -1.6);
    }

    #[test]
    fn zero_input_returns_zero() {
        let (id, iq) = circle_limitation(0.0, 0.0, 5.0);
        approx(id, 0.0);
        approx(iq, 0.0);
    }

    #[test]
    fn zero_max_returns_zero() {
        let (id, iq) = circle_limitation(3.0, 4.0, 0.0);
        approx(id, 0.0);
        approx(iq, 0.0);
    }

    #[test]
    fn negative_max_returns_zero() {
        let (id, iq) = circle_limitation(3.0, 4.0, -1.0);
        approx(id, 0.0);
        approx(iq, 0.0);
    }

    #[test]
    fn iq_alone_clamped_below_max() {
        // Rectangular-clamp would let this through (|iq|=4 ≤ 5),
        // circle-clamp catches the vector too.
        let (id, iq) = circle_limitation(-3.0, 4.0, 5.0);
        // Magnitude 5, exactly on circle
        let mag = (id * id + iq * iq).sqrt();
        approx(mag, 5.0);
    }

    #[test]
    fn output_never_exceeds_max() {
        // Sweep over a range of inputs to verify magnitude constraint.
        for &(id, iq) in &[
            (10.0, 0.0), (-10.0, 0.0), (0.0, 10.0), (0.0, -10.0),
            (7.0, 7.0), (-7.0, -7.0), (5.0, -12.0),
        ] {
            let (id_c, iq_c) = circle_limitation(id, iq, 5.0);
            let mag = (id_c * id_c + iq_c * iq_c).sqrt();
            assert!(mag <= 5.0 + 1e-4, "|{id},{iq}| → |{id_c},{iq_c}| = {mag} > 5");
        }
    }
}
