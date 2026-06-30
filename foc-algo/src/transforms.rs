//! Clarke and Park transforms for FOC.
//!
//! Pure math — no hardware dependencies.
//!
//! ```text
//! ABC ──[Clarke]──→ αβ ──[Park(θ)]──→ dq
//!                                          ──[inv Park]──→ αβ ──[inv Clarke]──→ ABC
//! ```

#[cfg(feature = "libm-trig")]
use libm::{cosf, sinf};

/// `√3/2` — precomputed half-sqrt3 for Clarke transforms and SVPWM.
pub(crate) const HALF_SQRT3: f32 = 0.866_025_4;
const INV_SQRT_3: f32 = 0.577_350_27;
const TWO_THIRDS: f32 = 2.0 / 3.0;

/// Pluggable trig provider for the Park transforms.
///
/// Stateless — methods take no `&self`.  Apps can substitute `CordicTrig` to
/// call the STM32G4 CORDIC peripheral instead of `libm`.
pub trait Trig {
    fn sin(theta: f32) -> f32;
    fn cos(theta: f32) -> f32;
}

/// Default `Trig` backed by `libm::sinf` / `libm::cosf`.
///
/// Only available when the `libm-trig` feature is enabled (on by default).
/// Disable default features and implement [`Trig`] to use hardware CORDIC or
/// a lookup table instead.
#[cfg(feature = "libm-trig")]
pub struct LibmTrig;
#[cfg(feature = "libm-trig")]
impl Trig for LibmTrig {
    #[inline] fn sin(theta: f32) -> f32 { sinf(theta) }
    #[inline] fn cos(theta: f32) -> f32 { cosf(theta) }
}

/// Three-phase quantities (a, b, c).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Abc {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

/// Stationary αβ frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AlphaBeta {
    pub alpha: f32,
    pub beta: f32,
}

/// Rotating dq frame (synchronised with the rotor).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Dq {
    pub d: f32,
    pub q: f32,
}

// ── Clarke (ABC → αβ) ──

/// Full Clarke transform (amplitude-invariant).
///
/// | α |   | 1     -1/2   -1/2  | | a |
/// | β | = | 0     √3/2  -√3/2 |·| b |
/// | 0 |   | 1/2    1/2    1/2  | | c |
///
/// The zero-sequence component is discarded (assumed balanced).
#[inline]
#[must_use]
pub fn clark(abc: Abc) -> AlphaBeta {
    AlphaBeta {
        alpha: TWO_THIRDS * (abc.a - 0.5 * abc.b - 0.5 * abc.c),
        beta:  TWO_THIRDS * (HALF_SQRT3 * abc.b - HALF_SQRT3 * abc.c),
    }
}

/// Fast Clarke for a balanced system (ia + ib + ic = 0).
///
/// Only two phases are needed: α = ia, β = (ia + 2·ib) / √3.
#[inline]
#[must_use]
pub fn clark_balanced(ia: f32, ib: f32) -> AlphaBeta {
    AlphaBeta { alpha: ia, beta: INV_SQRT_3 * (ia + 2.0 * ib) }
}

/// Inverse Clarke (αβ → ABC).
#[inline]
#[must_use]
pub fn inv_clark(ab: AlphaBeta) -> Abc {
    Abc {
        a: ab.alpha,
        b: -0.5 * ab.alpha + HALF_SQRT3 * ab.beta,
        c: -0.5 * ab.alpha - HALF_SQRT3 * ab.beta,
    }
}

// ── Park (αβ → dq) ──

/// Park transform (αβ → dq).  `theta` = rotor electrical angle (radians).
#[inline]
#[must_use]
pub fn park<T: Trig>(ab: AlphaBeta, theta: f32) -> Dq {
    let s = T::sin(theta);
    let c = T::cos(theta);
    Dq { d: c * ab.alpha + s * ab.beta, q: -s * ab.alpha + c * ab.beta }
}

/// Inverse Park (dq → αβ).
#[inline]
#[must_use]
pub fn inv_park<T: Trig>(dq: Dq, theta: f32) -> AlphaBeta {
    let s = T::sin(theta);
    let c = T::cos(theta);
    AlphaBeta { alpha: c * dq.d - s * dq.q, beta: s * dq.d + c * dq.q }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clark_abc_to_alpha_beta() {
        let ab = clark(Abc { a: 1.0, b: -0.5, c: -0.5 });
        approx(ab.alpha, 1.0);
        approx(ab.beta, 0.0);
    }

    #[test]
    fn clark_balanced_same_as_full() {
        let fast = clark_balanced(1.0, -0.2);
        let full = clark(Abc { a: 1.0, b: -0.2, c: -0.8 });
        approx(fast.alpha, full.alpha);
        approx(fast.beta, full.beta);
    }

    #[test]
    fn inv_clark_round_trip() {
        let ab = AlphaBeta { alpha: 0.8, beta: 0.6 };
        let abc = inv_clark(ab);
        let back = clark(abc);
        approx(back.alpha, ab.alpha);
        approx(back.beta, ab.beta);
    }

    #[test]
    #[cfg(feature = "libm-trig")]
    fn park_theta_zero() {
        let dq = park::<LibmTrig>(AlphaBeta { alpha: 1.0, beta: 0.5 }, 0.0);
        approx(dq.d, 1.0);
        approx(dq.q, 0.5);
    }

    #[test]
    #[cfg(feature = "libm-trig")]
    fn park_theta_pi_over_2() {
        let dq = park::<LibmTrig>(AlphaBeta { alpha: 1.0, beta: 0.0 }, core::f32::consts::FRAC_PI_2);
        approx(dq.d, 0.0);
        approx(dq.q, -1.0);
    }

    #[test]
    #[cfg(feature = "libm-trig")]
    fn inv_park_round_trip() {
        let dq = Dq { d: 0.8, q: 0.3 };
        let theta = 1.23;
        let ab = inv_park::<LibmTrig>(dq, theta);
        let back = park::<LibmTrig>(ab, theta);
        approx(back.d, dq.d);
        approx(back.q, dq.q);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}