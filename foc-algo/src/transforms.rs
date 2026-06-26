//! Clarke and Park transforms for FOC.
//!
//! Pure math — no hardware dependencies.
//!
//! ```text
//! ABC ──[Clarke]──→ αβ ──[Park(θ)]──→ dq
//!                                          ──[逆 Park]──→ αβ ──[逆 Clarke]──→ ABC
//! ```

use libm::{cosf, sinf};

const SQRT_3: f32 = 1.732_050_8;
const INV_SQRT_3: f32 = 0.577_350_27;
const TWO_THIRDS: f32 = 2.0 / 3.0;

/// Pluggable trig provider for the Park transforms.
///
/// Default implementation `LibmTrig` uses `libm` (works everywhere).
/// An application using a STM32G4 can substitute a `CordicTrig` that
/// calls the hardware CORDIC peripheral (much faster, ~8 cycles for
/// sin+cos in hardware).
pub trait Trig {
    fn sin(&self, theta: f32) -> f32;
    fn cos(&self, theta: f32) -> f32;
}

/// Trig provider backed by `libm::sinf` / `libm::cosf`.
#[derive(Clone, Copy)]
pub struct LibmTrig;

impl Trig for LibmTrig {
    #[inline] fn sin(&self, theta: f32) -> f32 { sinf(theta) }
    #[inline] fn cos(&self, theta: f32) -> f32 { cosf(theta) }
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

// ── Clarke (ABC → αβ) ──────────────────────────────────────────────────

/// Full Clarke transform (amplitude-invariant).
///
/// | α |   | 1     -1/2   -1/2  | | a |
/// | β | = | 0     √3/2  -√3/2 |·| b |
/// | 0 |   | 1/2    1/2    1/2  | | c |
///
/// The zero-sequence component is discarded (assumed balanced).
#[inline]
pub fn clark(abc: Abc) -> AlphaBeta {
    AlphaBeta {
        alpha: TWO_THIRDS * (abc.a - 0.5 * abc.b - 0.5 * abc.c),
        beta:  TWO_THIRDS * (SQRT_3 * 0.5 * abc.b - SQRT_3 * 0.5 * abc.c),
    }
}

/// Fast Clarke for a balanced system (ia + ib + ic = 0).
///
/// Only two phases are needed: α = ia, β = (ia + 2·ib) / √3.
#[inline]
pub fn clark_balanced(ia: f32, ib: f32) -> AlphaBeta {
    AlphaBeta {
        alpha: ia,
        beta:  INV_SQRT_3 * (ia + 2.0 * ib),
    }
}

/// Inverse Clarke (αβ → ABC).
#[inline]
pub fn inv_clark(ab: AlphaBeta) -> Abc {
    Abc {
        a: ab.alpha,
        b: -0.5 * ab.alpha + SQRT_3 * 0.5 * ab.beta,
        c: -0.5 * ab.alpha - SQRT_3 * 0.5 * ab.beta,
    }
}

// ── Park (αβ → dq) ─────────────────────────────────────────────────────

/// Park transform (αβ → dq).  `theta` = rotor electrical angle (radians).
///
/// | d |   |  cos(θ)  sin(θ) | | α |
/// | q | = | −sin(θ)  cos(θ) |·| β |
#[inline]
pub fn park<T: Trig>(trig: &T, ab: AlphaBeta, theta: f32) -> Dq {
    let s = trig.sin(theta);
    let c = trig.cos(theta);
    Dq {
        d:  c * ab.alpha + s * ab.beta,
        q: -s * ab.alpha + c * ab.beta,
    }
}

/// Inverse Park (dq → αβ).
#[inline]
pub fn inv_park<T: Trig>(trig: &T, dq: Dq, theta: f32) -> AlphaBeta {
    let s = trig.sin(theta);
    let c = trig.cos(theta);
    AlphaBeta {
        alpha: c * dq.d - s * dq.q,
        beta:  s * dq.d + c * dq.q,
    }
}

/// Convenience: Park using the default `LibmTrig` trig provider.
#[inline]
pub fn park_default(ab: AlphaBeta, theta: f32) -> Dq {
    park(&LibmTrig, ab, theta)
}

/// Convenience: inverse Park using the default `LibmTrig` trig provider.
#[inline]
pub fn inv_park_default(dq: Dq, theta: f32) -> AlphaBeta {
    inv_park(&LibmTrig, dq, theta)
}

// ── Convenience ─────────────────────────────────────────────────────────

/// Chain: ABC → Clarke → Park → dq, in one call.
#[inline]
pub fn abc_to_dq<T: Trig>(trig: &T, abc: Abc, theta: f32) -> Dq {
    park(trig, clark(abc), theta)
}

/// Chain: dq → inv-Park → inv-Clarke → ABC, in one call.
#[inline]
pub fn dq_to_abc<T: Trig>(trig: &T, dq: Dq, theta: f32) -> Abc {
    inv_clark(inv_park(trig, dq, theta))
}

/// Convenience: ABC → dq with default `LibmTrig`.
#[inline]
pub fn abc_to_dq_default(abc: Abc, theta: f32) -> Dq {
    abc_to_dq(&LibmTrig, abc, theta)
}

/// Convenience: dq → ABC with default `LibmTrig`.
#[inline]
pub fn dq_to_abc_default(dq: Dq, theta: f32) -> Abc {
    dq_to_abc(&LibmTrig, dq, theta)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    static TRIG: LibmTrig = LibmTrig;

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
    fn park_theta_zero() {
        let dq = park(&TRIG, AlphaBeta { alpha: 1.0, beta: 0.5 }, 0.0);
        approx(dq.d, 1.0);
        approx(dq.q, 0.5);
    }

    #[test]
    fn park_theta_pi_over_2() {
        let dq = park(&TRIG, AlphaBeta { alpha: 1.0, beta: 0.0 }, core::f32::consts::FRAC_PI_2);
        approx(dq.d, 0.0);
        approx(dq.q, -1.0);
    }

    #[test]
    fn inv_park_round_trip() {
        let dq = Dq { d: 0.8, q: 0.3 };
        let theta = 1.23;
        let ab = inv_park(&TRIG, dq, theta);
        let back = park(&TRIG, ab, theta);
        approx(back.d, dq.d);
        approx(back.q, dq.q);
    }

    #[test]
    fn abc_to_dq_round_trip() {
        let abc = Abc { a: 0.9, b: -0.3, c: -0.6 };
        let theta = 0.73;
        let dq = abc_to_dq(&TRIG, abc, theta);
        let back = dq_to_abc(&TRIG, dq, theta);
        approx(back.a, abc.a);
        approx(back.b, abc.b);
        approx(back.c, abc.c);
    }

    #[test]
    fn park_default_matches_libm() {
        // Both paths should give the same result.
        let dq1 = park(&TRIG, AlphaBeta { alpha: 0.5, beta: 0.3 }, 0.7);
        let dq2 = park_default(AlphaBeta { alpha: 0.5, beta: 0.3 }, 0.7);
        approx(dq1.d, dq2.d);
        approx(dq1.q, dq2.q);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
