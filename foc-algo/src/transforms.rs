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
const INV_SQRT_3: f32 = 0.577_350_27; // 1/√3
const TWO_THIRDS: f32 = 2.0 / 3.0;

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
pub fn park(ab: AlphaBeta, theta: f32) -> Dq {
    let s = sinf(theta);
    let c = cosf(theta);
    Dq {
        d:  c * ab.alpha + s * ab.beta,
        q: -s * ab.alpha + c * ab.beta,
    }
}

/// Inverse Park (dq → αβ).
#[inline]
pub fn inv_park(dq: Dq, theta: f32) -> AlphaBeta {
    let s = sinf(theta);
    let c = cosf(theta);
    AlphaBeta {
        alpha: c * dq.d - s * dq.q,
        beta:  s * dq.d + c * dq.q,
    }
}

// ── Convenience ─────────────────────────────────────────────────────────

/// Chain: ABC → Clarke → Park → dq, in one call.
#[inline]
pub fn abc_to_dq(abc: Abc, theta: f32) -> Dq {
    park(clark(abc), theta)
}

/// Chain: dq → inv-Park → inv-Clarke → ABC, in one call.
#[inline]
pub fn dq_to_abc(dq: Dq, theta: f32) -> Abc {
    inv_clark(inv_park(dq, theta))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clark_abc_to_alpha_beta() {
        // Balanced: a=1, b=-0.5, c=-0.5  →  α=1, β=0
        let ab = clark(Abc { a: 1.0, b: -0.5, c: -0.5 });
        approx(ab.alpha, 1.0);
        approx(ab.beta, 0.0);
    }

    #[test]
    fn clark_balanced_same_as_full() {
        // ia=1, ib=-0.2  →  ic = -(1 + -0.2) = -0.8
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
        // θ=0: d=α, q=β
        let dq = park(AlphaBeta { alpha: 1.0, beta: 0.5 }, 0.0);
        approx(dq.d, 1.0);
        approx(dq.q, 0.5);
    }

    #[test]
    fn park_theta_pi_over_2() {
        // θ=π/2: d=−β, q=α  (simplified rotation)
        let dq = park(AlphaBeta { alpha: 1.0, beta: 0.0 }, core::f32::consts::FRAC_PI_2); // π/2
        approx(dq.d, 0.0);
        approx(dq.q, -1.0);
    }

    #[test]
    fn inv_park_round_trip() {
        let dq = Dq { d: 0.8, q: 0.3 };
        let theta = 1.23;
        let ab = inv_park(dq, theta);
        let back = park(ab, theta);
        approx(back.d, dq.d);
        approx(back.q, dq.q);
    }

    #[test]
    fn abc_to_dq_round_trip() {
        let abc = Abc { a: 0.9, b: -0.3, c: -0.6 };
        let theta = 0.73;
        let dq = abc_to_dq(abc, theta);
        let back = dq_to_abc(dq, theta);
        approx(back.a, abc.a);
        approx(back.b, abc.b);
        approx(back.c, abc.c);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {b}, got {a}");
    }
}
