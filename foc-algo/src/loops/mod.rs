//! Closed-loop control components.
//!
//! Each loop is a self-contained PI + optional feedforward block.  All loops
//! take measurements as parameters and return a reference — the cascade
//! ([`crate::cascade`]) owns one instance of each and chains them.

pub mod current;
pub mod feedforward;
pub mod position;
pub mod speed;

pub use current::CurrentLoop;
pub use feedforward::{coulomb_friction, inertia_viscous};
pub use position::{PositionFfFn, PositionLoopController};
pub use speed::{SpeedFfFn, SpeedLoopController};
