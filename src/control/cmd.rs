//! Shared command structure between the shell task and the motor task.
//!
//! Both tasks run inside the same embassy executor (single thread), so a
//! `CriticalSectionRawMutex` is enough — no `await` is ever needed on the
//! lock. The shell writes; the motor reads. We use `Cell<OpenLoopCmd>`
//! inside the mutex because `OpenLoopCmd` is `Copy` and the access window
//! is "swap then use, atomically w.r.t. interrupts".

use core::cell::Cell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;

/// Shell → motor command. `Copy` so we can hand it off without allocation.
#[derive(Clone, Copy, Debug)]
pub struct OpenLoopCmd {
    /// `true` ⇒ motor task should ramp voltage up and run.
    /// `false` ⇒ motor task should ramp voltage to 0 and gate the PWM.
    pub enabled: bool,
    /// Electrical frequency of the rotating stator vector (Hz).
    /// Sign sets direction. Magnitude is `|f|` — `advance_angle` wraps
    /// `θ` so a very large `f` saturates rather than overflowing.
    pub freq_hz: f32,
    /// Requested peak phase voltage (V). 0 ≤ v ≤ `MAX_OPENLOOP_V` after
    /// the shell clamps the user input. The motor task's
    /// `OpenLoop::step` runs this through `Ramp` so transitions are
    /// smooth.
    pub voltage: f32,
}

impl Default for OpenLoopCmd {
    fn default() -> Self {
        Self { enabled: false, freq_hz: 0.0, voltage: 0.0 }
    }
}

const CMD_DEFAULT: OpenLoopCmd = OpenLoopCmd { enabled: false, freq_hz: 0.0, voltage: 0.0 };

/// Single shared command cell. Both shell and motor task call
/// `try_lock` for a non-blocking read or write. The static init uses a
/// const literal because `Mutex::new(Cell::new(Default::default()))`
/// isn't const-callable.
pub static OPEN_LOOP_CMD: Mutex<CriticalSectionRawMutex, Cell<OpenLoopCmd>> =
    Mutex::new(Cell::new(CMD_DEFAULT));
