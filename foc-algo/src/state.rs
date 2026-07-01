//! Per-cycle state for [`crate::FocController`].
//!
//! Three flat structs — one for setpoints, one for sensor inputs, one for
//! diagnostics.  All fields are `pub` because the caller fills them every
//! control cycle before calling `update()`.

/// User-setpoint for the active mode.  Only one of these is read at a time,
/// based on `mode`:
///
/// | Mode            | Field            | Notes                                    |
/// |-----------------|------------------|------------------------------------------|
/// | `Mode::Torque`  | `iq`             | direct Iq setpoint                       |
/// | `Mode::Speed`   | `speed_ref`      | mechanical rad/s                         |
/// | `Mode::Position`| `position`       | mechanical rad                           |
///
/// `id_ref` is consumed by the current loop in every non-Off mode (use it
/// for MTPA / field-weakening).
#[derive(Default, Clone, Copy)]
pub struct Target {
    /// Q-axis current setpoint (A).  Used in `Mode::Torque`.
    pub iq: f32,
    /// Mechanical speed setpoint (rad/s).  Used in `Mode::Speed`.
    pub speed_ref: f32,
    /// Mechanical position setpoint (rad).  Used in `Mode::Position`.
    pub position: f32,
    /// D-axis current reference (A).  Usually ≤ 0 for MTPA / field
    /// weakening.  Applied in every non-Off mode.
    pub id_ref: f32,
}

/// Sensor inputs to the controller.  Fill in once per control cycle before
/// calling [`crate::FocController::update`].
#[derive(Default, Clone, Copy)]
pub struct Meas {
    /// Mechanical position (rad).  Used in `Mode::Position`.
    pub position: f32,
    /// Mechanical speed (rad/s).  Used in `Mode::Speed` and as feedforward
    /// input to the position loop.
    pub speed: f32,
    /// Mechanical acceleration (rad/s²).  Used as feedforward input to
    /// the speed loop.
    pub accel: f32,
    /// Phase A current (A).  From ADC.
    pub ia: f32,
    /// Phase B current (A).  From ADC.  Phase C is derived.
    pub ib: f32,
    /// Rotor **electrical** angle (radians).  From encoder / observer.
    pub theta: f32,
    /// DC bus voltage (V).  0 triggers a safe zero-duty fallback.
    pub vdc: f32,
}

/// Per-cycle controller diagnostics.  Populated by [`crate::FocController::update`].
///
/// Named `ControllerState` (not `Runtime`) so it doesn't collide with the
/// loop-internal `Runtime` structs in `loops::{current,speed,position}`.
#[derive(Default, Clone, Copy)]
pub struct ControllerState {
    /// Iq target after the cascaded loops (pre circle-limiter).
    pub iq_target: f32,
    /// Id reference after circle limitation — what actually enters the
    /// current loop.  Equals `target.id_ref` when the limiter is disabled
    /// or the input vector is already inside the circle.
    pub id_command: f32,
    /// Iq reference after circle limitation — what actually enters the
    /// current loop.  Differs from `iq_target` only when the limiter is
    /// active.
    pub iq_command: f32,
    pub speed_target: f32,
    /// True when circle limitation reduced the current vector this cycle.
    pub current_limited: bool,
    /// True when the demagnetization floor clamped `id_cmd` upward this
    /// cycle (i.e. field weakening demanded a more-negative `Id` than the
    /// motor's magnet could survive).
    pub demag_limited: bool,
}
