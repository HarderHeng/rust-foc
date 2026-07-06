pub mod heartbeat;
pub mod motor;

// The CANopen task lives in `crate::can::canopen` because the
// FDCAN driver init also lives there; re-export here so the
// `tasks` namespace stays flat for `spawner.spawn(tasks::canopen_task(...))`.
pub use crate::drivers::can::canopen::canopen_task;
pub use heartbeat::heartbeat;
pub use motor::motor_task;
pub use crate::shell::task::shell_task;
