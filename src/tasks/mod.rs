pub mod heartbeat;
pub mod motor;
pub mod shell;

// The CANopen task lives in `crate::can::canopen` because the
// FDCAN driver init also lives there; re-export here so the
// `tasks` namespace stays flat for `spawner.spawn(tasks::canopen_task(...))`.
pub use crate::can::canopen::canopen_task;
pub use heartbeat::heartbeat;
pub use motor::motor_task;
pub use shell::shell_task;
