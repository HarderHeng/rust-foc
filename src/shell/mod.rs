//! CLI shell over USART2 (debug UART).
//!
//! Two submodules:
//! - `commands` — command parser + executors
//! - `task`     — async task that owns the UART and drives the shell

pub mod commands;
pub mod task;
