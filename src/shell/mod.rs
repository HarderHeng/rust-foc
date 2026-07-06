//! CLI shell over USART2 (debug UART).
//!
//! Two submodules:
//! - `commands` — embedded-cli command enum + handler implementations
//! - `task`     — async task that owns the UART and drives the CLI

pub mod commands;
pub mod task;
