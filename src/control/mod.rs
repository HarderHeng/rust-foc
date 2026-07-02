//! Motor control layer.
//!
//! Milestone 1 only ships the open-loop rotating voltage vector path
//! (`open_loop`); closed-loop / SMO observer / current-loop ISR belong to
//! later milestones.
//!
//! `#[allow(dead_code)]` keeps the build warning-free while the wiring
//! (`drivers/motor_pwm.rs`, `tasks/motor.rs`, `spin`/`stop` shell cmds)
//! is landed in follow-up commits. The pure-logic surface (structs,
//! constants, helpers) is already exercised by host unit tests in
//! `open_loop.rs` and `cmd.rs`.

#![allow(dead_code)]

pub mod cmd;
pub mod open_loop;
