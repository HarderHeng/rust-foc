//! Motor control layer.
//!
//! Milestone 1 only ships the open-loop rotating voltage vector path
//! (`open_loop`); closed-loop / SMO observer / current-loop ISR belong to
//! later milestones.

pub mod cmd;
pub mod open_loop;
