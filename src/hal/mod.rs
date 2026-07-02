//! Hardware abstraction layer — minimal PAC-level drivers that
//! don't depend on embassy-stm32 HAL.
//!
//! Phase 4 v1 ships just the flash driver; future phases may
//! add GPIO / DMA helpers etc.
#![allow(dead_code)]

pub mod flash;
