pub mod debug_uart;
pub mod flash;

// Re-exports are intentionally kept available for task modules in later tasks
// (5, 6, 7). Suppressing the unused warning until something imports them.
#[allow(unused_imports)]
pub use debug_uart::{DebugShellSink, Uart2Sink};
