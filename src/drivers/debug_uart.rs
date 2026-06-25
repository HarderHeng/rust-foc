//! Debug serial output driver for USART2.
//!
//! `Uart2Sink` wraps a `BufferedUart` and implements both:
//! - [`DebugShellSink`]: high-level `&str` writing for app tasks
//! - `embedded_io::Write`: standard interface for third-party libs (e.g. `embedded-cli`)
//!
//! All blocking semantics: this sink writes synchronously to a TX ringbuffer.
//! Long blocking is only possible if the ringbuffer is full.

#![allow(dead_code)]

/// High-level sink for debug shell output.
///
/// Tasks should depend on this trait, not on the concrete USART type.
/// This keeps the application layer independent of the transport.
pub trait DebugShellSink {
    type Error: core::fmt::Debug;

    /// Write a string slice to the sink. Returns the number of bytes written.
    /// Default implementation forwards to the implementor's `embedded_io::Write`.
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error>;
}

/// USART2-backed debug sink.
///
/// Owns the underlying buffered UART type (not borrows it). This lets
/// the BSP return a `Uart2Sink` by value from `board_init()` and move
/// it into a task without lifetime gymnastics.
pub struct Uart2Sink<U> {
    inner: U,
}

impl<U> Uart2Sink<U> {
    /// Construct a sink from an owned buffered UART.
    pub fn new(inner: U) -> Self {
        Self { inner }
    }
}

impl<U: embedded_io::Write> DebugShellSink for Uart2Sink<U> {
    type Error = U::Error;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        // We delegate to the underlying `Write::write`. To avoid losing bytes
        // on partial writes, loop until everything is sent or an error occurs.
        let mut written = 0;
        while written < s.len() {
            match self.inner.write(&s.as_bytes()[written..]) {
                Ok(0) => break, // writer can't accept more right now
                Ok(n) => written += n,
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }
}

impl<U: embedded_io::Write> embedded_io::ErrorType for Uart2Sink<U> {
    type Error = U::Error;
}

impl<U: embedded_io::Write> embedded_io::Write for Uart2Sink<U> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.inner.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.inner.flush()
    }
}