//! Debug serial output driver for USART2.
//!
//! `Uart2Sink` wraps a `BufferedUart` and implements both:
//! - [`DebugShellSink`]: high-level `&str` writing for app tasks
//! - `embedded_io::Write`: standard interface for third-party libs (e.g. `embedded-cli`)
//!
//! All blocking semantics: this sink writes synchronously to a TX ringbuffer.
//! Long blocking is only possible if the ringbuffer is full.

// Task 4 deviation: this module is part of the task tree but is not yet wired
// into the composition root. Suppress dead-code warnings on the public API
// surface that task modules (5, 6, 7) will depend on.
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
/// `'d` is the lifetime of the USART peripheral borrowed from `embassy-stm32`.
/// The type parameter `U` is the underlying buffered UART type (left as
/// `?Sized` for now; in Task 5 we tighten it).
pub struct Uart2Sink<'d, U: ?Sized> {
    inner: &'d mut U,
}

impl<'d, U: ?Sized> Uart2Sink<'d, U> {
    /// Construct a sink from a borrowed buffered UART.
    /// Will be wired to actual `BufferedUart` in Task 5.
    pub fn new(inner: &'d mut U) -> Self {
        Self { inner }
    }
}

impl<'d, U: embedded_io::Write + ?Sized> DebugShellSink for Uart2Sink<'d, U> {
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

impl<'d, U: embedded_io::Write + ?Sized> embedded_io::ErrorType for Uart2Sink<'d, U> {
    type Error = U::Error;
}

impl<'d, U: embedded_io::Write + ?Sized> embedded_io::Write for Uart2Sink<'d, U> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.inner.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.inner.flush()
    }
}
