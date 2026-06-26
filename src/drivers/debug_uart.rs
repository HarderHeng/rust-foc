//! Debug serial output driver for USART2.
//!
//! `Uart2Sink` wraps a `BufferedUart` and implements:
//! - [`DebugShellSink`]: high-level `&str` writing for app tasks
//! - `embedded_io::Write` (v0.7): standard interface for third-party libs
//! - `embedded-io-06` `Write` + `ErrorType`: adapter for `embedded-cli` 0.2.1
//!   (which depends on `embedded-io` 0.6.x internally)
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

    /// Access the inner writer (v0.7 `embedded_io::Write`).
    pub fn inner(&mut self) -> &mut U {
        &mut self.inner
    }

    /// Consume self and recover the inner value.
    pub fn into_inner(self) -> U {
        self.inner
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

// ── embedded-io v0.6 adapter for embedded-cli 0.2.1 ──

use core::fmt;
use embassy_stm32::usart::{BufferedUart, Error as UsartError};
use embedded_io::Write as _;
use embedded_io_06 as eio06;

/// Wraps `embassy_stm32::usart::Error` into embedded-io v0.6's error trait.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UsartError06(UsartError);

impl UsartError06 {
    const fn kind(&self) -> eio06::ErrorKind {
        eio06::ErrorKind::Other
    }
}

impl eio06::Error for UsartError06 {
    fn kind(&self) -> eio06::ErrorKind {
        eio06::ErrorKind::Other
    }
}

impl From<UsartError> for UsartError06 {
    fn from(e: UsartError) -> Self {
        UsartError06(e)
    }
}

impl fmt::Display for UsartError06 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

/// Wraps a `Uart2Sink<BufferedUart<'static>>` and implements `embedded-io` 0.6
/// traits for use with `embedded-cli` 0.2.1.
pub struct Uart2Sink06 {
    inner: Uart2Sink<BufferedUart<'static>>,
}

impl Uart2Sink06 {
    /// Wrap a concrete debug sink.
    pub fn new(inner: Uart2Sink<BufferedUart<'static>>) -> Self {
        Self { inner }
    }
}

impl eio06::ErrorType for Uart2Sink06 {
    type Error = UsartError06;
}

impl eio06::Write for Uart2Sink06 {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.inner.inner().write(buf).map_err(UsartError06)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.inner.inner().flush().map_err(UsartError06)
    }
}
