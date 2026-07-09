//! Debug serial output driver for USART2.
//!
//! `Uart2Sink` wraps a `BufferedUart` and implements:
//! - `embedded_io::Write` (v0.7): standard interface for third-party libs
//! - `embedded-io-06` `Error` + `From<UsartError>` adapter so errors
//!   from the v0.7 write can be returned as v0.6 errors (the version
//!   `embedded-cli` 0.2.1 expects)
//!
//! All blocking semantics: this sink writes synchronously to a TX ringbuffer.
//! Long blocking is only possible if the ringbuffer is full.

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

    /// Consume self and recover the inner value.
    pub fn into_inner(self) -> U {
        self.inner
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

// ── embedded-io v0.6 error adapter for embedded-cli 0.2.1 ──
//
// embedded-cli 0.2.1 takes an embedded-io v0.6 writer.  We use
// embassy-stm32's v0.7-backed BufferedUart but expose only the error
// type to cross the boundary; the actual writer is the local
// `TxWriter06` adapter in tasks/shell.rs.

use embassy_stm32::usart::Error as UsartError;
use embedded_io_06 as eio06;

/// Wraps `embassy_stm32::usart::Error` into embedded-io v0.6's error trait.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UsartError06(UsartError);

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
