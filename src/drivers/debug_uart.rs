//! Debug serial output driver for USART2.
//!
//! `Uart2Sink` wraps a `BufferedUart` and implements `embedded_io::Write`
//! (v0.7), the version embassy-stm32 uses natively.

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

