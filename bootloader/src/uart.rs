//! Synchronous USART2 read/write for the bootloader.
//!
//! No async, no ringbuffer. Each byte is polled. The bootloader has no other
//! concurrent work, so this is safe and sufficient.
//!
//! Clocks are configured by `embassy_stm32::init()` before these functions
//! are called (170 MHz sysclk, USART2 at 921600 baud).

use core::sync::atomic::AtomicU8;

// ── helpers ────────────────────────────────────────────────────────────

/// Busy-wait delay: spins for approximately `ms` milliseconds at 170 MHz.
///
/// Uses `cortex_m::asm::delay(N)` where N ≈ 170_000 cycles per ms.
/// On Cortex-M4F the delay loop is 1 cycle per iteration.
#[inline(never)]
fn delay_ms(ms: u32) {
    cortex_m::asm::delay(ms * (170_000_000 / 1000));
}

// ── public API ─────────────────────────────────────────────────────────

/// Non-blocking read. Returns the byte if the RX register is non-empty.
#[inline]
pub fn uart_try_read() -> Option<u8> {
    let usart = embassy_stm32::pac::USART2;
    if usart.isr().read().rxne() {
        Some(usart.rdr().read().dr() as u8)
    } else {
        None
    }
}

/// Block until a byte is read with no timeout.
#[inline(never)]
#[allow(dead_code)]
pub fn uart_read_byte() -> u8 {
    loop {
        if let Some(b) = uart_try_read() {
            return b;
        }
    }
}

/// Read a byte with a timeout.
///
/// Returns `Ok(byte)` if data arrives within `timeout_ms` milliseconds, or
/// `Err(())` if the timeout expires.
#[inline(never)]
pub fn uart_read_byte_timeout(timeout_ms: u32) -> Result<u8, ()> {
    for _ in 0..timeout_ms {
        if let Some(b) = uart_try_read() {
            return Ok(b);
        }
        delay_ms(1);
    }
    Err(())
}

/// Write a byte, blocking until the TX register is empty.
#[inline]
pub fn uart_write_byte(b: u8) {
    let usart = embassy_stm32::pac::USART2;
    while !usart.isr().read().txe() {}
    usart.tdr().write(|w| w.set_dr(b as u16));
}

/// Write a string of bytes, blocking until all are sent.
#[inline(never)]
pub fn uart_write_str(s: &str) {
    let usart = embassy_stm32::pac::USART2;
    for &b in s.as_bytes() {
        while !usart.isr().read().txe() {}
        usart.tdr().write(|w| w.set_dr(b as u16));
    }
    // Wait for transmission complete (last byte may still be shifting out).
    while !usart.isr().read().tc() {}
}

// Suppress unused warning for AtomicU8 (kept as a placeholder for future
// buffering or status tracking).
#[allow(dead_code)]
static _SUPPRESS: AtomicU8 = AtomicU8::new(0);
