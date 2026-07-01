//! Synchronous USART2 read/write for the bootloader.
//! No async, no ringbuffer — bootloader has no concurrent work.

#[inline(never)]
fn delay_ms(ms: u32) {
    cortex_m::asm::delay(ms * (170_000_000 / 1000));
}

#[inline]
pub fn uart_try_read() -> Option<u8> {
    let usart = embassy_stm32::pac::USART2;
    if usart.isr().read().rxne() {
        Some(usart.rdr().read().dr() as u8)
    } else {
        None
    }
}

#[inline(never)]
pub fn uart_read_byte_timeout(timeout_ms: u32) -> Result<u8, ()> {
    for _ in 0..timeout_ms {
        if let Some(b) = uart_try_read() { return Ok(b); }
        delay_ms(1);
    }
    Err(())
}

#[inline]
pub fn uart_write_byte(b: u8) {
    let usart = embassy_stm32::pac::USART2;
    while !usart.isr().read().txe() {}
    usart.tdr().write(|w| w.set_dr(b as u16));
}

#[inline(never)]
pub fn uart_write_str(s: &str) {
    let usart = embassy_stm32::pac::USART2;
    for &b in s.as_bytes() {
        while !usart.isr().read().txe() {}
        usart.tdr().write(|w| w.set_dr(b as u16));
    }
    while !usart.isr().read().tc() {}
}
