#![no_std]
#![no_main]

mod bsp;
mod drivers;

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_probe as _;

use crate::drivers::debug_uart::DebugShellSink;

// Linker retention (required — do not remove)
use defmt_rtt as _;
use embassy_stm32::pac as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("{} on {}: HAL init ok", bsp::BOARD_NAME, bsp::BOARD_MCU);

    let handles = bsp::board_init(p);
    info!("board_init done; USART2 ringbuffer ready");

    // For now, the heartbeat task in main itself writes to it.
    let mut uart = handles.debug_uart;
    uart.write_str("hello from B-G431B-ESC1\n").unwrap();
    info!("wrote first USART2 line");

    loop {
        Timer::after_millis(500).await;
    }
}