#![no_std]
#![no_main]

mod bsp;
mod commands;
mod drivers;
mod tasks;

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::usart::BufferedUart;
use panic_probe as _;

// Linker retention (required — do not remove)
use defmt_rtt as _;
use embassy_stm32::pac as _;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(bsp::clocks());
    info!("{} on {}: HAL init ok", bsp::BOARD_NAME, bsp::BOARD_MCU);

    let handles = bsp::board_init(p);
    info!("board_init done; USART2 ringbuffer ready");

    // Split the BufferedUart into TX / RX halves so the shell task can
    // write (via embedded-cli) and read (via embedded_io_async::Read)
    // independently.
    let buffered_uart: BufferedUart<'static> = handles.debug_uart.into_inner();
    let (tx, rx) = buffered_uart.split();

    // Spawn the heartbeat task (defmt-only, no USART2).
    spawner.spawn(tasks::heartbeat().unwrap());

    // Spawn the shell task — takes ownership of TX and RX halves.
    spawner.spawn(tasks::shell_task(tx, rx).unwrap());

    // Main task: park in WFI forever. Real work happens in spawned tasks.
    loop {
        cortex_m::asm::wfi();
    }
}
