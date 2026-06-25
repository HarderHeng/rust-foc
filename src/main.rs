#![no_std]
#![no_main]

mod bsp;
mod commands;
mod drivers;
mod tasks;

use defmt::info;
use embassy_executor::Spawner;
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

    // Spawn the heartbeat task. In embassy-executor 0.10, the
    // `#[embassy_executor::task]` macro returns a `Result<SpawnToken,
    // SpawnError>` from the function call (the inner `.unwrap()`), and
    // `Spawner::spawn` consumes the token returning `()`. (Tried with
    // an outer `.unwrap()` on the spawner call too — fails to compile
    // in 0.10 because `Spawner::spawn` returns `()`, not `Result`.)
    spawner.spawn(tasks::heartbeat(handles.debug_uart).unwrap());

    // Main task: park in WFI forever. Real work happens in spawned tasks.
    loop {
        cortex_m::asm::wfi();
    }
}
