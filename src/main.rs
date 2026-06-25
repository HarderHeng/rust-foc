#![no_std]
#![no_main]

mod bsp;
mod drivers;

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_probe as _;

// Linker retention (required — do not remove)
use defmt_rtt as _;
use embassy_stm32::pac as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("{} on {}: HAL init ok", bsp::BOARD_NAME, bsp::BOARD_MCU);

    let _handles = bsp::board_init(p);
    info!("board_init done");

    loop {
        Timer::after_millis(500).await;
    }
}
