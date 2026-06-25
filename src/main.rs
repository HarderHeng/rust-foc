#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::init;
use embassy_time::Timer;
use panic_probe as _;

// Task 2 deviation from brief: the brief omitted the defmt-rtt and PAC retention
// imports. Both crates are pulled into Cargo.toml but `use ... as _;` is required
// for the linker to keep the symbols `_defmt_write` / `_defmt_acquire` /
// `_defmt_release` (from defmt-rtt) and the interrupt vector table (from PAC).
use defmt_rtt as _;
use embassy_stm32::pac as _;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Task 2 deviation from brief: the brief did not call embassy_stm32::init().
    // embassy-stm32's time driver (which provides _embassy_time_now and
    // _embassy_time_schedule_wake) is only linked when something references
    // `crate::time_driver`. `init()` is the public entry point that does this.
    // Without this call, embassy_time::Timer cannot resolve its driver and the
    // link fails.
    let _p = init(Default::default());

    info!("B-G431B-ESC1 init ok");

    // Task 2 deviation from brief: embassy-executor 0.10 changed the task macro
    // so `#[embassy_executor::task]` returns `Result<SpawnToken, SpawnError>`
    // (not a `SpawnToken` directly). `Spawner::spawn` consumes the token by
    // value and returns `()`. So the correct call is `spawn(... .unwrap())`
    // (one unwrap, on the inner Result).
    spawner.spawn(heartbeat().unwrap());

    loop {
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn heartbeat() {
    loop {
        info!("heartbeat");
        Timer::after_millis(500).await;
    }
}