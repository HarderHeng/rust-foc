#![no_std]
#![no_main]

mod bsp;
mod drivers;
mod metadata;
mod motor;
mod ota;
mod shell;
mod tasks;
mod uds;

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::usart::BufferedUart;
use panic_probe as _;

use crate::bsp::BoardHandles;

// Linker retention (required — do not remove)
use defmt_rtt as _;
use embassy_stm32::pac as _;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(bsp::clocks());
    info!("{} on {}: HAL init ok", bsp::BOARD_NAME, bsp::BOARD_MCU);

    let handles = bsp::board_init(p);
    info!("board_init done; USART2 ringbuffer ready");

    // Log firmware metadata if a valid block was injected into flash.
    if let Some(meta) = metadata::read() {
        let version_str = core::str::from_utf8(&meta.version).unwrap_or("?");
        info!("Firmware: {} (built {})", version_str, meta.build_timestamp);
        info!("  image: {} bytes, CRC32 0x{:08x}", meta.image_size, meta.image_crc32);
    } else {
        info!("No valid metadata (first boot or unprogrammed)");
    }

    // Split the BSP handles into the three task owners (motor pwm,
    // CAN, debug uart) so the partial-move borrow checker doesn't
    // try to re-use `handles` after the first `move`.
    let BoardHandles { debug_uart, motor_pwm, can } = handles;

    // Split the BufferedUart into TX / RX halves so the shell task
    // can write (via embedded-cli) and read (via
    // embedded_io_async::Read) independently.
    let buffered_uart: BufferedUart<'static> = debug_uart.into_inner();
    let (tx, rx) = buffered_uart.split();

    // Spawn the heartbeat task (defmt-only, no USART2).
    spawner.spawn(tasks::heartbeat().unwrap());

    // Spawn the shell task — takes ownership of TX and RX halves.
    spawner.spawn(tasks::shell_task(tx, rx).unwrap());

    // Spawn the motor task — 10 kHz Ticker that drives TIM1 from the
    // shared `OPEN_LOOP_CMD` cell written by the shell.
    spawner.spawn(tasks::motor_task(motor_pwm).unwrap());

    // The CANopen task owns the only `&mut Can` for the lifetime of
    // the executor. `cortex_m::singleton!` is the standard way to
    // turn a `T` into a `&'static mut T` in single-threaded no_std.
    let can: &'static mut embassy_stm32::can::Can<'static> =
        cortex_m::singleton!(: embassy_stm32::can::Can<'static> = can)
            .expect("Can singleton taken twice");

    // Spawn the CANopen task — NMT state machine + 1 Hz heartbeat
    // over FDCAN1. This is the OTA-side protocol stack (Phase 2
    // adds SDO, Phase 3 adds UDS, Phase 4 adds OTA transfer).
    spawner.spawn(tasks::canopen_task(can).unwrap());

    // Main task: park in WFI forever. Real work happens in spawned tasks.
    loop {
        cortex_m::asm::wfi();
    }
}
