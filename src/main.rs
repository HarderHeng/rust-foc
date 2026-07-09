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

use crate::bsp::BoardHandles;

// Linker retention (required — do not remove)
use defmt_rtt as _;
use embassy_stm32::pac as _;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::asm::udf()
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(bsp::clocks());
    info!("{} on {}: HAL init ok", bsp::BOARD_NAME, bsp::BOARD_MCU);

    let handles = bsp::board_init(p);
    info!("board_init done; USART2 ringbuffer ready");

    // Start IWDG so the watchdog doesn't trip before tasks are up.
    feed_watchdog();

    // Log firmware identity.
    info!("Firmware: {} (git {})", env!("FOC_VERSION"), env!("FOC_GIT_SHA"));

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

    static UDS_TRANSPORT: drivers::can::uds_bridge::DefaultUdsTransport =
        drivers::can::uds_bridge::DefaultUdsTransport;

    // Spawn the CANopen task — NMT state machine + 1 Hz heartbeat
    // over FDCAN1. This is the OTA-side protocol stack (Phase 2
    // adds SDO, Phase 3 adds UDS, Phase 4 adds OTA transfer).
    spawner.spawn(tasks::canopen_task(can, &UDS_TRANSPORT).unwrap());

    // Main task: park in WFI forever. Real work happens in spawned tasks.
    loop {
        cortex_m::asm::wfi();
    }
}

fn feed_watchdog() {
    unsafe {
        const KR: u32 = 0x4000_3000;
        const PR: u32 = 0x4000_3004;
        const RLR: u32 = 0x4000_3008;
        core::ptr::write_volatile(KR as *mut u32, 0x5555);
        core::ptr::write_volatile(PR as *mut u32, 6);       // /256
        core::ptr::write_volatile(RLR as *mut u32, 0xFFF);  // ~125 ms
        core::ptr::write_volatile(KR as *mut u32, 0xCCCC);  // start
        core::ptr::write_volatile(KR as *mut u32, 0xAAAA);  // refresh
    }
}
