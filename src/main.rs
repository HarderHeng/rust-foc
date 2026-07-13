#![no_std]
#![no_main]

mod bsp;
mod drivers;
mod key_store;
mod metadata;
mod motor;
mod ota;
mod shell;
mod tasks;
mod uds;
mod wdog;

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

    let BoardHandles { debug_uart, motor_pwm, can, iwdg } = handles;

    // C2: start the IWDG. The heartbeat task refreshes it every
    // 500 ms. Previous code started it but never refreshed — the
    // device reset every ~32 s.
    wdog::init(iwdg);

    // C5: load (or generate) the per-device SAL keys and install
    // them into UDS_CONFIG. The previous code shipped plaintext
    // key material in the ELF, which meant anyone with the source
    // could complete the 0x27 handshake offline. The new flow
    // reads the keys from a dedicated flash region (or generates
    // them on first boot) so each device has unique material.
    let live_keys = key_store::init();
    critical_section::with(|cs| {
        crate::uds::static_config::UDS_CONFIG
            .borrow_ref_mut(cs)
            .key_masks
            .set(live_keys);
    });

    // Log firmware identity.
    info!("Firmware: {} (git {})", env!("FOC_VERSION"), env!("FOC_GIT_SHA"));

    // Confirm boot to embassy-boot: after a DFU→ACTIVE swap, the bootloader
    // sets REVERT_MAGIC (0xC0) in the STATE partition. We write BOOT_MAGIC
    // (0xD0) to confirm the new firmware works. Without this, the next reset
    // would swap back to the old firmware.
    mark_booted();

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
    // C2: replaced by `wdog::init()` + `wdog::pet()` from the
    // heartbeat task. The previous implementation started the IWDG
    // with a ~32 s timeout and never refreshed it, so the device
    // would reset itself every ~32 s. The new flow is:
    //   1. main() calls `wdog::init()` which calls
    //      `IndependentWatchdog::new(5_000_000).unleash()`.
    //   2. heartbeat() calls `wdog::pet()` every 500 ms.
    //   3. If heartbeat stops for 5 s, the IWDG fires and the
    //      device resets cleanly.
    wdog::pet();
}

fn mark_booted() {
    // C1: embassy-boot's `read_state` checks that **every** byte in
    // the 8-byte state word equals the magic (BOOT_MAGIC = 0xD0).
    // The previous `0x0000_0000_0001_00D0_u64` only set byte 0 to
    // 0xD0 — the rest of the word didn't match, so the bootloader
    // couldn't tell the boot was confirmed and the new image would
    // be reverted on the next power-cycle. Fill all 8 bytes with
    // BOOT_MAGIC instead.
    //
    // Side effect: with the magic written correctly, the bootloader
    // stops re-entering the swap loop on the next boot and the new
    // ACTIVE firmware sticks.
    const STATE_ADDR: u32 = 0x0800_6000;
    unsafe {
        let page = STATE_ADDR & !2047;
        let _ = crate::drivers::flash::erase_region(page, page + 2048);
        let _ = crate::drivers::flash::write_u64(STATE_ADDR, page, page + 2048, 0xD0D0_D0D0_D0D0_D0D0_u64);
    }
}
