#![no_std]
#![no_main]

mod crc;
mod flash;
mod uart;
mod ymodem;

use cortex_m_rt::entry;
use embedded_storage::nor_flash::NorFlash;
use foc_common::{
    APP_START_ADDRESS, APP_END_ADDRESS, FlashOtaFlag, OtaFlag, OtaState, OTA_FLAG_ADDRESS,
};

// Re-export the PAC so the linker retains the device crate's vector table.
use embassy_stm32::pac as _;

use crate::flash::Stm32g4Flash;

/// Application entry point (called by bootloader when OTA flag is None or after
/// successful OTA). Sets VTOR, sets MSP, jumps to app reset vector.
#[inline(never)]
unsafe fn jump_to_app() -> ! {
    cortex_m::interrupt::disable();
    let p = cortex_m::Peripherals::steal();
    p.SCB.vtor.write(APP_START_ADDRESS);
    cortex_m::asm::bootload(APP_START_ADDRESS as *const u32)
}

/// Write a string to USART2 (PB3) using raw blocking TX. No defmt, no embassy driver.
///
/// USART2 on the B-G431B-ESC1 board uses pins PB3 (TX) and PB10 (RX). The
/// peripheral must already be configured (by the bootloader, or left as the app
/// configured it before reset).
///
/// The bootloader sets up clocks and USART2 via `embassy_stm32::init()` before
/// calling this function, so the registers are accessible at their PAC addresses.
fn uart_write_str(s: &str) {
    let usart = embassy_stm32::pac::USART2;
    for &b in s.as_bytes() {
        while !usart.isr().read().txe() {}
        // SAFETY: Writing a byte to the TDR is safe; we own the peripheral and
        // have exclusive access (single-threaded bootloader).
        usart.tdr().write(|w| w.set_dr(b as u16));
    }
    // Wait for transmission complete so the last byte makes it out before
    // any reset/sleep/etc.
    while !usart.isr().read().tc() {}
}

/// Read the OTA flag state using a temporary FlashOtaFlag borrowing flash.
fn read_ota_flag(flash: &mut Stm32g4Flash) -> OtaState {
    FlashOtaFlag::new(flash, OTA_FLAG_ADDRESS).read()
}

/// Clear the OTA flag state using a temporary FlashOtaFlag borrowing flash.
fn clear_ota_flag(flash: &mut Stm32g4Flash) -> Result<(), ()> {
    FlashOtaFlag::new(flash, OTA_FLAG_ADDRESS)
        .clear()
        .map_err(|_| ())
}

#[entry]
fn main() -> ! {
    // ------------------------------------------------------------------
    // Clock configuration: HSE 8 MHz + PLLN=85 + PLLR=4 => 170 MHz sysclk
    //
    // This MUST match the app's clock config so that USART2 uses the same
    // baud rate divisor (170 MHz / 921600 = ~184.6). If the bootloader ran
    // on HSI (16 MHz or 64 MHz), the baud rate would be computed differently
    // and the terminal would receive garbage.
    // ------------------------------------------------------------------
    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = false;
    config.rcc.hse = Some(embassy_stm32::rcc::Hse {
        freq: embassy_stm32::time::Hertz::mhz(8),
        mode: embassy_stm32::rcc::HseMode::Oscillator,
    });
    config.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_R;
    config.rcc.pll = Some(embassy_stm32::rcc::Pll {
        source: embassy_stm32::rcc::PllSource::HSE,
        prediv: embassy_stm32::rcc::PllPreDiv::DIV1,
        mul: embassy_stm32::rcc::PllMul::MUL85,
        divp: None,
        divq: None,
        divr: Some(embassy_stm32::rcc::PllRDiv::DIV4),
    });
    config.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::DIV1;
    config.rcc.apb1_pre = embassy_stm32::rcc::APBPrescaler::DIV4;
    config.rcc.apb2_pre = embassy_stm32::rcc::APBPrescaler::DIV1;
    config.rcc.boost = true;

    let _p = embassy_stm32::init(config);

    let mut flash = Stm32g4Flash::new();

    // Read flag via a temporary borrow, then the borrow is released before
    // we use flash for erase/write below.
    match read_ota_flag(&mut flash) {
        OtaState::None => {
            // Normal boot: jump to app.
            unsafe { jump_to_app() }
        }

        OtaState::Pending => {
            uart_write_str("\n=== B-G431B-ESC1 OTA Bootloader ===\n");
            uart_write_str("Send y-modem (CRC mode) now... (timeout 30s)\n");

            // Erase the entire app region (56 pages of 2 KB each).
            if let Err(_e) = flash.erase(APP_START_ADDRESS, APP_END_ADDRESS) {
                uart_write_str("Erase failed; aborting\n");
                // Stay in bootloader; user can power-cycle to retry.
                loop {
                    cortex_m::asm::wfi();
                }
            }

            // Call the y-modem receive (stub in Task 4, full in Task 5).
            let result = crate::ymodem::receive_image(&mut flash);

            match result {
                Ok(()) => {
                    // y-modem complete — clear the flag and reset.
                    if clear_ota_flag(&mut flash).is_err() {
                        uart_write_str("Flag clear failed; rebooting anyway\n");
                    }
                    uart_write_str("OTA OK, rebooting...\n");
                    // Brief delay so the message reaches the terminal.
                    cortex_m::asm::delay(170_000_000 / 100); // ~10 ms at 170 MHz
                    cortex_m::peripheral::SCB::sys_reset();
                }
                Err(_e) => {
                    uart_write_str("OTA error; power-cycle to retry\n");
                    // Stay in bootloader per spec.
                    loop {
                        cortex_m::asm::wfi();
                    }
                }
            }
        }
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}
