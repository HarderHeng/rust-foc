#![no_std]
#![no_main]

mod crc;
mod uart;
mod ymodem;

use cortex_m_rt::entry;
use embedded_storage::nor_flash::NorFlash;
use foc_common::{
    APP_START_ADDRESS, APP_END_ADDRESS, FlashOtaFlag, OtaFlag, OtaState, OTA_FLAG_ADDRESS,
};

// Re-export the PAC so the linker retains the device crate's vector table.
use embassy_stm32::pac as _;

use foc_common::Stm32g4Flash;

/// Application entry point (called by bootloader when OTA flag is None or after
/// successful OTA). Sets VTOR, sets MSP, jumps to app reset vector.
#[inline(never)]
unsafe fn jump_to_app() -> ! {
    cortex_m::interrupt::disable();
    let p = cortex_m::Peripherals::steal();
    p.SCB.vtor.write(APP_START_ADDRESS);
    cortex_m::asm::bootload(APP_START_ADDRESS as *const u32)
}

/// Configure USART2 on PB3 (TX) / PB4 (RX) for 921_600 baud, 8N1.
///
/// The bootloader runs before any HAL-level USART setup, so it must
/// drive the RCC / GPIOB / USART2 registers directly. The pin map and
/// baud rate here MUST match the app's `bsp::board_init` exactly so
/// that the same terminal config survives an `ota_update` reset.
///
/// Clock path:
///   HSE 8 MHz × PLLN=85 ÷ PLLR=4 = 170 MHz sysclk.
///   USART2 sits on APB1 = 170 MHz / 4 = 42.5 MHz.
///   BRR = 42_500_000 / 921_600 ≈ 46.115 → 46 (within ±2% UART
///   tolerance for 8N1 short bursts — better than the start-byte
///   slip that mid-OTA user reset would introduce).
fn usart2_init() {
    use embassy_stm32::pac::{
        gpio::vals::{Moder, Ot, Ospeedr, Pupdr},
        GPIOB, RCC, USART2,
    };

    // 1. Enable GPIOB peripheral clock so the AF config below takes effect.
    RCC.ahb2enr().modify(|w| w.set_gpioben(true));

    // 2. PB3 (TX) / PB4 (RX) → alternate function 7 (USART2).
    let gp = GPIOB;
    gp.moder().modify(|w| {
        w.set_moder(3, Moder::ALTERNATE);
        w.set_moder(4, Moder::ALTERNATE);
    });
    gp.otyper().modify(|w| {
        w.set_ot(3, Ot::PUSH_PULL);
        w.set_ot(4, Ot::PUSH_PULL);
    });
    gp.ospeedr().modify(|w| {
        // High speed on TX (PB3) keeps the 921_600 baud rise time inside
        // the line's tolerance. PB4 is RX so medium is enough.
        w.set_ospeedr(3, Ospeedr::HIGH_SPEED);
        w.set_ospeedr(4, Ospeedr::HIGH_SPEED);
    });
    gp.pupdr().modify(|w| {
        w.set_pupdr(3, Pupdr::FLOATING);
        w.set_pupdr(4, Pupdr::FLOATING);
    });
    // AFR[0] covers pins 0..7. AF7 = 0x7 for both PB3 and PB4.
    gp.afr(0).modify(|w| {
        w.set_afr(3, 0x7);
        w.set_afr(4, 0x7);
    });

    // 3. Enable USART2 peripheral clock on APB1.
    RCC.apb1enr1().modify(|w| w.set_usart2en(true));

    // 4. BRR for 921_600 baud @ APB1 = 42.5 MHz.
    //    42_500_000 / 921_600 = 46.115 → 46 (≈0.25% slow, within ±2% UART
    //    tolerance for 8N1 short bursts).
    USART2.brr().modify(|w| w.set_brr(46));

    // 5. CR1 = UE | RE | TE.
    USART2.cr1().modify(|w| {
        w.set_ue(true);
        w.set_re(true);
        w.set_te(true);
    });

    // SAFETY: PAC register writes; the bootloader runs single-threaded with
    // IRQs disabled, so the globally-named USART2 / GPIOB / RCC registers
    // belong exclusively to this code path until `jump_to_app` runs.
}

/// Write a string to USART2 (PB3) using raw blocking TX. No defmt, no embassy driver.
///
/// USART2 on the B-G431B-ESC1 board uses pins PB3 (TX) and PB4 (RX). The
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
    // Clock configuration: HSE 8 MHz + PLLN=85 + PLLR=4 => 170 MHz sysclk.
    //
    // USART2 sits on APB1, which we divide by 4 → 42.5 MHz. The BRR for
    // 921_600 baud is therefore APB1 / baud = 42_500_000 / 921_600 ≈ 46.1
    // (the spec note in `usart2_init` writes the exact computed value).
    //
    // This MUST match the app's clock config exactly: any divergence in
    // sysclk or APB1 prescaler changes USART2's effective baud rate and
    // produces terminal garbage after a reboot / OTA entry.
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

    // USART2 was not configured by `embassy_stm32::init` (clocks only).
    // Set up the GPIO + USART2 registers here so the bootloader's banner
    // and the y-modem receiver can drive PB3 / PB4 reliably — without this
    // the post-`sys_reset()` re-entry to the bootloader (OtaState::Pending
    // branch) would output on a silent peripheral.
    usart2_init();

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

            // Erase the entire app region.
            if let Err(_e) = flash.erase(APP_START_ADDRESS, APP_END_ADDRESS) {
                uart_write_str("Erase failed; clearing flag and halting\n");
                // Clear the flag so power-cycle returns to (existing) app
                // instead of trapping us in this bootloader forever.
                // Without this the user is stuck with no recovery until
                // they SWD-clear the flag byte.
                let _ = clear_ota_flag(&mut flash);
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
                Err(crate::ymodem::YmodemError::Timeout) => {
                    // Spec: timeout clears flag so power-cycle returns to app.
                    let _ = clear_ota_flag(&mut flash);
                    uart_write_str("OTA timeout, power cycle to return to app\n");
                    loop {
                        cortex_m::asm::wfi();
                    }
                }
                Err(_) => {
                    // Other errors (CRC mismatch, abort, etc.): keep flag set,
                    // so power-cycle resumes OTA (doesn't go back to app).
                    uart_write_str("OTA error; power cycle to retry\n");
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
