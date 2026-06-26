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

use embassy_stm32::pac as _;
use crate::flash::Stm32g4Flash;

#[inline(never)]
unsafe fn jump_to_app() -> ! {
    cortex_m::interrupt::disable();
    let p = cortex_m::Peripherals::steal();
    p.SCB.vtor.write(APP_START_ADDRESS);
    cortex_m::asm::bootload(APP_START_ADDRESS as *const u32)
}

fn uart_write_str(s: &str) {
    let usart = embassy_stm32::pac::USART2;
    for &b in s.as_bytes() {
        while !usart.isr().read().txe() {}
        usart.tdr().write(|w| w.set_dr(b as u16));
    }
    while !usart.isr().read().tc() {}
}

fn read_ota_flag(flash: &mut Stm32g4Flash) -> OtaState {
    FlashOtaFlag::new(flash, OTA_FLAG_ADDRESS).read()
}

fn clear_ota_flag(flash: &mut Stm32g4Flash) -> Result<(), ()> {
    FlashOtaFlag::new(flash, OTA_FLAG_ADDRESS).clear().map_err(|_| ())
}

#[entry]
fn main() -> ! {
    // Clock: HSE 8 MHz → PLL ×85 /4 → 170 MHz sysclk.
    // MUST match the app's config — USART2 baud divisor (921600) depends on it.
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

    match read_ota_flag(&mut flash) {
        OtaState::None => unsafe { jump_to_app() },

        OtaState::Pending => {
            uart_write_str("\n=== B-G431B-ESC1 OTA Bootloader ===\n");
            uart_write_str("Send y-modem (CRC mode) now... (timeout 30s)\n");

            if let Err(_e) = flash.erase(APP_START_ADDRESS, APP_END_ADDRESS) {
                uart_write_str("Erase failed\n");
                loop { cortex_m::asm::wfi(); }
            }

            match crate::ymodem::receive_image(&mut flash) {
                Ok(()) => {
                    if clear_ota_flag(&mut flash).is_err() {
                        uart_write_str("Flag clear failed, rebooting anyway\n");
                    }
                    uart_write_str("OTA OK, rebooting...\n");
                    cortex_m::asm::delay(170_000_000 / 100);
                    cortex_m::peripheral::SCB::sys_reset();
                }
                Err(crate::ymodem::YmodemError::Timeout) => {
                    let _ = clear_ota_flag(&mut flash);
                    uart_write_str("OTA timeout, power cycle to return to app\n");
                    loop { cortex_m::asm::wfi(); }
                }
                Err(_) => {
                    uart_write_str("OTA error; power cycle to retry\n");
                    loop { cortex_m::asm::wfi(); }
                }
            }
        }
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop { cortex_m::asm::wfi(); }
}
