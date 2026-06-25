#![no_std]
#![no_main]

mod flash;

use cortex_m_rt::entry;
use foc_common::{
    APP_START_ADDRESS, FlashOtaFlag, OtaFlag, OtaState, OTA_FLAG_ADDRESS,
};

// Re-export the PAC so the linker retains the device crate's vector table.
use embassy_stm32::pac as _;
// No defmt-rtt in bootloader.

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

#[entry]
fn main() -> ! {
    let mut flash = Stm32g4Flash::new();
    let mut flag = FlashOtaFlag::new(&mut flash, OTA_FLAG_ADDRESS);

    match flag.read() {
        OtaState::None => {
            // Normal: jump to app.
            unsafe { jump_to_app() };
        }
        OtaState::Pending => {
            // Y-modem mode — full implementation in Tasks 4-5.
            // For now, clear the flag and jump to app so the user has
            // a working fallback if Task 5 isn't complete yet.
            let _ = flag.clear();
            unsafe { jump_to_app() };
        }
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}
