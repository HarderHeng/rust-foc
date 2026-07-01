//! `ota_update` command: set OTA flag → reboot into bootloader.

use cortex_m::peripheral::SCB;

use foc_common::{FlashOtaFlag, OtaFlag, OTA_FLAG_ADDRESS};

use foc_common::Stm32g4Flash;

/// Execute OTA update: write flag, print message, reset.
pub fn run_ota_update<W, E>(cli: &mut embedded_cli::cli::CliHandle<'_, W, E>)
where
    W: embedded_cli::__private::io::Write<Error = E>,
    E: embedded_cli::__private::io::Error,
{
    let mut flash = Stm32g4Flash::new();
    let mut flag = FlashOtaFlag::new(&mut flash, OTA_FLAG_ADDRESS);
    if let Err(_e) = flag.set_pending() {
        let _ = cli.writer().write_str("ota_update: flag set failed, aborting\r\n");
        defmt::error!("ota_update: set_pending failed");
        return;
    }

    let _ = cli.writer().write_str("Rebooting to OTA bootloader, send y-modem...\r\n");
    cortex_m::asm::delay(170_000_000 / 20); // ~50 ms
    SCB::sys_reset();
}
