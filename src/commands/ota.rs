//! `ota_update` command: set OTA_FLAG, write user-visible message to shell, reset.

use cortex_m::peripheral::SCB;
use defmt::info;

use embedded_cli::cli::CliHandle;
use embedded_cli::command::RawCommand;

use foc_common::OtaFlag;
use foc_common::{FlashOtaFlag, OTA_FLAG_ADDRESS};

use crate::drivers::flash::Stm32g4Flash;

/// The actual processor body for the `ota_update` command.
///
/// Implements `CommandProcessor` directly (rather than via the blanket
/// closure impl) to avoid HRTB lifetime-inference issues with
/// `ProcessError<'a, E>`.
///
/// NOTE: currently unused from outside this module — `run_ota_update()`
/// below provides a `RawCommand`-free alternative for the shell command.
#[allow(dead_code)]
pub struct OtaUpdateProcessor;

impl<W, E> embedded_cli::service::CommandProcessor<W, E> for OtaUpdateProcessor
where
    W: embedded_cli::__private::io::Write<Error = E>,
    E: embedded_cli::__private::io::Error,
{
    fn process<'a>(
        &mut self,
        cli: &mut CliHandle<'_, W, E>,
        _cmd: RawCommand<'a>,
    ) -> Result<(), embedded_cli::service::ProcessError<'a, E>> {
        info!("ota_update: setting OTA flag");

        // 1. Write the OTA flag to flash.
        let mut flash = Stm32g4Flash::new();
        let mut flag = FlashOtaFlag::new(&mut flash, OTA_FLAG_ADDRESS);
        if let Err(e) = flag.set_pending() {
            let _ = cli.writer().write_str("ota_update: flag set failed, aborting\r\n");
            defmt::error!("ota_update: FlashOtaFlag::set_pending failed: {:?}", e);
            return Ok(());
        }
        info!("ota_update: flag written OK");

        // 2. Tell the user what's about to happen.
        let _ = cli.writer().write_str("Rebooting to OTA bootloader, send y-modem now...\r\n");

        // 3. Brief busy-wait so the message reaches the terminal before reset.
        cortex_m::asm::delay(170_000_000 / 20); // ~50 ms at 170 MHz

        // 4. System reset — this never returns.
        info!("ota_update: sys_reset");
        SCB::sys_reset();
    }
}

/// Execute the OTA update workflow: set flag, write message, reset.
///
/// This free function extracts the side-effect logic from `OtaUpdateProcessor` so
/// it can be called directly from other command implementations without requiring
/// a `RawCommand` argument.
pub fn run_ota_update<W, E>(cli: &mut CliHandle<'_, W, E>)
where
    W: embedded_cli::__private::io::Write<Error = E>,
    E: embedded_cli::__private::io::Error,
{
    info!("ota_update: setting OTA flag");

    let mut flash = Stm32g4Flash::new();
    let mut flag = FlashOtaFlag::new(&mut flash, OTA_FLAG_ADDRESS);
    if let Err(e) = flag.set_pending() {
        let _ = cli.writer().write_str("ota_update: flag set failed, aborting\r\n");
        defmt::error!("ota_update: FlashOtaFlag::set_pending failed: {:?}", e);
        return;
    }
    info!("ota_update: flag written OK");

    let _ = cli.writer().write_str("Rebooting to OTA bootloader, send y-modem now...\r\n");
    cortex_m::asm::delay(170_000_000 / 20); // ~50 ms at 170 MHz

    info!("ota_update: sys_reset");
    SCB::sys_reset();
}

/// Concrete OTA update command entry point.
#[allow(dead_code)]
pub struct OtaUpdateCommand;

impl OtaUpdateCommand {
    /// Return a `CommandProcessor` for the `ota_update` command.
    #[allow(dead_code)]
    pub fn processor<W, E>() -> OtaUpdateProcessor
    where
        W: embedded_cli::__private::io::Write<Error = E>,
        E: embedded_cli::__private::io::Error,
    {
        OtaUpdateProcessor
    }
}
