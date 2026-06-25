//! Shell command implementations for embedded-cli.
//!
//! Defines a `#[derive(Command)]` enum `ShellCommand` with 5 unit variants:
//!   - `help`   — list available commands (built-in from embedded-cli via `-h`)
//!   - `version`— firmware version string
//!   - `info`   — chip + flash usage info
//!   - `reboot` — reset the MCU
//!   - `ota_update` — set OTA flag and reboot into bootloader

use cortex_m::peripheral::SCB;
use embedded_cli::cli::CliHandle;

use crate::commands::ota::run_ota_update;

// ---------------------------------------------------------------------------
// ShellCommand enum — one variant per shell command
// ---------------------------------------------------------------------------

/// The complete set of shell commands. Each variant maps to a CLI keyword.
///
/// The `#[command(name = "...")]` attributes supply the command name; the
/// first paragraph of the doc comment is the short help text shown by
/// `help` / `-h`.
#[derive(embedded_cli::Command)]
pub enum ShellCommand {
    /// List available commands
    #[command(name = "help")]
    Help,

    /// Show firmware version
    #[command(name = "version")]
    Version,

    /// Show chip + flash usage info
    #[command(name = "info")]
    Info,

    /// Reset the MCU
    #[command(name = "reboot")]
    Reboot,

    /// Trigger OTA firmware update
    #[command(name = "ota_update")]
    OtaUpdate,
}

// ---------------------------------------------------------------------------
// Processor closure: the single dispatch function for all commands
// ---------------------------------------------------------------------------

/// Build the closure that `ShellCommand::processor(f)` will wrap.
///
/// The closure receives a mutable `CliHandle` (for writing output) and the
/// parsed `ShellCommand` variant. It must return `Result<(), E>` where `E`
/// is the writer's error type (the `embedded-cli` derive macro handles the
/// conversion to `ProcessError` internally).
pub fn make_processor<W, E>() -> impl embedded_cli::service::CommandProcessor<W, E>
where
    W: embedded_cli::__private::io::Write<Error = E>,
    E: embedded_cli::__private::io::Error,
{
    ShellCommand::processor(|cli: &mut CliHandle<'_, W, E>, cmd: ShellCommand| {
        match cmd {
            ShellCommand::Help => {
                // The built-in help is triggered by `-h`; providing an explicit
                // message in case the user types `help` and expects a response.
                let _ = cli.writer().write_str("Type <command> -h for help on a specific command\r\n");
            }
            ShellCommand::Version => {
                let _ = cli.writer().write_str(env!("CARGO_PKG_VERSION"));
                let _ = cli.writer().write_str("\r\n");
            }
            ShellCommand::Info => {
                let _ = cli.writer().write_str("STM32G431CBU6\r\n");
                let _ = cli.writer().write_str("  flash: 128 KB\r\n");
                let _ = cli.writer().write_str("  sram:  32 KB\r\n");
            }
            ShellCommand::Reboot => {
                let _ = cli.writer().write_str("Rebooting...\r\n");
                // Brief delay so the message reaches the terminal before reset.
                cortex_m::asm::delay(170_000_000 / 20); // ~50 ms at 170 MHz
                SCB::sys_reset();
            }
            ShellCommand::OtaUpdate => {
                run_ota_update(cli);
            }
        }
        Ok(())
    })
}
