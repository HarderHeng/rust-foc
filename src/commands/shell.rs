//! Shell command implementations for embedded-cli.
//!
//! Defines a `#[derive(Command)]` enum `ShellCommand` with six variants:
//!   - `help`        — list available commands
//!   - `version`     — firmware version string
//!   - `info`        — chip + flash usage info
//!   - `reboot`      — reset the MCU
//!   - `spin <f> <v>`— start open-loop rotating voltage vector
//!   - `stop`        — soft-stop the open-loop spin
//!
//! (The previous `ota_update` command was removed when the y-modem
//! bootloader was deleted; OTA is now driven over FDCAN1 by the
//! CANopen + UDS protocol stack — see
//! `docs/superpowers/specs/2026-07-02-can-ota-uds-design.md`.)

use cortex_m::peripheral::SCB;
use embedded_cli::cli::CliHandle;

use crate::bsp::{BOARD_MCU, BOARD_NAME, FLASH_SIZE_KB, SRAM_SIZE_KB};
use crate::control::cmd::{OpenLoopCmd, OPEN_LOOP_CMD};
use crate::control::open_loop::MAX_OPENLOOP_V;

// ---------------------------------------------------------------------------
// ShellCommand enum — one variant per shell command
// ---------------------------------------------------------------------------

/// The complete set of shell commands. Each variant maps to a CLI keyword.
///
/// The `#[command(name = "...")]` attributes supply the command name; the
/// first paragraph of the doc comment is the short help text shown by
/// `help` / `-h`. Struct-variant fields with no `#[arg(...)]` attribute
/// become **positional** args — `spin 10 2.0` parses into
/// `Spin { freq_hz: 10.0, voltage: 2.0 }`.
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

    /// Start the open-loop spin: `<freq_hz> <voltage>`
    /// (electrical frequency of the rotating vector, peak phase voltage).
    /// Voltage is clamped to `MAX_OPENLOOP_V` (= 3.0 V) for safety.
    #[command(name = "spin")]
    Spin { freq_hz: f32, voltage: f32 },

    /// Soft-stop the open-loop spin (voltage ramps to 0, then MOE = 0)
    #[command(name = "stop")]
    Stop,
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

/// Decimal `u32` formatter.  Writes digits one byte at a time, so no
/// buffer is allocated and no allocator is required.
fn write_u32<W, E>(cli: &mut CliHandle<'_, W, E>, mut n: u32)
where
    W: embedded_cli::__private::io::Write<Error = E>,
    E: embedded_cli::__private::io::Error,
{
    if n == 0 {
        let _ = cli.writer().write_str("0");
        return;
    }
    let mut digits = [0u8; 10];
    let mut i = digits.len();
    while n > 0 {
        i -= 1;
        digits[i] = b'0' + (n % 10) as u8;
        n /= 10;
    }
    // Safe: each byte is `b'0'..=b'9'`, all ASCII.
    let s = core::str::from_utf8(&digits[i..]).unwrap_or("?");
    let _ = cli.writer().write_str(s);
}

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
                let _ = cli.writer().write_str(BOARD_NAME);
                let _ = cli.writer().write_str("\r\n");
                let _ = cli.writer().write_str(BOARD_MCU);
                let _ = cli.writer().write_str("\r\n");
                let _ = cli.writer().write_str("  flash: ");
                let _ = write_u32(cli, FLASH_SIZE_KB);
                let _ = cli.writer().write_str(" KB\r\n");
                let _ = cli.writer().write_str("  sram:  ");
                let _ = write_u32(cli, SRAM_SIZE_KB);
                let _ = cli.writer().write_str(" KB\r\n");
            }
            ShellCommand::Reboot => {
                let _ = cli.writer().write_str("Rebooting...\r\n");
                // Brief delay so the message reaches the terminal before reset.
                cortex_m::asm::delay(170_000_000 / 20); // ~50 ms at 170 MHz
                SCB::sys_reset();
            }
            ShellCommand::Spin { freq_hz, voltage } => {
                run_spin(cli, freq_hz, voltage);
            }
            ShellCommand::Stop => {
                run_stop(cli);
            }
        }
        Ok(())
    })
}

// ---------------------------------------------------------------------------
// spin / stop — write the shared `OPEN_LOOP_CMD`.
// ---------------------------------------------------------------------------

/// Handle `spin <freq_hz> <voltage>`.
///
/// Clamps `voltage` to `[0, MAX_OPENLOOP_V]` and warns the user on the
/// serial line if their input was out of range. Frequency is taken
/// verbatim — the motor task's `advance_angle` wraps `θ` so any finite
/// value is safe.
fn run_spin<W, E>(cli: &mut CliHandle<'_, W, E>, freq_hz: f32, voltage: f32)
where
    W: embedded_cli::__private::io::Write<Error = E>,
    E: embedded_cli::__private::io::Error,
{
    if !freq_hz.is_finite() || !voltage.is_finite() {
        let _ = cli.writer().write_str("spin: freq/voltage must be finite\r\n");
        return;
    }
    if voltage < 0.0 {
        let _ = cli.writer().write_str("spin: voltage must be ≥ 0; ignored\r\n");
        return;
    }
    let clamped = if voltage > MAX_OPENLOOP_V {
        let _ = cli.writer().write_str("spin: voltage clamped to MAX_OPENLOOP_V\r\n");
        MAX_OPENLOOP_V
    } else {
        voltage
    };
    let cmd = OpenLoopCmd { enabled: true, freq_hz, voltage: clamped };
    OPEN_LOOP_CMD.lock(|c| c.set(cmd));
    let _ = cli.writer().write_str("spin ok\r\n");
}

fn run_stop<W, E>(cli: &mut CliHandle<'_, W, E>)
where
    W: embedded_cli::__private::io::Write<Error = E>,
    E: embedded_cli::__private::io::Error,
{
    // Read-modify-write so a `stop` doesn't accidentally clear
    // frequency/voltage that the user might want to keep for the
    // next `spin` (the motor task ramps them from current value to 0
    // on the `enabled` edge).
    OPEN_LOOP_CMD.lock(|c| {
        let mut cmd = c.get();
        cmd.enabled = false;
        c.set(cmd);
    });
    let _ = cli.writer().write_str("stop ok\r\n");
}
