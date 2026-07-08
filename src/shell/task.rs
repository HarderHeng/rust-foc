//! Async shell task.
//!
//! Owns the debug UART (USART2) split into TX / RX halves.
//!   - The TX half is wrapped to provide `embedded-io` v0.6 `Write` for
//!     `embedded-cli` 0.2.1 (which depends on v0.6 internally).
//!   - The RX half provides `embedded_io_async::Read` for byte-level input.
//!
//! The loop reads one byte at a time from the RX half (async, waiting until
//! data is available) and feeds it into `Cli::process_byte()`. The Cli
//! handles line editing, built-in help (`-h`), and command dispatch.

use defmt::info;
use embassy_stm32::usart::{BufferedUartRx, BufferedUartTx};
use embedded_cli::__private::io as eio06;
use embedded_io_async::Read;

use crate::shell::commands::{make_processor, ShellCommand, REBOOT_REQUESTED};
use crate::drivers::debug_uart::UsartError06;

const CMD_BUF_SIZE: usize = 64;
const HIST_BUF_SIZE: usize = 128;

// ── embedded-io v0.6 Write adapter for BufferedUartTx ──
//
// embedded-cli 0.2.1 uses embedded-io v0.6 internally, while embassy
// implements v0.7.  This adapter bridges the two.

struct TxWriter06 {
    inner: BufferedUartTx<'static>,
}

impl eio06::ErrorType for TxWriter06 {
    type Error = UsartError06;
}

impl eio06::Write for TxWriter06 {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        <BufferedUartTx<'_> as embedded_io::Write>::write(&mut self.inner, buf)
            .map_err(UsartError06::from)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        <BufferedUartTx<'_> as embedded_io::Write>::flush(&mut self.inner)
            .map_err(UsartError06::from)
    }
}

// ── Shell task ──

/// Async shell task.  Takes ownership of the TX and RX halves of the debug
/// UART and runs an interactive command-line interface.
#[embassy_executor::task]
pub async fn shell_task(tx: BufferedUartTx<'static>, mut rx: BufferedUartRx<'static>) {
    info!("shell task started");

    // Wrap TX into a v0.6 writer for embedded-cli.
    let tx06 = TxWriter06 { inner: tx };

    // Build the Cli (line editor + prompt).  The Cli owns the writer
    // and uses it for prompt, echo, and command output.
    let mut cli = embedded_cli::cli::CliBuilder::default()
        .writer(tx06)
        .command_buffer([0u8; CMD_BUF_SIZE])
        .history_buffer([0u8; HIST_BUF_SIZE])
        .build()
        .unwrap();

    // Build the processor closure once (dispatches to all 5 commands).
    let mut processor = make_processor::<TxWriter06, _>();

    loop {
        // Read one byte from the UART RX (async, waits for data).
        let mut buf = [0u8; 1];
        match rx.read(&mut buf).await {
            Ok(_) => {}
            Err(e) => {
                // Framing/overrun errors are common on noisy USB-serial
                // bridges; log once-per-error and skip the byte.  The
                // CLI line editor recovers automatically on the next
                // valid byte.
                defmt::warn!("USART2 read error: {:?}", e);
                continue;
            }
        }

        // Feed the byte into the CLI line editor / command processor.
        // The `C` generic parameter is `ShellCommand` (provides help /
        // autocomplete).  The `P` parameter is the processor closure.
        let _ = cli.process_byte::<ShellCommand, _>(buf[0], &mut processor);

        // If the `reboot` command set the flag, perform the async
        // delay here (yields to the executor so the motor task can
        // ramp down) then fire NVIC reset.
        if REBOOT_REQUESTED.load(core::sync::atomic::Ordering::Relaxed) {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(50)).await;
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}
