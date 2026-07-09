//! Async shell task — minimal, no embedded-cli, no core::fmt.

use defmt::info;
use embassy_stm32::usart::{BufferedUartRx, BufferedUartTx};
use embedded_io::Write;
use embedded_io_async::Read;

use crate::shell::commands::{self, REBOOT_REQUESTED};

const CMD_BUF_SIZE: usize = 64;

/// Async shell task. Reads bytes from UART RX, buffers until newline,
/// parses and executes commands.
#[embassy_executor::task]
pub async fn shell_task(mut tx: BufferedUartTx<'static>, mut rx: BufferedUartRx<'static>) {
    info!("shell task started");

    let mut buf = [0u8; CMD_BUF_SIZE];
    let mut len: usize = 0;

    // Prompt
    let _ = tx.write(b"\r\n> ");

    loop {
        let mut byte = [0u8; 1];
        match rx.read(&mut byte).await {
            Ok(_) => {}
            Err(_) => continue,
        }

        match byte[0] {
            b'\r' | b'\n' => {
                if len > 0 {
                    let _ = tx.write(b"\r\n");
                    let cmd = commands::parse_cmd(&buf, len);
                    commands::exec_cmd(&mut tx, cmd);
                    len = 0;
                }
                let _ = tx.write(b"\r\n> ");
            }
            0x08 | 0x7f => {
                // Backspace / DEL
                if len > 0 {
                    len -= 1;
                    let _ = tx.write(b"\x08 \x08");
                }
            }
            b if b.is_ascii_graphic() || b == b' ' => {
                if len < CMD_BUF_SIZE {
                    buf[len] = b;
                    len += 1;
                    let _ = tx.write(&[b]);
                }
            }
            _ => {} // ignore other control chars
        }

        if REBOOT_REQUESTED.load(core::sync::atomic::Ordering::Relaxed) {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(50)).await;
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}
