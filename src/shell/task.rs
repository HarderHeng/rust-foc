//! Async shell task — minimal, no embedded-cli, no core::fmt.
//!
//! Line editor with left/right arrow cursor movement, backspace,
//! and printable-character insert at any position in the buffer.

use defmt::info;
use embassy_stm32::usart::{BufferedUartRx, BufferedUartTx};
use embedded_io_async::{Read, Write};

use crate::shell::commands::{self, REBOOT_REQUESTED};

const CMD_BUF_SIZE: usize = 64;

/// Async shell task. Reads bytes from UART RX, buffers until newline,
/// parses and executes commands.
#[embassy_executor::task]
pub async fn shell_task(mut tx: BufferedUartTx<'static>, mut rx: BufferedUartRx<'static>) {
    info!("shell task started");

    let mut buf = [0u8; CMD_BUF_SIZE];
    let mut len: usize = 0;
    let mut pos: usize = 0;

    // Prompt
    let _ = tx.write(b"\r\n> ").await;

    loop {
        let mut byte = [0u8; 1];
        match rx.read(&mut byte).await {
            Ok(_) => {}
            Err(_) => continue,
        }

        match byte[0] {
            b'\r' | b'\n' => {
                if len > 0 {
                    let _ = tx.write(b"\r\n").await;
                    let cmd = commands::parse_cmd(&buf, len);
                    commands::exec_cmd(&mut tx, cmd).await;
                    len = 0;
                    pos = 0;
                }
                let _ = tx.write(b"\r\n> ").await;
            }

            0x08 | 0x7f => {
                // Backspace / DEL — delete character before cursor
                if pos > 0 {
                    pos -= 1;
                    // Shift characters after deleted position left by one
                    for i in pos..len - 1 {
                        buf[i] = buf[i + 1];
                    }
                    len -= 1;
                    // Redraw from cursor to end
                    let _ = tx.write(&buf[pos..len]).await;
                    let _ = tx.write(b" ").await;
                    let back = len - pos + 1;
                    for _ in 0..back {
                        let _ = tx.write(b"\x08").await;
                    }
                }
            }

            0x1b => {
                // ANSI escape sequence — read next two bytes
                let mut seq = [0u8; 2];
                let n = rx.read(&mut seq).await.unwrap_or(0);
                if n == 2 && seq[0] == b'[' {
                    match seq[1] {
                        b'D' => {
                            // Left arrow
                            if pos > 0 {
                                pos -= 1;
                                let _ = tx.write(b"\x1b[D").await;
                            }
                        }
                        b'C' => {
                            // Right arrow
                            if pos < len {
                                let _ = tx.write(b"\x1b[C").await;
                                pos += 1;
                            }
                        }
                        _ => {}
                    }
                }
            }

            b if b.is_ascii_graphic() || b == b' ' => {
                if len < CMD_BUF_SIZE {
                    if pos < len {
                        // Insert in the middle — shift content right
                        for i in (pos..len).rev() {
                            buf[i + 1] = buf[i];
                        }
                        buf[pos] = b;
                        len += 1;
                        // Redraw from cursor to end, then back up
                        let _ = tx.write(&buf[pos..len]).await;
                        let back = len - (pos + 1);
                        pos += 1;
                        for _ in 0..back {
                            let _ = tx.write(b"\x08").await;
                        }
                    } else {
                        // Append at end
                        buf[len] = b;
                        len += 1;
                        pos = len;
                        let _ = tx.write(&[b]).await;
                    }
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
