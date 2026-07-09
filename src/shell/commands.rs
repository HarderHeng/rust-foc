//! Minimal shell command parser — no embedded-cli, no core::fmt.

use crate::bsp::{BOARD_MCU, BOARD_NAME, FLASH_SIZE_KB, SRAM_SIZE_KB};
use crate::motor::cmd::{OpenLoopCmd, OPEN_LOOP_CMD};
use crate::motor::open_loop::MAX_OPENLOOP_V;

pub static REBOOT_REQUESTED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

const MAX_SPIN_FREQ_HZ: f32 = 500.0;

// ── Parse a single command line ──

pub enum Cmd {
    Help,
    Version,
    Info,
    Reboot,
    Spin { freq_hz: f32, voltage: f32 },
    Stop,
    Unknown,
}

/// Parse a null-terminated command buffer.
pub fn parse_cmd(buf: &[u8], len: usize) -> Cmd {
    let input = &buf[..len];
    let s = core::str::from_utf8(input).unwrap_or("");
    let s = s.trim();

    if s.is_empty() {
        return Cmd::Unknown;
    }

    let (cmd, args) = match s.find(' ') {
        Some(pos) => (&s[..pos], s[pos + 1..].trim()),
        None => (s, ""),
    };

    match cmd {
        "help" | "?" => Cmd::Help,
        "version" | "ver" => Cmd::Version,
        "info" => Cmd::Info,
        "reboot" => Cmd::Reboot,
        "stop" => Cmd::Stop,
        "spin" => {
            let mut parts = args.split_whitespace();
            let freq: f32 = match parts.next().and_then(|s| fast_parse_f32(s)) {
                Some(v) => v,
                None => return Cmd::Unknown,
            };
            let voltage: f32 = match parts.next().and_then(|s| fast_parse_f32(s)) {
                Some(v) => v,
                None => return Cmd::Unknown,
            };
            Cmd::Spin { freq_hz: freq, voltage }
        }
        _ => Cmd::Unknown,
    }
}

/// Cheap f32 parser: integer + optional fraction, no exponent.
fn fast_parse_f32(s: &str) -> Option<f32> {
    let s = s.as_bytes();
    if s.is_empty() {
        return None;
    }
    let mut i = 0;
    let negative = s[0] == b'-';
    if negative {
        i = 1;
    }
    let mut int_part: u32 = 0;
    while i < s.len() && s[i].is_ascii_digit() {
        int_part = int_part.saturating_mul(10).saturating_add((s[i] - b'0') as u32);
        i += 1;
    }
    let mut frac: f32 = 0.0;
    if i < s.len() && s[i] == b'.' {
        i += 1;
        let mut div = 10.0_f32;
        while i < s.len() && s[i].is_ascii_digit() {
            frac += (s[i] - b'0') as f32 / div;
            div *= 10.0;
            i += 1;
        }
    }
    if i != s.len() {
        return None;
    }
    let mut val = int_part as f32 + frac;
    if negative {
        val = -val;
    }
    Some(val)
}

// ── Execute a parsed command ──

/// Execute a command. Returns a static response string (no formatting needed).
pub fn exec_cmd<W: embedded_io::Write>(writer: &mut W, cmd: Cmd) {
    match cmd {
        Cmd::Help => {
            let _ = writer.write(
                b"help | ?       list commands\r\n\
                  version | ver  firmware version\r\n\
                  info           chip + flash info\r\n\
                  reboot         reset MCU\r\n\
                  spin <hz> <v>  open-loop spin\r\n\
                  stop           stop spin\r\n",
            );
        }
        Cmd::Version => {
            let _ = writer.write(env!("CARGO_PKG_VERSION").as_bytes());
            let _ = writer.write(b"\r\n");
        }
        Cmd::Info => {
            let _ = writer.write(BOARD_NAME.as_bytes());
            let _ = writer.write(b"\r\n");
            let _ = writer.write(BOARD_MCU.as_bytes());
            let _ = writer.write(b"\r\n  flash: ");
            let _ = write_u32(writer, FLASH_SIZE_KB);
            let _ = writer.write(b" KB\r\n  sram:  ");
            let _ = write_u32(writer, SRAM_SIZE_KB);
            let _ = writer.write(b" KB\r\n");
        }
        Cmd::Reboot => {
            let _ = writer.write(b"Rebooting...\r\n");
            run_stop_inner();
            REBOOT_REQUESTED.store(true, core::sync::atomic::Ordering::Relaxed);
        }
        Cmd::Spin { freq_hz, voltage } => run_spin(writer, freq_hz, voltage),
        Cmd::Stop => {
            run_stop_inner();
            let _ = writer.write(b"stop ok\r\n");
        }
        Cmd::Unknown => {
            let _ = writer.write(b"unknown command\r\n");
        }
    }
}

// ── Helpers ──

fn write_u32<W: embedded_io::Write>(w: &mut W, mut n: u32) {
    if n == 0 {
        let _ = w.write(b"0");
        return;
    }
    let mut digits = [0u8; 10];
    let mut i = digits.len();
    while n > 0 {
        i -= 1;
        digits[i] = b'0' + (n % 10) as u8;
        n /= 10;
    }
    let _ = w.write(&digits[i..]);
}

fn run_spin<W: embedded_io::Write>(w: &mut W, freq_hz: f32, voltage: f32) {
    if !freq_hz.is_finite() || !voltage.is_finite() {
        let _ = w.write(b"spin: values must be finite\r\n");
        return;
    }
    if freq_hz < 0.0 {
        let _ = w.write(b"spin: freq must be >= 0\r\n");
        return;
    }
    if voltage < 0.0 {
        let _ = w.write(b"spin: voltage must be >= 0\r\n");
        return;
    }
    let clamped_f = if freq_hz > MAX_SPIN_FREQ_HZ { MAX_SPIN_FREQ_HZ } else { freq_hz };
    let clamped_v = if voltage > MAX_OPENLOOP_V { MAX_OPENLOOP_V } else { voltage };
    OPEN_LOOP_CMD.lock(|c| c.set(OpenLoopCmd {
        enabled: true,
        freq_hz: clamped_f,
        voltage: clamped_v,
    }));
    let _ = w.write(b"spin ok\r\n");
}

fn run_stop_inner() {
    OPEN_LOOP_CMD.lock(|c| {
        let mut cmd = c.get();
        cmd.enabled = false;
        c.set(cmd);
    });
}
