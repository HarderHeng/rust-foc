//! Board Support Package for ST B-G431B-ESC1.
//!
//! This module owns no persistent state. It exposes board identity
//! constants and an `board_init()` function that takes raw HAL
//! peripherals and returns typed handles ready to be moved into tasks.

// The board constants are part of the public BSP API; tasks 6/7 will
// reference them. Suppress dead-code warnings until they are consumed.
#![allow(dead_code)]

use embassy_stm32::{
    bind_interrupts,
    peripherals::USART2,
    usart::{BufferedInterruptHandler, BufferedUart, Config},
    Peripherals,
};

use crate::drivers::debug_uart::Uart2Sink;

/// Type alias for the debug UART sink handed to tasks.
///
/// Using an alias (rather than a bare `Uart2Sink<BufferedUart<'static>>`)
/// lets the tasks layer stay free of HAL imports — it depends only on
/// the BSP's public type, not on `embassy_stm32` directly.
pub type DebugUartSink = Uart2Sink<BufferedUart<'static>>;

pub const BOARD_NAME: &str = "B-G431B-ESC1";
pub const BOARD_MCU: &str = "STM32G431CBU6";

pub const DEBUG_UART_BAUD: u32 = 921_600;
pub const DEBUG_UART_TX_PORT: char = 'B';
pub const DEBUG_UART_TX_PIN: u8 = 3;
pub const DEBUG_UART_RX_PORT: char = 'B';
pub const DEBUG_UART_RX_PIN: u8 = 4;
pub const DEBUG_UART_AF: u8 = 7;

pub const DEBUG_UART_TX_BUF_SIZE: usize = 256;
pub const DEBUG_UART_RX_BUF_SIZE: usize = 64;

// Static buffers for the ringbuffers — must be `'static` so the
// BufferedUart can outlive the BSP scope and be moved into a task.
static mut DEBUG_UART_TX_BUF: [u8; DEBUG_UART_TX_BUF_SIZE] = [0; DEBUG_UART_TX_BUF_SIZE];
static mut DEBUG_UART_RX_BUF: [u8; DEBUG_UART_RX_BUF_SIZE] = [0; DEBUG_UART_RX_BUF_SIZE];

bind_interrupts!(struct Irqs {
    USART2 => BufferedInterruptHandler<USART2>;
});

pub struct BoardHandles {
    pub debug_uart: DebugUartSink,
}

pub fn board_init(p: Peripherals) -> BoardHandles {
    // SAFETY: This is a single-threaded (pre-executor) init function;
    // no other code has access to these statics yet.
    let tx_buf: &'static mut [u8] =
        unsafe { &mut *(&raw mut DEBUG_UART_TX_BUF as *mut [u8; DEBUG_UART_TX_BUF_SIZE]) };
    let rx_buf: &'static mut [u8] =
        unsafe { &mut *(&raw mut DEBUG_UART_RX_BUF as *mut [u8; DEBUG_UART_RX_BUF_SIZE]) };

    let mut cfg = Config::default();
    cfg.baudrate = DEBUG_UART_BAUD;

    let buffered: BufferedUart<'static> = BufferedUart::new(
        p.USART2,
        p.PB4, // RX
        p.PB3, // TX
        tx_buf,
        rx_buf,
        Irqs,
        cfg,
    )
    .unwrap();

    BoardHandles {
        debug_uart: Uart2Sink::new(buffered),
    }
}