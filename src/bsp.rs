//! Board Support Package for ST B-G431B-ESC1.
//!
//! This module owns no persistent state. It exposes board identity
//! constants and an `board_init()` function that takes raw HAL
//! peripherals and returns typed handles ready to be moved into tasks.

use embassy_stm32::{
    bind_interrupts,
    peripherals::USART2,
    rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllPreDiv, PllRDiv, PllSource,
        PllMul, Sysclk,
    },
    time::Hertz,
    usart::{BufferedInterruptHandler, BufferedUart, Config as UsartConfig},
    Config as HalConfig, Peripherals,
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
pub const FLASH_SIZE_KB: u32 = 128;
pub const SRAM_SIZE_KB: u32 = 32;

pub const DEBUG_UART_BAUD: u32 = 921_600;

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

/// System clock: HSE 8 MHz → PLL ×85 /4 = 170 MHz.
///
/// HSE chosen over HSI for stable baud rate (±1% HSI vs crystal).
/// `boost: true` for sysclk > 150 MHz (RM0440 §7.4.3).
pub fn clocks() -> HalConfig {
    let mut config = HalConfig::default();
    config.rcc.hsi = false;
    config.rcc.hse = Some(Hse {
        freq: Hertz::mhz(8),
        mode: HseMode::Oscillator,
    });
    config.rcc.sys = Sysclk::PLL1_R;
    config.rcc.pll = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL85,
        divp: None,
        divq: None,
        divr: Some(PllRDiv::DIV4),  // 680/4 = 170 MHz sysclk
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV4;
    config.rcc.apb2_pre = APBPrescaler::DIV1;
    config.rcc.boost = true;
    config
}

pub fn board_init(p: Peripherals) -> BoardHandles {
    // SAFETY: This is a single-threaded (pre-executor) init function;
    // no other code has access to these statics yet.
    let tx_buf: &'static mut [u8] =
        unsafe { &mut *(&raw mut DEBUG_UART_TX_BUF as *mut [u8; DEBUG_UART_TX_BUF_SIZE]) };
    let rx_buf: &'static mut [u8] =
        unsafe { &mut *(&raw mut DEBUG_UART_RX_BUF as *mut [u8; DEBUG_UART_RX_BUF_SIZE]) };

    let mut cfg = UsartConfig::default();
    cfg.baudrate = DEBUG_UART_BAUD;

    let buffered: BufferedUart<'static> = BufferedUart::new(
        // Pin assignments are hardcoded below; embassy-stm32 takes
        // concrete peripheral pins at compile time, not runtime values,
        // so the constants for these would be documentation-only.
        // B-G431B-ESC1 schematic: USART2 = PB3 (TX), PB4 (RX), AF7.
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