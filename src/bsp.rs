//! Board Support Package for ST B-G431B-ESC1.
//!
//! This module owns no persistent state. It exposes board identity
//! constants and an `board_init()` function that takes raw HAL
//! peripherals and returns typed handles ready to be moved into tasks.

// Placeholder constants and enum are not used until Task 5 wires up USART2.
// Suppress dead-code warnings for the public API surface that task modules
// will depend on.
#![allow(dead_code)]

use embassy_stm32::Peripherals;

pub const BOARD_NAME: &str = "B-G431B-ESC1";
pub const BOARD_MCU: &str = "STM32G431CBU6";

// Debug serial: USART2, PB3 (TX), PB4 (RX), AF7
pub const DEBUG_UART: PeriName = PeriName::Usart2;
pub const DEBUG_UART_BAUD: u32 = 115_200;
pub const DEBUG_UART_TX_PORT: char = 'B';
pub const DEBUG_UART_TX_PIN: u8 = 3;
pub const DEBUG_UART_RX_PORT: char = 'B';
pub const DEBUG_UART_RX_PIN: u8 = 4;
pub const DEBUG_UART_AF: u8 = 7;

/// Named peripheral placeholders. We use a single enum until Task 5
/// replaces this with concrete `embassy_stm32::peripherals::*` types.
#[derive(Debug, Clone, Copy)]
pub enum PeriName {
    Usart2,
}

/// Board handles returned by `board_init()`. Each field is moved
/// into the task that owns it; BSP retains nothing.
pub struct BoardHandles {
    // USART2 handle will be added in Task 5.
}

impl BoardHandles {
    fn new() -> Self {
        Self {}
    }
}

/// Initialize the board: takes HAL peripherals, returns typed handles.
///
/// The actual wiring of USART2 happens in Task 5. For now this is
/// a passthrough that proves the composition root pattern.
pub fn board_init(_p: Peripherals) -> BoardHandles {
    defmt::info!("{} on {}: board_init (no-op)", BOARD_NAME, BOARD_MCU);
    BoardHandles::new()
}
