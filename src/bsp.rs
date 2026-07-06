//! Board Support Package for ST B-G431B-ESC1.
//!
//! This module owns no persistent state. It exposes board identity
//! constants and an `board_init()` function that takes raw HAL
//! peripherals and returns typed handles ready to be moved into tasks.

use embassy_stm32::{
    bind_interrupts,
    can::Can,
    gpio::OutputType,
    peripherals::USART2,
    rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllPreDiv, PllRDiv, PllSource,
        PllMul, Sysclk,
    },
    time::Hertz,
    timer::{
        complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin, IdlePolarity},
        low_level::CountingMode,
        simple_pwm::PwmPin,
    },
    usart::{BufferedInterruptHandler, BufferedUart, Config as UsartConfig},
    Config as HalConfig, Peripherals,
};

use crate::drivers::can::init_fdcan1;
use crate::drivers::debug_uart::Uart2Sink;
use crate::drivers::motor_pwm::MotorPwm;

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

/// TIM1 PWM switching frequency. Center-aligned, so each "period" is
/// two counts of the auto-reload register. With sysclk 170 MHz and
/// ARR = 4250 we get exactly 20 kHz.
pub const PWM_FREQ_HZ: u32 = 20_000;

/// TIM1 software dead-time. The hardware gate driver adds another
/// 800 ns on top, so the total effective dead time is ~1.55 µs.
/// `set_dead_time` takes a value in TIM1 clock cycles; at 170 MHz each
/// cycle is 5.88 ns, so 128 cycles ≈ 753 ns.
pub const DEAD_TIME_CYCLES: u16 = 128;

// Static buffers for the ringbuffers — must be `'static` so the
// BufferedUart can outlive the BSP scope and be moved into a task.
static mut DEBUG_UART_TX_BUF: [u8; DEBUG_UART_TX_BUF_SIZE] = [0; DEBUG_UART_TX_BUF_SIZE];
static mut DEBUG_UART_RX_BUF: [u8; DEBUG_UART_RX_BUF_SIZE] = [0; DEBUG_UART_RX_BUF_SIZE];

bind_interrupts!(struct Irqs {
    USART2 => BufferedInterruptHandler<USART2>;
});

pub struct BoardHandles {
    pub debug_uart: DebugUartSink,
    /// Three-phase inverter on TIM1, MOE=0 (idle) on return.
    pub motor_pwm: MotorPwm<'static>,
    /// FDCAN1 on PB9 (TX) / PA11 (RX) for the OTA-side CANopen +
    /// UDS protocol stack. Always in `NormalOperationMode` on
    /// return; the canopen task does the boot-up message and
    /// the 1 Hz heartbeat.
    pub can: Can<'static>,
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

    // --------------------------------------------------------------
    // TIM1 — 3-phase complementary PWM, 20 kHz center-aligned, 750 ns
    // software dead-time, MOE = 0 (idle / safe) on return.
    //
    // Pin map (from B-G431B-ESC1 schematic, cross-checked against the
    // ST MCSDK reference .ioc):
    //   CH1:  PA8   (high)         CH1N: PC13  (low)
    //   CH2:  PA9   (high)         CH2N: PA12  (low)
    //   CH3:  PA10  (high)         CH3N: PB15  (low)
    //
    // `CenterAlignedDownInterrupts` is the centre-aligned mode that
    // triggers channel-interrupts on the count-down half. We don't
    // use the interrupts, so the choice among the three centre-aligned
    // modes is arbitrary — DownInterrupts is what the MCSDK reference
    // uses.
    // --------------------------------------------------------------
    let mut pwm = ComplementaryPwm::new(
        p.TIM1,
        Some(PwmPin::new(p.PA8,  OutputType::PushPull)),
        Some(ComplementaryPwmPin::new(p.PC13, OutputType::PushPull)),
        Some(PwmPin::new(p.PA9,  OutputType::PushPull)),
        Some(ComplementaryPwmPin::new(p.PA12, OutputType::PushPull)),
        Some(PwmPin::new(p.PA10, OutputType::PushPull)),
        Some(ComplementaryPwmPin::new(p.PB15, OutputType::PushPull)),
        None, None, // ch4 unused
        Hertz::hz(PWM_FREQ_HZ),
        CountingMode::CenterAlignedDownInterrupts,
    );

    // Hardware dead-time: 750 ns at 170 MHz t_clk.
    pwm.set_dead_time(DEAD_TIME_CYCLES);

    // Idle state on MOE=0: force low-sides ON, high-sides OFF so
    // current can freewheel through the low-side body diode of
    // every phase — the standard safe freewheel for a 3-phase
    // inverter. embassy-stm32 0.6 only exposes two `IdlePolarity`
    // variants (the timer hardware has no "all Hi-Z" mode through
    // this register); the "all Hi-Z" we'd ideally want is
    // achieved by leaving MOE=0 (which the BSP does — see below).
    // When MOE=0, the timer outputs are in this idle state
    // regardless of polarity. The uncharged-bootstrap-shoot-through
    // concern only applies once MOE=1 with both sides switching
    // and dead-time violated; with MOE=0 the half-bridge is
    // statically biased and no shoot-through is possible.
    pwm.set_output_idle_state(
        &[embassy_stm32::timer::Channel::Ch1,
          embassy_stm32::timer::Channel::Ch2,
          embassy_stm32::timer::Channel::Ch3],
        IdlePolarity::OisnActive,
    );

    // Enable per-channel complementary outputs but leave MOE=0 — the
    // pin driver is configured, but the bridge outputs are gated off
    // until the motor task calls `MotorPwm::enable()`.
    pwm.enable(embassy_stm32::timer::Channel::Ch1);
    pwm.enable(embassy_stm32::timer::Channel::Ch2);
    pwm.enable(embassy_stm32::timer::Channel::Ch3);
    pwm.set_master_output_enable(false);

    BoardHandles {
        debug_uart: Uart2Sink::new(buffered),
        motor_pwm: MotorPwm::new(pwm),
        can: init_fdcan1(p.FDCAN1, p.PB9, p.PA11),
    }
}