# B-G431B-ESC1 嵌入式项目初始化设计

**日期:** 2026-06-25
**状态:** 待审核
**目标平台:** ST B-G431B-ESC1 (MCU: STM32G431CBU6, 128KB flash, 32KB SRAM)

## 目标

为 foc-rust 项目搭建一个 Embassy Rust 嵌入式工程的**最小骨架**:
- 完成依赖与工具链配置
- 完成时钟与基础外设初始化
- 调通 USART2(PB3 TX / PB4 RX)作为将来的**调试 shell** 通道
- 集成 defmt 日志
- 按 embassy 官方推荐的"driver 与 app 通过 trait 解耦"的分层方式组织代码

本阶段**不**实现 shell 协议、电机控制、复杂任务编排,只确保系统能起、能打印、能验证时钟与串口。

## 设计范围

### 包含

- 单 crate binary 工程(后续可拆 workspace)
- Cargo / .cargo / build.rs / memory.x / rust-toolchain.toml 完整配置
- `bsp`、`drivers::debug_uart`、`tasks::heartbeat` 三个模块
- USART2 + GPIO(AF7)+ 中断驱动的 async 写入
- defmt 链路(`defmt-rtt` 走 SWD,直接 printf 走 USART2 — 两条日志通道并存)
- 一条 heartbeat 任务,每 500ms 调 `DebugShellSink::write` 写心跳字符串 + 写一条 defmt 日志

> 关于 LED:本阶段**不**接入 LED 指示。B-G431B-ESC1 的 LD3 占用 PB6,与板载 ST-LINK VCP 用的 USART1_TX 复用,本阶段我们用 USART2,所以也不影响。**但**这意味着 LD3 状态由 ST-LINK 决定,我们不应再把 PB6 当 GPIO 使用。如果后续真的需要 LED 指示,要么换一组未复用的 GPIO,要么用 B-G431B-ESC1 上的其他 LED。本 spec 排除该决策。

### 不包含(YAGNI)

- shell 命令解析、行编辑器(VT100/ANSI)
- 电机控制 / FOC 算法
- DMA 优化(后续如需再上)
- 任何其他外设(CAN、SPI、I2C、ADC)
- 多任务优先级细分
- 低功耗/STOP 模式

## 架构分层

| 层 | 模块 | 依赖 | 职责 |
|---|---|---|---|
| HAL | `embassy_stm32`(外部) | — | 直接提供 USART、GPIO、Peripherals |
| Driver | `src/drivers/debug_uart.rs` | `embassy_stm32` | 定义 `DebugShellSink` trait;`Uart2Sink` 把 USART2 适配成 trait |
| BSP | `src/bsp.rs` | `drivers`, `embassy_stm32` | 板级常量(板名、pin 映射、baud);`board_init()` 装配 HAL 外设,返回 `BoardHandles` |
| Tasks | `src/tasks/*` | `drivers`(**只依赖 trait**) | 异步任务;本阶段只 `heartbeat` |
| App/Composition | `src/main.rs` | 全部 | `embassy_stm32::init` → `bsp::board_init` → spawn tasks |

**关键约束:**

- tasks 文件夹**不允许**写 `use embassy_stm32::...`,只能 `use crate::drivers::debug_uart::DebugShellSink`。
- 这样未来替换 shell 后端或单测时,tasks 不用改。

## 关键设计决策

### 1. USART 选择: USART2, 不是 USART1

STM32G431CBU6 上 PB3/PB4 的 AF7 是 **USART2**(`USART2_TX` / `USART2_RX`)。USART1 的 PB6/PB7 在 B-G431B-ESC1 上接到了板载 ST-LINK VCP(USB 转串口),用户**已经能**通过 USB 虚拟串口拿到 USART1 输出了。我们自己板子上的 USART2 是 free-to-use 的一对,正好用来跑 shell。两条链路并存,defmt 走 RTT,shell 走 USART2。

### 2. `DebugShellSink` trait 抽象

```rust
pub trait DebugShellSink {
    type Error: core::fmt::Debug;
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error>;
    async fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        // 默认实现:把 &str 转字节,调 write
        self.write(s.as_bytes()).await.map(|_| ())
    }
    async fn flush(&mut self) -> Result<(), Self::Error>;
}
```

- 只暴露异步写,**不**暴露读。读将来单独做 `DebugShellSource` trait(本阶段不写)。
- `Error` 关联类型,便于替换底层。
- 默认实现的 `write_str` 减少调用方样板。

**为什么用 trait:** tasks 拿到的是 `&mut impl DebugShellSink`,完全不知道底下是 USART2、Mock,还是内存缓冲区。test 时塞个 `Vec<u8>` 实现就行。

### 3. BSP 装配方式

`bsp.rs` 不持有任何状态,只提供:

```rust
pub const BOARD_NAME: &str = "B-G431B-ESC1";
pub const BOARD_MCU: &str = "STM32G431CBU6";

pub const DEBUG_UART_BAUD: u32 = 115_200;
pub const DEBUG_UART_TX_PIN: char = 'B'; pub const DEBUG_UART_TX_NUM: u8 = 3;
pub const DEBUG_UART_RX_PIN: char = 'B'; pub const DEBUG_UART_RX_NUM: u8 = 4;
pub const DEBUG_UART_AF: u8 = 7;

pub struct BoardHandles {
    pub debug_uart: Uart2Sink<'static, Async>,
}

pub fn board_init(p: embassy_stm32::Peripherals) -> BoardHandles;
```

main.rs 调 `board_init` 拿到 `BoardHandles`,把里面的字段 move 进各个 task。**BSP 不保留任何 Peripheral 的所有权** — 所有权随装配结果交出去。

### 4. 时钟

- 不写 custom config,直接用 `embassy_stm32::init(Default::default())`。
- G431CB 默认配置:HSI16 → PLL → sysclk 170MHz,APB1 45MHz,APB2 90MHz。USART2 在 APB1,源时钟 45MHz,经过 USART 时钟分频后能精准给出 115200。

### 5. 日志双通道

- **defmt-rtt** 走 SWD/RTT(由 `probe-rs` 读)—— 用于开发期高频、低延迟、结构化日志。
- **USART2** 走真实串口 —— 用于现场/无人值守场景、心跳回显、shell 输出。

两者并存,**没有**统一接口。开发者按需选。

## 数据流(本阶段)

```
embassy_stm32::init() ──┐
                        ├─→ bsp::board_init(p) ─→ BoardHandles
                        │                              │
                        │         ┌────────────────────┤
                        │         ▼                    ▼
                        │   Uart2Sink            Output(LED)
                        │         │
                        ▼         ▼
            spawn(heartbeat(sink, led))
                        │
                        ▼
              每 500ms:
                - sink.write_str("[hb] tick\n").await
                - defmt::info!("heartbeat tick")
```

## 错误处理

- USART2 写失败 → `defmt::error!` 打日志,task panic(本阶段不写重连/恢复)
- heartbeat task panic → 由 `embassy-executor` 默认 panic handler 兜底,CPU 死循环 + 打印

## 测试 / 验证

由于这是裸机固件,本阶段**不**写 unit test(需要 mock,成本不低)。验证靠:

1. **编译通过**:`cargo build` 在 `thumbv7em-none-eabihf` 上无 warning
2. **下载成功**:`cargo run` (probe-rs) 能烧录
3. **运行可见**:
   - RTT 通道看到 defmt 心跳
   - USART2 串口(`screen /dev/ttyACM* 115200` 或 USB-TTL 接 PB3/PB4)看到 `[hb] tick` 字符串

## 文件结构

```
foc-rust/
├── Cargo.toml
├── .cargo/
│   └── config.toml                # target + probe-rs runner
├── memory.x                       # flash 128K, SRAM 32K
├── build.rs                       # embassy-build → bindings.rs
├── rust-toolchain.toml            # 锁 stable(1.81+)+ 必要 component
├── src/
│   ├── main.rs
│   ├── bsp.rs
│   ├── drivers/
│   │   ├── mod.rs
│   │   └── debug_uart.rs
│   └── tasks/
│       ├── mod.rs
│       └── heartbeat.rs
└── docs/
    └── superpowers/
        └── specs/
            └── 2026-06-25-b-g431b-esc1-initialization-design.md
```

## 依赖清单(Cargo.toml)

```toml
[dependencies]
embassy-stm32 = { version = "*", features = ["stm32g431cb", "defmt", "time-driver-any", "unstable-pac"] }
embassy-executor = { version = "*", features = ["arch-cortex-m", "executor-thread", "defmt"] }
embassy-time = { version = "*", features = ["defmt", "tick-hz-32_768"] }
embassy-sync = "*"

defmt = "*"
defmt-rtt = "*"

cortex-m = "*"
cortex-m-rt = "*"
panic-probe = { version = "*", features = ["print-defmt"] }

[build-dependencies]
embassy-build = "*"

[profile.release]
debug = 2        # defmt 需要 DWARF
opt-level = "s"  # size 优化,因为 flash 只有 128K
lto = true
codegen-units = 1
strip = true
```

## 风险与未决项

- **`.cargo/config.toml` 中的 probe-rs runner 路径**:不同机器上 `probe-rs` 安装方式不同(sudo apt / cargo install / VSCode),runner 行不写死,只示例,文档里说明。
- **G431CB 的 embassy 特性名**:有可能写成 `stm32g431cb` 也可能 `stm32g431c-b`(下划线/连字符的差异)。写代码时以 `cargo build` 报错提示为准,通常会自动展开。
- **RTT 内存段**:defmt-rtt 需要 RTT 控制块放在一个固定段。`memory.x` 之外可能要补 `build.rs` 标记 `._defmt_rtt` 之类(本阶段先按 embassy 官方模板走,出问题再调)。
- **rust-toolchain**:先写 stable;若 embassy-stm32 当前版本对 G4 仍要求 nightly,再降级到指定 nightly。
- **依赖版本**:Cargo.toml 里写 `"*"`,让 cargo 解出当前可用版本。锁版本是 CI 阶段的事,初始骨架阶段不锁。

## 后续

通过本 spec 的实施,下一阶段可独立做的:

- `DebugShellSource` trait + 接收侧异步读
- 行编辑 / 命令解析 / 命令注册表(`shell` 模块)
- 多个 sensor/actuator 任务加入,SPI/I2C/ADC 驱动按同样模式沉淀
- 看门狗 / 低功耗

每个都单独写 spec,本设计文档不背它们。
