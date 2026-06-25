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
- USART2(`BufferedUart`)+ GPIO(AF7)+ TX 中断驱动的 ringbuffer 入队
- defmt 链路(`defmt-rtt` 走 SWD,直接 printf 走 USART2 — 两条日志通道并存)
- `embedded-io` 0.7 接口(给将来 `embedded-cli` 用) + 我们自家 `DebugShellSink` trait(给应用层 task 用)
- 一条 heartbeat 任务,每 500ms 调 `DebugShellSink::write_str` 写心跳字符串 + 写一条 defmt 日志

> 关于 LED:本阶段**不**接入 LED 指示。B-G431B-ESC1 的 LD3 占用 PB6,与板载 ST-LINK VCP 用的 USART1_TX 复用,本阶段我们用 USART2,所以也不影响。**但**这意味着 LD3 状态由 ST-LINK 决定,我们不应再把 PB6 当 GPIO 使用。如果后续真的需要 LED 指示,要么换一组未复用的 GPIO,要么用 B-G431B-ESC1 上的其他 LED。本 spec 排除该决策。

### 不包含(YAGNI)

- shell 命令解析、行编辑器(VT100/ANSI)—— **下个 spec** 实施
- 电机控制 / FOC 算法
- DMA 优化(后续如需再上)
- 任何其他外设(CAN、SPI、I2C、ADC)
- 多任务优先级细分
- 低功耗/STOP 模式

## 架构分层

| 层 | 模块 | 依赖 | 职责 |
|---|---|---|---|
| HAL | `embassy_stm32`(外部) | — | 直接提供 USART、GPIO、Peripherals |
| External lib | `embedded-cli`(外部) | `embedded-io` | 下个 spec 才用,本 spec 仅锁版本 |
| Driver | `src/drivers/debug_uart.rs` | `embassy_stm32`, `embedded-io` | 定义 `DebugShellSink` trait;`Uart2Sink` 同时实现 `DebugShellSink` 和 `embedded_io::Write` |
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

- 只暴露 `write_str` 便利方法,**不**暴露原始 `write`/`flush`(那些是 `embedded_io::Write` 的事,不该在这里重复)。`write_str` 是同步方法(`embedded_io::Write::write_str` 是 sync 的),调用方负责把数据切成小块以避免长阻塞。
- `Error` 关联类型,便于替换底层。

```rust
pub trait DebugShellSink {
    type Error: core::fmt::Debug;
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error>;
}
```

**为什么不直接 `embedded_io::Write`:** 我们应用层 task 写日志/写命令响应,**绝大多数**场景是 `&str`,`write_str` 写起来比 `write(s.as_bytes())` 短 5 倍,且避免在 task 里重复 `.as_bytes()`。`embedded_io::Write` 留给 driver 与第三方库(`embedded-cli`)用。两个 trait 各司其职。

**Uart2Sink 同时实现两个 trait**(在 driver 里写两个 `impl` 块):

```rust
impl DebugShellSink for Uart2Sink<'_> { ... }   // 给我们 task 用
impl embedded_io::Write for Uart2Sink<'_> { ... }  // 给 embedded-cli 用
```

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
    pub debug_uart: Uart2Sink<'static, BufferedUart>,  // ringbuffer 256B
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

### 6. 调试 shell 库选型:`embedded-cli` (funbiscuit/embedded-cli-rs)

**调研结论(2026-06-25):**

| 维度 | 结论 |
|---|---|
| 最新 release | 0.2.1 (2024-03),但 GitHub 仍活跃 — 最近 commit 2026-05-20 |
| License | MIT OR Apache-2.0 |
| 维护 | 单人(funbiscuit),bus factor = 1,有风险但当前活跃 |
| 内存占用 | 16 KiB ROM + 0.6 KiB RAM(Arduino Nano 实测) |
| async | **非 async**,同步 API |
| Embassy 兼容 | ✅ 通过 `embedded-io` 0.7 — embassy-stm32 USART 已实现 `embedded_io::Read/Write` |
| defmt 集成 | 内部用 `ufmt`,非 defmt — 包一层即可 |
| 输入模型 | 同步逐字节:`cli.process_byte(b, &mut writer)` |

**为什么选它(权衡记录):**
- 写交互式 shell(行编辑、历史、补全、子命令、help 文本生成)从零开始成本极高
- Rust 生态里没有"原生 async CLI",任何 shell 库本质都是同步 + 上层桥接
- `embedded-io` 是 Rust embedded 事实标准接口,选它就等于选生态兼容
- 单人维护的 bus factor 风险可以接受:本仓库有 spec 与 vendor 信息固定,必要时 fork 维护

**对本架构的影响:**
- `Uart2Sink` 必须同时实现 `DebugShellSink` 和 `embedded_io::Write`(双 trait,见设计决策 #2)
- 下游 shell task 会调 `cli.process_byte(b, &mut uart_sink)`,**这里 `process_byte` 是同步方法**,会运行在 shell task 的调用栈上(不 await)
- **阻塞风险**:UART 走 `BufferedUart` + 256B ringbuffer,`Write::write` 是入队而非真发送,绝大多数情况下 O(1) 返回。只有当 ringbuffer 满时才会忙等,需在命令 handler 里手工分片让出 — 见风险与未决项

### 7. UART2 用 `BufferedUart`,不带 DMA

为避免 `Write::write` 长时间阻塞 shell task,USART2 配置为:
- `BufferedUart`,TX ringbuffer 256B,RX ringbuffer 64B
- TX 中断驱动(不轮询),无需 DMA(本阶段单条串口无 DMA 收益,DMA 留给将来 FOC 高速外设)
- 发送 `write(&[u8])` 是入队,几乎不阻塞
- 接收 `read(&mut [u8])` 阻塞等到 ringbuffer 至少有 1 字节(因为没设超时)

ringbuffer 大小经验值:
- TX 256B:够一次性塞下典型命令响应(几十字节)+ prompt(几个字节)+ 行编辑回显
- RX 64B:够接收 1-2 次按键,常规 read 1 字节调用不浪费

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

**下个 spec 会增加:**

```
└── src/
    ├── app/
    │   ├── mod.rs
    │   └── shell.rs        # embedded-cli 集成
    └── tasks/
        └── shell.rs        # shell_task
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

embedded-io = "0.7"

cortex-m = "*"
cortex-m-rt = "*"
panic-probe = { version = "*", features = ["print-defmt"] }

# 本 spec 不直接用,锁版本以备下个 spec 写 shell 时一致
embedded-cli = { version = "0.2.1", default-features = false, features = ["ufmt"] }

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
- **UART ringbuffer 阻塞**:当 shell 一次性输出 > 256B 时(例如 dump 整个 `defmt` 日志),`Write::write` 会因 ringbuffer 满而忙等,**阻塞 shell task 期间 executor 不调度其他 task**。本阶段不写大输出命令,无影响;但 shell 实现 spec 必须约定:长输出必须分片 + 主动让出(例如 `for chunk in data.chunks(64) { w.write(chunk)?; embassy_time::Timer::after_millis(1).await; }`)。
- **`embedded-cli` 单维护者风险**:funbiscuit 一人维护,bus factor = 1。仓库里固定 crate 版本(写进 `Cargo.lock`),fork 路径在 `funbiscuit/embedded-cli-rs` 文档化。

## 后续

**直接后续(下个 spec,本 spec 结束后立刻):`embedded-cli` shell 集成**

新增 `app::shell` 模块,作为独立 task:

```rust
// app/shell.rs (本 spec 不写,这是下个 spec 的内容)
#[embassy_executor::task]
pub async fn shell_task(
    mut uart: Uart2Sink<'static, BufferedUart>,  // 同一 Uart2Sink,实现 embedded_io::Write
) {
    let mut writer = ...;  // 我们的 writer 包装
    let mut cli = Cli::builder()
        .writer(...)
        .command(...)
        .build();
    let mut buf = [0u8; 1];
    loop {
        // Read 1 字节(uart.read 阻塞到 ringbuffer 至少 1 字节)
        uart.read(&mut buf).await.ok();
        // process_byte 同步调用,可能产生响应
        cli.process_byte(buf[0], &mut writer);
    }
}
```

集成关注点:
- `Uart2Sink` 既给 heartbeat 用(实现 `DebugShellSink`),又给 shell 用(实现 `embedded_io::Write`)。**两个 consumer,但 `DebugShellSink::write_str` 与 `embedded_io::Write::write` 都走同一个 ringbuffer**。安全吗?取决于 `Uart2Sink` 内部是否需要 `&mut self` — 若是,**同一时刻只能一个 task 持有**,所以 heartbeat 和 shell 各自要 clone/own 独立实例,或者走 `embassy_sync::Mutex`。
- heartbeat 是否还在本 spec 范围?—— 是,作为存活证明。**下个 spec 把 heartbeat 改成"只在 shell 没启动时跑"** 或干脆停掉。
- 命令注册:首批 `help`、`version`、`reset`、`reboot`(各自在 `app::shell::commands` 子模块下用 `#[derive(embedded_cli::Command)]`)。

**更后续:**

- 多个 sensor/actuator 任务加入,SPI/I2C/ADC 驱动按同样模式沉淀
- 看门狗 / 低功耗
- FOC 电机控制

每个都单独写 spec,本设计文档不背它们。
