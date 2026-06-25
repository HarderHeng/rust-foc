# Shell + OTA 设计

**日期:** 2026-06-25
**状态:** 待审核
**目标平台:** ST B-G431B-ESC1 (STM32G431CBU6, 128KB flash)
**前置依赖:** [基础初始化 spec](./2026-06-25-b-g431b-esc1-initialization-design.md)、[128KB flash 容量评估](./2026-06-25-flash-budget-128kb.md)

**Bootloader 选型决定(2026-06-25):** 采用**自写最简 bootloader**(单 slot,~13.5 KB),**不**使用 embassy-boot。理由:STM32G431CBU6 flash 仅 128KB,embassy-boot 的 ACTIVE+DFU 双分区模型把 app 区压到 ~58KB,装不下完整栈(FOC + shell + zencan + 持久化);自写 + 单 slot 给 app 留 108KB 够用。A/B 回滚 / power-fail safety 等 embassy-boot 优势,在换芯片(如 STM32G473 512KB)或上产品时再考虑。

## 目标

在已有基础初始化之上,增加:

1. **交互式 shell**:基于 `embedded-cli`,支持 `help` / `version` / `info` / `reboot` / `ota_update` 5 条命令。
2. **OTA 升级**:通过 USART2 走 y-modem 协议,把新固件刷入 app flash 段。

flash 占用预估 ~80-95 KB,在 128 KB 内有余量(详见容量评估文档)。

## 范围

### 包含

- 拆 workspace:app crate + bootloader crate
- bootloader crate:最简自写,~13.5 KB,无 defmt,纯裸 `uart.write_str` 调试
- y-modem 协议(精简实现,~250-350 行 Rust)
- CRC32 校验(image 末尾 4 字节)
- shell `ota_update` 命令触发:通过 Uart2Sink 提示用户 → 写 flag → sys_reset(详细顺序见状态机章节)
- heartbeat task 改走 defmt,不再持 Uart2Sink(避免 `&mut` 竞争)
- 启动 banner(板子 + MCU + 版本 + 编译期嵌入的 Git SHA 可选)
- 升级状态机:成功 / 失败 / CAN abort / 超时(都明确处理)

### 不包含(YAGNI)

- 升级进度百分比回显(只打印 ACK,不加文件大小)
- 签名 / 加密(明文 + CRC32 足矣;物理接触 USART 的人能伪造)
- A/B 双 slot 回滚
- 自动 boot-time 检测 flag(只由 shell 显式触发)
- 看门狗(30s 超时 + 用户手动 reset 已够)
- 失败计数 / 重试上限
- 升级过程断点续传

## 架构

### Workspace 拓扑

```
foc-rust/                                 # Cargo workspace
├── Cargo.toml                            # workspace 根 [workspace] + [package] for app
├── Cargo.lock                            # 统一锁
├── .cargo/config.toml                    # target + probe-rs runner
├── memory.x                              # **app** 用的 link 脚本
├── build.rs                              # **app** build script
├── src/                                  # **app crate**
│   ├── main.rs
│   ├── bsp.rs
│   ├── drivers/{mod.rs, debug_uart.rs}
│   ├── commands/{mod.rs, shell.rs, ota.rs}
│   └── tasks/{mod.rs, heartbeat.rs, shell.rs}
├── common/                               # **foc-common lib crate**(共享常量)
│   ├── Cargo.toml
│   └── src/lib.rs                        # APP_START, OTA_FLAG, METADATA_* 常量
└── bootloader/                           # **bootloader crate**
    ├── Cargo.toml
    ├── memory.x                          # bootloader 自己的 link 脚本
    ├── build.rs
    └── src/
        ├── main.rs
        ├── ymodem.rs
        └── flag.rs
```

**为什么拆 workspace:** bootloader 和 app 是两个独立的 ELF 二进制,不能塞进同一 crate。app 编译产物是 `target/thumbv7em-none-eabihf/debug/foc-rust.elf`,bootloader 是 `target/.../bootloader.elf`,各自 link 各自的 `memory.x`。`foc-common` 是 lib crate,被两者依赖,共享地址常量。

### Flash 布局(128 KB 实际分布)

STM32G431 的 flash page 大小 = 2 KB(`0x800`)。所有区域按 page 对齐。

```
0x0800_0000  +------------------+ 0 KB
              |  bootloader 代码  |  ~13.5 KB(编译产物)
              |                  |
              |  (link 时填充    |  ~2.5 KB(空 padding 到 page 边界)
              |   到 16 KB)      |
0x0800_3800  +------------------+ 14 KB
              |  config page     |  2 KB
              |  0x0800_3F00:    |        ↑ OTA_FLAG 字节
              |    OTA_FLAG      |        | 0xAA = pending
              |                  |        | 0x00 = no OTA
0x0800_4000  +------------------+ 16 KB
              |  app             |  110 KB
0x0801_F800  +------------------+
              |  metadata        |  2 KB
              |  内容见下        |
0x0802_0000  +------------------+ 128 KB end
```

**关键常量(在 `foc-common` 共享):**

| 常量 | 值 | 说明 |
|---|---|---|
| `APP_START_ADDRESS` | `0x0800_4000` | app 起始(= bootloader 段结束) |
| `APP_END_ADDRESS` | `0x0801_F800` | app 结束(= metadata 起始) |
| `APP_SIZE` | `0x1B800` (= 110 KB) | app 段大小 |
| `OTA_FLAG_ADDRESS` | `0x0800_3F00` | flag 字节地址 |
| `OTA_FLAG_PENDING` | `0xAA` | 表示进入 bootloader 模式 |
| `OTA_FLAG_NONE` | `0x00` | 正常,bootloader 直接跳 app |

**Metadata 段内容(2 KB,`0x0801_F800`):**

| 偏移 | 大小 | 字段 | 用途 |
|---|---|---|---|
| 0x00 | 4 B | `magic` | `0xDEADBEEF`,标识有效 metadata |
| 0x04 | 4 B | `image_size` | 实际 image 字节数(由 app 在 build 后期自动写入) |
| 0x08 | 4 B | `image_crc32` | image 末 4 字节的 CRC32(应与 `foc_common::metadata` 计算结果一致) |
| 0x0C | 16 B | `version` | UTF-8 字符串,如 `"v0.2.0-sha1234567"` |
| 0x1C | 4 B | `build_timestamp` | Unix 秒,`build.rs` 注入 |
| 0x20 | ... | (保留) | 给将来的签名 / extra 字段 |

app 在 build 时(`build.rs` 后期步骤)把 magic + image_size + image_crc32 + version + build_timestamp 拼成一个 32 字节的 blob,链接器把它放在 metadata 段的固定位置。bootloader 跳 app 前读 metadata,看 magic 是否有效、CRC 是否匹配,不匹配则拒绝跳 app(用户 SWD 救)。

> **本阶段简化**:metadata 只用于 bootloader 跳 app 前的 sanity check,不做"OTA 写完后由 bootloader 更新 metadata"——app 自己写,bootloader 只读。这样少写一段 flash 写代码。

### 模块职责

| 模块 | 依赖 | 职责 |
|---|---|---|
| `app::bsp` | embassy-stm32, drivers | 板级配置 + `board_init()` + `reset_to_bootloader()` |
| `app::drivers::debug_uart` | embassy-stm32, embedded-io | `DebugShellSink` trait + `Uart2Sink` (已有,不改) |
| `app::drivers::flash` | embassy-stm32, embedded-storage | `Stm32g4Flash` 实现 `embedded_storage::NorFlash` |
| `app::commands::shell` | embedded-cli, drivers | 命令注册(5 条);提供 `shell_task` 启动入口 |
| `app::commands::ota` | cortex-m, foc-common | `ota_update` 命令:写 flag + 提示 + sys_reset |
| `app::tasks::heartbeat` | defmt, embassy-time | 改走 defmt,持 `Ticker`,不持 Uart2Sink |
| `app::tasks::shell` | drivers, app::commands::shell | 异步任务:uart.read 循环 + cli.process_byte |
| `app::main` | 全部 | composition root |
| `bootloader::main` | embassy-stm32, drivers, foc-common | 入口,检查 flag → y-modem / 跳 app |
| `bootloader::ymodem` | 串口读/写 | 协议状态机:SOH/STX/EOT/ACK/NAK/CAN |
| `bootloader::flag` | foc-common | OTA_FLAG 读/写/清,基于 `OtaFlag` trait |
| `foc-common::OtaFlag` | (无外部 dep) | trait + 共享地址常量 + `FlashOtaFlag<F>` 实现 |

## 状态机

### App 启动

```
上电 / reset
  ↓
bootloader 段执行
  ↓
读 OTA_FLAG
  ├─ 0x00 → 跳 APP_START_ADDRESS(app 启动)
  └─ 0xAA → 进 bootloader 模式(见下)
  ↓
app 启动:main() → HAL init → board_init → spawn heartbeat + shell
  ↓
shell 提示符出现
```

### Bootloader 模式

```
进入 bootloader(flag == 0xAA)
  ↓
打印 banner:
  "=== B-G431B-ESC1 OTA Bootloader v0.1.0 ==="
  "Send y-modem to start (timeout 30s)..."
  ↓
擦 app flash 段(0x0800_4000 ~ 0x0801_F800)
  ↓
y-modem 接收循环:
  ├─ 收到 SOH/STX(数据包) → 写 flash(覆盖)→ ACK → 继续
  ├─ 收到 EOT → 校验 CRC32(image 末 4 字节)→ ACK 或 NAK
  │     ├─ CRC OK → 清 flag = 0x00 → 打印 "OTA OK" → sys_reset
  │     └─ CRC 错 → 打印 "CRC fail" → 继续等重传(EOT 协议)
  ├─ 收到 CAN (abort) → 打印 "aborted, ready for retry" → 重新进 y-modem 接收
  └─ 30s 无活动(无包无 CAN)→ 打印 "OTA timeout, power cycle to return" + 清 flag = 0x00
        继续等(sleep) 等待下一次活动
```

**关键点:**

- 擦 flash 在 y-modem 开始时(不是结束)→ 中途断电安全,下次进入 bootloader 会重擦
- timeout 清 flag → 用户 power cycle 能回 app
- CAN abort 不清 flag → 用户可以在 y-modem 模式重发
- y-modem 协议本身支持 EOT 后重传(CRC 错时 NAK),bootloader 持续在线不退出

### Shell `ota_update` 命令

```
shell 模式
  ↓
用户输入 "ota_update" + 回车
  ↓
ota.rs::OtaUpdateCommand::run(ctx):
  1. ctx.writer().write_str("Rebooting to OTA bootloader, send y-modem now...\n").ok();
     ↑ 通过 embedded-cli 的 writer(底层是 Uart2Sink),用户从串口看得到
  2. 短暂 delay(用 cortex_m::asm::delay 软循环 ~50ms)
     ↑ 必须用 busy-wait,因为 sys_reset 之后 async timer 上下文就没了;
        50ms 大约是 170 MHz × 50e-3 / 4 = ~2.1M 次循环,准确
  3. 写 OTA_FLAG = 0xAA(在 foc-common 常量地址)
  4. cortex_m::peripheral::SCB::sys_reset()
  ↓
CPU 复位 → bootloader 段执行
```

**为什么不用 defmt:** defmt 走 RTT,只有连 ST-LINK + probe-rs 才看得到。本场景下用户用纯 USART2 终端,看不到 RTT。`Rebooting...` 必须从 USART2 走。

**为什么用 busy-wait 而不是 `embassy_time::Timer`:** `Timer::after().await` 需要 executor 调度,而 `sys_reset` 之后一切都消失了,await 永远不会被 poll;reset 可能在 await 还没 resume 时就触发了。busy-wait 同步等待,确定性最强。

## 关键设计决策

### 1. 升级协议:y-modem

**为什么 y-modem 不是 XMODEM/ZMODEM/裸二进制:**

- 包大小 1024B(对比 XMODEM 128B)
- 起始包带文件名 + size,bootloader 无需预知 image 长度
- CRC16 per packet + NAK 重传 + CAN abort + EOT 结束
- 主流串口终端都自带 y-modem send(minicom / picocom / TeraTerm / PuTTY / screen),host 端零工具开发
- 实现 ~250-350 行,ZMODEM 翻 3 倍

### 2. Bootloader 实现:自写最简

- 不引 `stm32-bootloader` / `mcuboot`(它们抽象多,本场景过重)
- 全部自己写,代码量 ~600-800 行(ymodem 350 + flash 200 + crc 100 + main 150)
- 强约束:**bootloader 代码 ≤ 14 KB**(实际 flash 占用 16 KB,因 page 对齐;代码本身 ≤ 14 KB)
- 验证手段:`cargo size` 每次改 bootloader 后必跑,看 `text` 段是否 ≤ 14 KB

### 3. 状态存储:1 字节 flag

- 1 个 byte 足够:0x00 / 0xAA
- 放在 bootloader 段的最后一页(0x0800_3F00),不与 app 重叠
- flash 写之前先做 page erase(单 byte 在 STM32G4 必须先擦)
- 不用 option byte /不用备份

### 4. 串口归属切换

| 模式 | 持 Uart2Sink 的角色 |
|---|---|
| App 启动后 | **shell task 独占** |
| heartbeat | **不持 sink**,改走 defmt 通道 |
| bootloader 模式 | bootloader 独占 |

shell task 和 heartbeat task 之间没有 `&mut` 竞争。`Uart2Sink` 在 board_init 后直接 move 进 shell task。heartbeat 改用 `defmt::info!`,走 RTT。

### 5. 时钟

- bootloader 沿用 app 的默认时钟配置:`embassy_stm32::init(Default::default())`
- G431CB 默认 HSI16 → PLL → sysclk 170MHz,APB1 45MHz,USART2 源时钟 45MHz 下 BRR 4-bit fraction 可精准给出 921600(误差 <0.5%)。
- 不为 bootloader 写 custom clock config

### 6. CRC32 用 STM32G4 硬件 CRC 外设(配置成 CRC-32/ISO-HDLC)

**算法选择:** **CRC-32/ISO-HDLC**(zlib / gzip / PNG / Ethernet / zip 用的标准 CRC32)

| 参数 | 值 |
|---|---|
| 多项式 | `0x04C11DB7` |
| 初始值 | `0xFFFFFFFF` |
| 输入位反转 | 按字节(byte-level) |
| 输出位反转 | 是 |
| 最终 XOR | `0xFFFFFFFF`(软件做) |

**STM32G4 CRC 外设现状(参考手册 RM0440):**

| 配置项 | 寄存器 | 默认值 | 标准 CRC-32 需求 | 需配置? |
|---|---|---|---|---|
| 多项式 | `CRC.POL` | `0x04C11DB7` ✓ | 同 | 否 |
| 多项式位数 | `CRC.CR.POLYSIZE` | 32 bits ✓ | 同 | 否 |
| 初始值 | `CRC.INIT` | `0xFFFFFFFF` ✓ | 同 | 否 |
| 输入反转 | `CRC.CR.REV_IN` | 不反转 ✗ | 字节反转 | **是** |
| 输出反转 | `CRC.CR.REV_OUT` | 不反转 ✗ | 反转 | **是** |
| 最终 XOR | — | (无寄存器) | XOR 0xFFFFFFFF | **软件做** |

**实际代码(~15 行,不是 1 行):**

```rust
// bootloader/src/crc.rs (~15 lines)
use embassy_stm32::pac::CRC;

pub fn crc32_init() {
    let crc = unsafe { &*CRC::ptr() };
    crc.cr().modify(|_, w| unsafe {
        w.rev_in().bits(0b10)     // 字节级输入反转
         .rev_out().set_bit()    // 输出反转
        // POL、POLYSIZE、INIT 都用默认值,已经是 ISO-HDLC 配置
    });
}

pub fn crc32_update(data: &[u8]) {
    let crc = unsafe { &*CRC::ptr() };
    for &b in data {
        crc.dr().write(|w| unsafe { w.dr().bits(b as u32) });
    }
}

pub fn crc32_finalize() -> u32 {
    let crc = unsafe { &*CRC::ptr() };
    // 外设没 final XOR,软件做
    crc.dr().read().dr().bits() ^ 0xFFFF_FFFF
}
```

**build.rs 端同步:** app 编译时也用 ISO-HDLC CRC32 算 image 末 4 字节。可用 `crc32fast` crate(`build-dependencies`),或调 Python `zlib.crc32()` 算后注入。

**收益(对比手写表驱动软件 CRC32):**
- `bootloader/crc.rs` 从 ~100 行降到 ~15 行(模块仍保留,只是瘦了)
- 省 1KB flash(256 字节表 + 反转/移位代码)
- 省 256 字节 RAM(CRC 表不放在 RAM)
- 性能 0 损失(硬件 100KB ~ 0.5ms,软件 ~ 1ms)
- 一致性:build.rs 与 bootloader 算同一种标准 CRC32,Python `zlib.crc32()` 可独立验证

## 错误处理

| 情况 | 处理 |
|---|---|
| y-modem 收 NAK 重传超过 10 次 | 打印 "too many retries, sending 'C' to restart" → 重新发 'C' 引导 |
| flash 写失败(擦错/写错) | 打印 "flash write err at offset 0xXXXX" → 停止写,继续等 y-modem 重传 |
| app 段内容校验失败(校验和 vs metadata) | y-modem 写入完成后做一次全段 CRC32,与 metadata 中的期望值比对,失败拒跳 app |
| bootloader 自己 crash | 用户只能 SWD 重烧(单 slot 限制,接受) |
| 升级中途 app 段擦空但新 image 没写完 | 下次进 bootloader(flag 还设)会**重新擦 + 重收**,单 slot 模式就是会重擦 |
| 升级成功但 app 启动后立即 crash | 用户 power cycle → bootloader 看到 flag 已清 → 跳 app(还是坏) → SWD 救 |

## 测试 / 验证

1. **编译通过**:`cargo build` (app) + `cargo build -p bootloader` 都干净
2. **flash 占用**:`cargo size` 验证 bootloader ≤ 14 KB,app ≤ 110 KB
3. **bootloader 独立烧录**:`probe-rs` 烧 bootloader 到 `0x0800_0000`,再烧 app 到 `0x0800_4000`
4. **基本跳 app**:上电后看到 shell prompt(说明 bootloader 跳 app 成功)
5. **shell 命令**:`help` `version` `info` `reboot` 各试一次
6. **y-modem 升级端到端**:
   - 改 `version` 字符串,`cargo build --release`
   - shell `ota_update` → bootloader 提示
   - terminal 发新 bin(y-modem send)
   - 看到 ACK 流
   - OTA OK → 跳新 app → 看到新 `version`
7. **失败恢复**:
   - shell `ota_update` → bootloader 模式
   - 在 terminal 故意 abort(CAN)
   - 看到 "aborted" → 再发一次
   - 升级成功
8. **超时恢复**:
   - shell `ota_update` → bootloader 模式
   - 不发任何东西,等 30s
   - 看到 "OTA timeout" 消息
   - power cycle → 跳回 app(flag 已清)
9. **断电恢复**:
   - shell `ota_update` → bootloader 模式
   - 收一半 y-modem 包,拔电
   - 上电 → bootloader 看到 flag → 重擦 + 重收
   - 重发完整 image → 升级成功

## 文件结构

```
foc-rust/                                  # workspace 根
├── Cargo.toml                             # [workspace] + [package] for app
├── Cargo.lock
├── .cargo/config.toml
├── memory.x                               # app 用的:FLASH=128K 全,APP_START=0x08004000
├── build.rs                               # app 的 build script
├── src/
│   ├── main.rs
│   ├── bsp.rs                             # 增加 reset_to_bootloader 辅助
│   ├── drivers/
│   │   ├── mod.rs
│   │   ├── debug_uart.rs                  # 不变
│   │   └── flash.rs                       # 新:Stm32g4Flash: NorFlash
│   ├── commands/                          # CLI 命令集(原 app/, 去重命名)
│   │   ├── mod.rs
│   │   ├── shell.rs                       # 新:命令注册表
│   │   └── ota.rs                         # 新:OtaUpdateCommand
│   └── tasks/
│       ├── mod.rs
│       ├── heartbeat.rs                   # 改:走 defmt
│       └── shell.rs                       # 新:shell_task
├── common/                                # foc-common lib crate
│   ├── Cargo.toml
│   └── src/
│       ├── lib.rs                         # APP_START, OTA_FLAG, METADATA_* 常量
│       └── flag.rs                        # 新:OtaFlag trait + FlashOtaFlag 实现
├── bootloader/
│   ├── Cargo.toml                         # 独立 crate
│   ├── memory.x                           # bootloader 段专属:FLASH 0..16K,RAM 0..32K
│   ├── build.rs                           # link memory.x
│   └── src/
│       ├── main.rs                        # 入口 + 状态机
│       ├── ymodem.rs                      # y-modem 协议
│       └── flag.rs                        # OTA_FLAG 操作(走 foc-common::OtaFlag)
└── docs/
    └── superpowers/
        └── specs/
            ├── 2026-06-25-b-g431b-esc1-initialization-design.md
            ├── 2026-06-25-flash-budget-128kb.md
            └── 2026-06-25-shell-and-ota-design.md   # 本文档
```

## 依赖清单

### app crate (在 workspace root Cargo.toml)

```toml
[dependencies]
# Embassy(版本号 = 锁当前 major,允许 patch/minor 浮动)
embassy-stm32    = { version = "0.6",  features = ["stm32g431cb", "defmt", "time-driver-any", "unstable-pac"] }
embassy-executor = { version = "0.10", features = ["arch-cortex-m", "executor-thread", "defmt"] }
embassy-time     = { version = "0.5",  features = ["defmt", "tick-hz-32_768"] }
embassy-sync     = "0.8"

# Rust embedded 标准 trait
embedded-io     = "0.7"     # UART/SPI 等字节流抽象
embedded-storage = "0.3"    # NorFlash trait(替代自写 FlashStorage)

# 日志
defmt     = "1"
defmt-rtt = "1"

# Cortex-M 基础
cortex-m    = "0.7"
cortex-m-rt = "0.7"
panic-probe = { version = "1", features = ["print-defmt"] }

# shell(锁版本:生态里没有更靠谱的,bus factor 1 可接受)
embedded-cli = { version = "0.2.1", default-features = false, features = ["ufmt"] }

# 与 bootloader 共享
foc-common = { path = "common" }

[dev-dependencies]
# host-side 测试 flash 抽象
embedded-storage-inmemory = "0.1"

[build-dependencies]
# build.rs 算 image CRC32(ISO-HDLC),写到 metadata 段
crc32fast = "1"
```

> **注意:** 不引 `embassy-build` 这个 crate(crates.io 不存在)。link script 通过 `build.rs` 里 `println!("cargo:rustc-link-arg-bins=...")` 直接打。

### bootloader crate

```toml
[dependencies]
# Bootloader 不需要 async runtime,只引底层
embassy-stm32    = { version = "0.6",  features = ["stm32g431cb", "time-driver-any", "unstable-pac"] }
cortex-m         = "0.7"
cortex-m-rt      = "0.7"
embedded-io      = "0.7"     # Uart 字节流
embedded-storage = "0.3"     # NorFlash trait
foc-common       = { path = "../common" }
```

**不引 defmt / defmt-rtt / embassy-executor / embassy-time** — bootloader 是纯同步,裸 `uart.write_str` 调试。这省下 ~2-3KB flash。

### `foc-common` lib crate

```toml
[dependencies]
# 纯常量 + OtaFlag trait,无外部 dep
```

```rust
// common/src/lib.rs
#![no_std]

// 共享地址常量
pub const APP_START_ADDRESS: u32 = 0x0800_4000;
pub const APP_END_ADDRESS: u32 = 0x0801_F800;
pub const APP_SIZE: u32 = APP_END_ADDRESS - APP_START_ADDRESS;  // 0x1B800
pub const OTA_FLAG_ADDRESS: u32 = 0x0800_3F00;
pub const OTA_FLAG_PENDING: u8 = 0xAA;
pub const OTA_FLAG_NONE: u8 = 0x00;

// Metadata 段(magic + image size + CRC + version + timestamp,详见 flash 布局章节)
pub const METADATA_ADDRESS: u32 = 0x0801_F800;
pub const METADATA_MAGIC: u32 = 0xDEAD_BEEF;
pub const METADATA_SIZE: usize = 32;  // 实际只用 32 字节,段内保留空间
```

```rust
// common/src/flag.rs
use embedded_storage::nor_flash::NorFlash;

pub enum OtaState { Pending, None }

/// flag 操作抽象。app 和 bootloader 各自实现各自的。
/// 双方通过 `foc-common` 共享状态字节地址。
pub trait OtaFlag {
    fn read(&self) -> OtaState;
    fn set_pending(&mut self) -> Result<(), FlashError>;
    fn clear(&mut self) -> Result<(), FlashError>;
}

/// 唯一实现(目前)。基于任何 `NorFlash` + 地址。
/// 由 `bsp` 实例化并通过 trait 暴露给上层。
pub struct FlashOtaFlag<F: NorFlash> { storage: F, addr: u32 }

impl<F: NorFlash> OtaFlag for FlashOtaFlag<F> { ... }

pub type FlashError = F::Error;
```

两个 crate 通过 `foc-common` 共享常量与 `OtaFlag` 抽象。`foc-common` 自身只依赖 `embedded-storage` (trait-only,不影响体积)。

## 未来可能性:embassy-boot(暂不引入)

embassy-rs 官方有 `embassy-boot` + `embassy-boot-stm32`(2026-03 还在更新,见 [embassy-boot 0.7](https://docs.embassy.dev/embassy-boot/)),提供:

- A/B 双分区 + swap 状态机
- Power-fail 安全升级
- 可选 ed25519 签名验证
- ~8KB bootloader + 2KB state

**为何本期不引入(128KB flash 装不下):**

| 方案 | bootloader | state | ACTIVE | DFU | app 总可用 |
|---|---|---|---|---|---|
| 自写(本 spec) | 13.5KB | 0(用 config page 替代) | 108KB | 0(单 slot) | **108 KB** |
| embassy-boot(单 slot 假象) | 8KB | 2KB | 58KB | 58KB+ | 58 KB(被压半) |
| embassy-boot(真 A/B) | 8KB | 2KB | 28KB | 28KB | 28 KB |

完整栈(FOC + shell + zencan + 持久化)估 ~67 KB,只有自写能装下。

**何时重新评估:**

- 换芯片到 STM32G473 (512KB flash)→ ACTIVE 翻 4 倍,embassy-boot 优势显现
- 商业化,需要 A/B 回滚防止"升级失败砖机"风险
- 上了实际产品,愿意接受 embassy-boot 的"用空间换可靠性"

**移植成本预估:** 半天到一天,把 f3 example 的 linker 与 main.rs 改一下。主要工作:替换 y-modem 调用为 `FirmwareUpdater::write_firmware`,bootloader 改用 `BootLoader::prepare::<2048>()` + `unsafe load()`。`OtaFlag` trait 改用 `FirmwareState` API,其余代码不需要动。

## 风险与未决项

- **bootloader 写错就砖**:任何 bootloader 代码改动必须 SWD 重烧,失败只能 SWD 救。建议 bootloader 改动后,先在另一块板上测试,或者至少保留能 SWD 连的路径。
- **y-modem 实现易踩坑**:超时、CAN 检测、ETB/EOT、CRC16 校验,200-300 行容易有边界 case。要 1-2 天专门写+测。
- **跳 app 时 `SCB.VTOR` 重设**:bootloader 跳 app 之前必须重设 VTOR 到 APP_START_ADDRESS,漏写会跳到 bootloader 的中断向量,hard fault。
- **flash 写锁**:STM32G4 的 flash 在 Option Byte 控制下能锁住(读/写保护),本项目**不**写保护 option byte,以保留现场修复能力。
- **workspace 首次构建慢**:新加 workspace + bootloader + foc-common,首次 `cargo build` 会多编译一份 embassy-stm32,要几分钟。
- **CAN 字节干扰**:y-modem 协议里 CAN 字节序列是 5 个 0x18,但用户终端输入任意字符时可能产生 CAN。需要 bootloader 严格按 5 字节 0x18 序列检测,而不是单字节。

## 与之前 spec 的关系

- **继承**:基础初始化 spec(USART2、defmt、heartbeat、分层、DebugShellSink、Uart2Sink 全部不变)
- **修订**:基础初始化 spec 中"heartbeat 任务使用 Uart2Sink 写心跳字符串"改为"heartbeat 任务使用 defmt 写心跳字符串",因为 `Uart2Sink` 现在归 shell task 独占。**行为差异**:heartbeat 不再出现在 USART2 串口,只在 RTT(probe-rs 终端)可见。shell 提示符与命令响应仍走 USART2。
- **新增**:本 spec 引入 workspace 结构、`foc-common` crate、bootloader crate
- **flash 容量**:本 spec 落地后,app 估计 ~85-95 KB,bootloader 13.5 KB,合计 ~100-110 KB(留 18-28 KB 余量给后续 FOC 等)

## 后续

- FOC 主算法(spec 3)
- zencan CANopen 集成(spec 4)
- 电机参数持久化(spec 5)
- 看门狗 / 低功耗(spec 6)
- 真实签名 / OTA 加密(看产品化需求)

每个独立 spec,本设计文档不背。
