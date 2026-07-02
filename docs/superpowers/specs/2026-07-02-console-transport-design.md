# 设计:`ConsoleTransport` 抽象 + 共享 USART 初始化

**日期:** 2026-07-02
**状态:** 设计稿(未实现)
**背景:** 当前 `tasks/shell.rs` 拿 `BufferedUartTx<'static, USART2>` + `BufferedUartRx<'static, USART2>`,bootloader 自己重写了一遍 USART2 init。换成 USART1 (PB6/PB7) 要改 ~3 个文件,换成 CAN 要 fork 整个 task。

**目标:** 不实现。先把 trait 表面、文件归属、迁移路径定下来。等真要换 USART instance 或换 CAN 时,按这个 spec 一次到位。

---

## 1. 范围

### 包含
- `ConsoleRead` / `ConsoleWrite` / `ConsoleTransport` 三个 trait。
- 一个 `UartConsole<T: Instance>` 实现,把 embassy 的 split buffered UART 包成 `ConsoleTransport`。
- 一个 `ConsoleWriter06<W: ConsoleWrite>` 适配器,把 `ConsoleWrite` 转成 `embedded-io` v0.6 的 `Write`(因为 `embedded-cli` 0.2.1 吃 v0.6)。
- `foc-common::init_usart*_pb*_pb*_921600()` 共享函数,bootloader 和 app 都调。
- `shell_task<T: ConsoleTransport>(transport: T)` 替代当前拿 split TX/RX 的签名。

### 不包含
- CAN 的 `ConsoleTransport` 实现(等真要换时再写,文档给出指引)。
- `embedded-cli` 升级到 0.3+(那是另一个 spec)。
- 把 `Uart2Sink` 重命名成 `DebugSink` / `UartSink`(顺手的 rename,跟 trait 重构一起做)。

---

## 2. 核心问题与设计原则

**问题 1: 换 USART instance 应该改 1 个文件。** 现状是改 3-5 个,因为 `Uart2Sink` / `DebugUartSink` 别名把 `USART2` 漏到类型层。

**问题 2: shell 不应被 byte stream 绑死。** 现状是 `tasks/shell.rs` 直接拿 `BufferedUartTx/Rx`,换 CAN 要 fork 整个 task。

**原则:**
- **Type-level 不漏具体外设。** `shell_task` 的签名只看到 `ConsoleTransport`,看不到 `USART` / `CAN` / 任何具体实例。
- **Bootloader 和 app 共享 init 函数。** 同一块 USART2 初始化逻辑(走 PAC)只在 `foc-common` 写一次。
- **Reader / Writer 独立。** shell task 读 / 写 不互相 borrow,各自走 `&mut self`,可以独立持有。
- **trait 表面是 async read + sync write。** read 必须是 async(CAN mailbox 等待,UART RX FIFO 等待);write 保持 sync(embedded-cli 0.2.1 吃 v0.6 `Write`,本身是 sync)。CAN 写阻塞在 transport 内部解决(Mailbox 满 → busy-wait 或 small buffer)。

---

## 3. Trait 设计

放在新文件 `src/drivers/console.rs`(不进 `foc-common`,因为 bootloader 不需要 console):

```rust
//! Console transport abstraction.
//!
//! The shell task feeds bytes one at a time into `embedded-cli` and
//! writes responses back. This trait surface is what the shell task
//! sees — concrete transports (UART, CAN, USB) implement it.

use core::fmt::Debug;

/// Read side: async byte stream.
pub trait ConsoleRead {
    type Error: Debug;

    /// Read one byte. `Ok(None)` = transport closed gracefully
    /// (USB unplugged, CAN bus-off, etc.) — the shell should print
    /// a message and exit. `Err(_)` is a transient read error —
    /// the shell logs and continues.
    async fn read_byte(&mut self) -> Result<Option<u8>, Self::Error>;
}

/// Write side: sync string + flush. `embedded-cli` 0.2.1 is sync-write,
/// so the writer is sync. A transport that needs async flush
/// (e.g. CAN with a full TX mailbox) blocks inside `write_str` or
/// buffers internally; the trait does not expose async write.
pub trait ConsoleWrite {
    type Error: Debug;

    /// Write a UTF-8 string. ASCII output from the shell is the
    /// only caller; non-UTF-8 returns an error.
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error>;

    /// Flush any buffered bytes to the transport. Default no-op
    /// for transports without a buffer.
    fn flush(&mut self) -> Result<(), Self::Error> { Ok(()) }
}

/// Owns the read + write halves. `split` consumes the transport
/// because the concrete type (e.g. `BufferedUartTx` + `BufferedUartRx`)
/// doesn't allow two `&mut self` to coexist once the CLI has taken
/// ownership of the writer.
///
/// Each impl defines its own `Reader` / `Writer` types — they
/// typically hold the underlying driver halves by value.
pub trait ConsoleTransport: Sized {
    type Reader: ConsoleRead;
    type Writer: ConsoleWrite;

    fn split(self) -> (Self::Reader, Self::Writer);
}
```

**关键设计决定 (note 给将来 reviewer):**
- 没有用 GAT (`type Reader<'a>`),只用了 owned split。原因:`BufferedUartTx` 本身是 owned 句柄,没有借用,owned split 是最简单的。GAT 等到真要 zero-copy 借用再说。
- `read_byte` 返回 `Result<Option<u8>, E>`,不是 `Result<u8, E>`。CAN bus-off / USB unplug 这类"传输层关了"的状态用 `None` 表示,跟普通错误区分开。
- `Error` 是 associated type 不是 trait object。shell task 调 `defmt::warn!("err: {:?}", e)`,需要 Debug 约束;不想要 dyn 抽象。

---

## 4. UART 实现

新文件 `src/drivers/uart_console.rs`:

```rust
use embassy_stm32::usart::{BufferedUartRx, BufferedUartTx, Instance};
use embedded_io::{Read as _, Write as _};

use crate::drivers::console::{ConsoleRead, ConsoleTransport, ConsoleWrite};
use crate::drivers::debug_uart::UsartError06; // 保留并通用化

/// Concrete transport: embassy split buffered UART.
pub struct UartConsole<T: Instance> {
    tx: BufferedUartTx<'static, T>,
    rx: BufferedUartRx<'static, T>,
}

impl<T: Instance> UartConsole<T> {
    pub fn new(
        tx: BufferedUartTx<'static, T>,
        rx: BufferedUartRx<'static, T>,
    ) -> Self { Self { tx, rx } }
}

pub struct UartReader<T: Instance> {
    rx: BufferedUartRx<'static, T>,
}

impl<T: Instance> ConsoleRead for UartReader<T> {
    type Error = UsartError06;  // v0.6 — 跟现在一样
    async fn read_byte(&mut self) -> Result<Option<u8>, Self::Error> {
        let mut buf = [0u8; 1];
        match self.rx.read(&mut buf).await {
            Ok(_) => Ok(Some(buf[0])),
            // v0.6 EmbeddedIoError 没有 WouldBlock;read_exact / read 在
            // 已连接但还没数据时返回类似 ErrorKind::Other 而不是 block.
            // 我们简单地把所有错误都视为"transient,继续读"。
            Err(e) => Err(UsartError06::from(e)),
        }
    }
}

pub struct UartWriter<T: Instance> {
    tx: BufferedUartTx<'static, T>,
}

impl<T: Instance> ConsoleWrite for UartWriter<T> {
    type Error = UsartError06;
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        use embedded_io::Write;
        self.tx.write_all(s.as_bytes())
            .map_err(UsartError06::from)?;
        Ok(())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        use embedded_io::Write;
        self.tx.flush().map_err(UsartError06::from)
    }
}

impl<T: Instance> ConsoleTransport for UartConsole<T> {
    type Reader = UartReader<T>;
    type Writer = UartWriter<T>;
    fn split(self) -> (Self::Reader, Self::Writer) {
        (UartReader { rx: self.rx }, UartWriter { tx: self.tx })
    }
}
```

**`UsartError06` 复用**:`src/drivers/debug_uart.rs` 里现在那个 `UsartError06(UsartError)` 改成 generic over `T: Instance`,或者用 `embassy_stm32::usart::Error`(已经是 generic),看哪个更省模板代码。

---

## 5. embedded-cli 0.2.1 v0.6 桥接

新文件 `src/drivers/console.rs` 同目录,`ConsoleWriter06`:

```rust
use embedded_io_06 as eio06;
use crate::drivers::console::ConsoleWrite;

/// Adapter: `ConsoleWrite` → `embedded-io` v0.6 `Write` (what
/// `embedded-cli` 0.2.1 wants).
pub struct ConsoleWriter06<W: ConsoleWrite> {
    inner: W,
}

impl<W: ConsoleWrite> ConsoleWriter06<W> {
    pub fn new(inner: W) -> Self { Self { inner } }
}

impl<W: ConsoleWrite> eio06::ErrorType for ConsoleWriter06<W> {
    type Error = W::Error;
}

impl<W: ConsoleWrite> eio06::Write for ConsoleWriter06<W> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let s = core::str::from_utf8(buf)
            .map_err(|_| /* error variant */)?;
        self.inner.write_str(s)?;
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> { self.inner.flush() }
}
```

**`UsartError06` 加一个 `Utf8` 变体** 或者在 `console.rs` 里定义一个 `ConsoleWriter06Error::Utf8` 枚举。倾向后者,跟 transport 解耦。

---

## 6. shell_task 改签名

`src/tasks/shell.rs`:

```rust
#[embassy_executor::task]
pub async fn shell_task<T: ConsoleTransport>(transport: T) {
    let (mut reader, writer) = transport.split();
    let mut writer06 = ConsoleWriter06::new(writer);

    let mut cli = embedded_cli::cli::CliBuilder::default()
        .writer(&mut writer06)   // 生命周期 dance
        .command_buffer([0u8; CMD_BUF_SIZE])
        .history_buffer([0u8; HIST_BUF_SIZE])
        .build()
        .unwrap();

    let mut processor = make_processor();

    loop {
        match reader.read_byte().await {
            Ok(Some(byte)) => {
                let _ = cli.process_byte::<ShellCommand, _>(byte, &mut processor);
            }
            Ok(None) => {
                defmt::info!("console: transport closed");
                break;
            }
            Err(e) => defmt::warn!("console read err: {:?}", e),
        }
    }
}
```

**生命周期问题:** `CliBuilder::writer(&mut writer06)` 要求 `writer06` 在 `cli` 之前存在;`cli` 持有 `&mut writer06`,所以 `writer06` 不能 move 走。解决办法:`writer06` 在 `shell_task` 栈上,跟 `cli` 一起活到 task 结束。`reader` 是独立的,跟 `writer06` 互不 borrow(因为 `ConsoleTransport::split` 是 owned,不是 borrow)。

**shell 命令 + processor 不变。** `make_processor()` 还在 `src/commands/shell.rs` 里,跟 `ConsoleTransport` 解耦。

---

## 7. 共享 USART init (bootloader 去重)

`foc-common/src/lib.rs` 加:

```rust
//! USART bring-up helpers, shared between the bootloader and the app.
//! The bootloader can't use the HAL (no IRQ infrastructure) so it
//! drives the registers directly — same PAC code is reused in both
//! binaries via this function.

#[cfg(feature = "usart-init")]
pub mod usart_init;
```

`foc-common/src/usart_init.rs`:

```rust
use embassy_stm32::pac;

#[inline(never)]
pub fn init_usart2_pb3_pb4_921600() {
    // 现有 bootloader/src/main.rs::usart2_init() 的 PAC 代码
    // (RCC AHB2ENR GPIOBEN, GPIOB PB3/PB4 AF7, APB1ENR1 USART2EN,
    //  BRR=46, CR1=0x0D)
}

#[inline(never)]
pub fn init_usart1_pb6_pb7_921600() {
    // 对称:PB6 (TX, AF7) / PB7 (RX, AF7) on APB2 (USART1 is on APB2,
    // 不再是 APB1 — BRR 公式要重算,APB2 = 170 MHz / 1 = 170 MHz,
    // BRR = 170e6 / 921600 = 184.6 → 185)
}
```

`bootloader/Cargo.toml` 已经依赖 `foc-common = { ..., features = ["flash-driver"] }`,加 `usart-init` feature。

`bootloader/src/main.rs`:

```rust
// 之前:
// use embassy_stm32::pac::{GPIOB, RCC, USART2};
// fn usart2_init() { /* 60 行 PAC */ }

// 之后:
foc_common::usart_init::init_usart2_pb3_pb4_921600();
```

`bootloader/src/uart.rs` 保持原样(`uart_read_byte` / `uart_write_byte` / `uart_write_str` 是 y-modem 同步协议用的,跟 console 抽象无关)。

---

## 8. 迁移步骤(等真要换时执行)

### 8.1 重构到 trait(USART2 不变,只换分层)

- [ ] 新建 `src/drivers/console.rs`:`ConsoleRead` / `ConsoleWrite` / `ConsoleTransport` / `ConsoleWriter06`。
- [ ] 新建 `src/drivers/uart_console.rs`:`UartConsole` / `UartReader` / `UartWriter`。
- [ ] `src/tasks/shell.rs::shell_task` 改签名,移除 `TxWriter06` 本地定义,改用 `ConsoleWriter06`。
- [ ] `src/main.rs`:`shell_task(handles.debug_uart.into_inner().split())` → `shell_task(UartConsole::new(tx, rx))`。
- [ ] `src/drivers/debug_uart.rs` 收窄:只保留 `UsartError06` 定义(或挪到 `console.rs`)。
- [ ] `foc-common/src/usart_init.rs` 新建,把 `bootloader/src/main.rs::usart2_init` 内容搬过去。
- [ ] `bootloader/src/main.rs` 调 `foc_common::usart_init::init_usart2_pb3_pb4_921600()`。
- [ ] 验证:`cargo build`,`spin 5 1.0` 在原 USART2 路径上行为不变。

### 8.2 换 USART1(PB6/PB7) (在 8.1 完成后)

- [ ] `foc-common/src/usart_init.rs` 加 `init_usart1_pb6_pb7_921600`。
- [ ] `src/bsp.rs` 改 4 行:`p.USART2 → p.USART1`,`p.PB3 → p.PB6`,`p.PB4 → p.PB7`,IRQ binding 换 USART1。
- [ ] `bootloader/src/main.rs` 调 `init_usart1_*` 替代 `init_usart2_*`。
- [ ] 验证:PB6/PB7 上电有 921600 串口输出。

### 8.3 换 CAN (在 8.1 完成后,可独立做)

- [ ] `src/drivers/can_console.rs` 新建:`CanConsole` 包 `embassy_stm32::can::Can<'d, CAN1>`,内部 SLIP 编码(20 行 encoder + 30 行 decoder)。
- [ ] `src/bsp.rs` 追加 CAN1 + TX/RX pin init(走 embassy HAL,不是 PAC)。
- [ ] `src/main.rs` 改:`shell_task(CanConsole::new(can1, ...))`。
- [ ] `foc-common/src/usart_init.rs` 留着但 bootloader 不调它(bootloader 仍然走 y-modem over USART2,换 CAN 是 app 端的事)。
- [ ] 验证:CAN analyzer 上看 SLIP 解码后的 ASCII 字节流。

---

## 9. 这次不动的东西

- **Bootloader 走 CAN / 走 USB:** y-modem 协议本身假设 byte stream,CAN 上面要重新设计大块传输协议,远超本 spec 范围。
- **GAT(`type Reader<'a>`):** 当前 owned split 够用,zero-copy borrow 是优化项,不是设计前置需求。
- **`embedded-cli` 升级到 0.3+ 吃 v0.7:** 那会让 `ConsoleWriter06` 整段消失,但要等上游版本。
- **`Uart2Sink` 重命名:** 等真做 8.1 时一起改,不单独立项。

---

## 10. 验证 / 决策点

- 是否同意 owned split(不引入 GAT)? **倾向 yes**:符合当前 driver API,代码简单,优化点不是瓶颈。
- USART2 init 是不是该挪 `foc-common`? **倾向 yes**:bootloader 已经依赖 foc-common,加一个 feature flag 即可,消除"bootloader 和 app 重复写 USART 配置"的硬味道。
- `ConsoleWrite::write_str` 接 `&str` 而不是 `&[u8]`? **倾向 yes**:shell 输出全 ASCII;CAN 切二进制时再加 `write_bytes` 或新 trait,不阻塞这次设计。
- 文档里 "ConsoleRead::read_byte 返回 `Result<Option<u8>>`" 的 `None` 语义? **倾向 ok**:USB unplug / CAN bus-off 都是 `Ok(None)`,transient 噪声是 `Err(_)`。比 `bool` 标记位干净。

如果上面四点都同意,8.1 就可以开工了。等真要换 USART1 或上 CAN 时执行。
