# 设计:CAN-based OTA + CANopen + UDS (替换 y-modem bootloader)

**日期:** 2026-07-02
**状态:** 设计稿(待确认)
**目标:** 干掉 y-modem bootloader,把所有 OTA 走 FDCAN1 (PB9 TX, PA11 RX);加简单 CANopen + UDS。
**预估:** 多 phase 交付,每 phase 1-3 个 commit。本 spec 写完整设计,先确认再开工。

---

## 0. 量级警告(先读)

这个 spec 覆盖**多周**的工作。预估代码量:

| Phase | 内容 | 代码量 | 验证方式 |
|---|---|---|---|
| 1 | 删旧 OTA + 加 FDCAN1 + CanConsole 走 console | ~600 行删 + ~300 行加 | 编译 + 串口/CAN analyzer |
| 2 | CANopen (NMT + SDO + heartbeat) | ~800 行 | CANopen 主站/cfuzz 测试 |
| 3 | UDS (session/DID/ECU reset) | ~500 行 | UDS 模拟器 |
| 4 | UDS TransferData OTA 流 | ~700 行 | 端到端烧录验证 |

每次提交都按 phase 走,不要跳。

---

## 1. 4 个必须先定的决策

### 决策 1: 留不留 bootloader 存根?

**问题:** 单 bank OTA (没有 scratch 槽) 没法在 128KB flash 里同时放下"正在运行的 app"和"待写入的新 image"。

**选项:**
- **A. 完全干掉 bootloader (单 bank 写 in-place)**
  - App 从 `0x0800_0000` 开始,占 124 KB;metadata 占最后 2 KB;中间 2 KB 保留
  - OTA 边下载边写:新 image 直接覆盖 app slot 的对应地址
  - 当前 PC 在 4 KB 区间内(最近 4 KB 栈使用 + 当前函数 + 嵌套调用),写入别的 4 KB 区间不会撞
  - 风险:写失败的 page + 当前 PC 同一行 8 字节 = brick(极小概率)
  - **不推荐**:线上无备份,生产不可接受

- **B. 留 4 KB bootloader 存根 (双 bank 但用 stub 做 swap)**
  - Bootloader 在 `0x0800_0000`,4 KB
  - App 在 `0x0800_1000`,120 KB
  - OTA 下载到 app slot,假设旧 app 的 bootloader 部分还在运行(bootloader 不参与 OTA)
  - Bootloader 的活儿:启动时检查"新 image 已写入且 CRC OK" flag,如果有,跳到 app(它直接跳,不做 swap)
  - 实际 swap 由 app 自己完成:app 在 RAM 里跑"swap" routine,把新 image 字节搬到 app slot
  - **推荐**:4 KB bootloader 几乎零成本,换来"下载中掉电 = 旧 image 仍在"的安全属性

- **C. bootloader 做 swap (传统)**
  - Bootloader 8 KB
  - App 116 KB
  - Scratch 0 KB(下载到 running slot,in-place 写)
  - **不推荐**:bootloader 一旦烧坏无救

**推荐选 B。** App slot 120 KB,bootloader 4 KB,剩 4 KB 给 metadata + future 用。

### 决策 2: 简单 CANopen 的范围?

**CiA 301 全集** ~400 页。"简单" 意味着子集:

**必选(v1):**
- NMT 状态机:`Initializing → Pre-operational → Operational`,`Stopped`
- NMT boot-up message(COB-ID `0x700 + NodeId`)
- Heartbeat producer(COB-ID `0x700 + NodeId`,周期可配,默认 1 s)
- NMT command consumer(COB-ID `0x000`,master 发)
- SDO server(expedited + segmented transfer)
- 最小 Object Dictionary:`0x1000` DeviceType, `0x1001` ErrorRegister, `0x1017` HeartbeatProducerTime, `0x1018` Identity, `0x2000-0x2FFF` manufacturer area

**可选(v2):**
- TPDO 周期性广播 status
- SYNC producer / consumer
- EMCY

**不选(v1 不做):**
- RPDO(消费 master 来的命令)
- LSS(Node ID 拨码)

**默认 NodeId: 1**(hardcode,后续可改)

### 决策 3: 简单 UDS 的范围?

**ISO 14229 全集** 几百页。子集:

**必选(v1):**
- `0x10` DiagnosticSessionControl(Default / Programming)
- `0x11` ECUReset(HardReset)
- `0x22` ReadDataByIdentifier
- `0x2E` WriteDataByIdentifier
- `0x3E` TesterPresent
- `0x14` ClearDiagnosticInformation
- `0x19` ReadDTCInformation(只 subfunc 0x02 reportDTCByStatusMask)
- `0x27` SecurityAccess(seed/key,简单固定 key 即可)

**OTA 专需(Phase 4):**
- `0x34` RequestDownload
- `0x36` TransferData
- `0x37` RequestTransferExit

**不选:**
- `0x31` RoutineControl
- `0x2A` ReadDataByPeriodicIdentifier
- `0x23` ReadMemoryByAddress / `0x3D` WriteMemoryByAddress(用 0x34/0x36 替代)

**Negative response codes**:`0x12` SubFunctionNotSupported, `0x13` IncorrectMessageLengthOrInvalidFormat, `0x14` ResponseTooLong, `0x22` ConditionsNotCorrect, `0x31` RequestOutOfRange, `0x33` SecurityAccessDenied, `0x72` GeneralProgrammingFailure。

### 决策 4: UDS transport 走 CANopen 还是直接 CAN?

UDS 通常不依赖 CANopen,直接走 ISO-TP (ISO 15765-2) over CAN。两种选择:

- **A. CAN-TP 独立** — UDS 服务独立,跟 CANopen 共存于 FDCAN1,UDS 用独立 COB-ID(比如 `0x7DF` / `0x7E0-0x7E7`)
- **B. 走 CANopen SDO** — SDO 协议跟 UDS 服务有重叠(indexed 读/写 vs DID 读/写),但语义不完全一致,不推荐混用

**推荐 A**:UDS 走独立 ID + ISO-TP 帧,跟 CANopen 互不干扰。两个协议栈在 driver 层之上互不感知。

---

## 2. 硬件:FDCAN1 接线 + 时钟

**Pin:**
- TX = PB9 (AF9 for FDCAN1)
- RX = PA11 (AF9 for FDCAN1)

**Clock:** FDCAN1 在 APB1。`apb1_pre = DIV4` → APB1 = 42.5 MHz。FDCAN 时钟源通常用 HSE(更稳),不分频直接给 FDCAN 内核。`bit_timing` 算:
- 500 kbps classic CAN: `bit_timing = 42_500_000 / 500_000 = 85` clocks/bit,1 bit = 85 tq,seg1 = 60, seg2 = 25(示例)
- 1 Mbps classic CAN: BT = 42.5
- CAN-FD 1 Mbps nominal / 5 Mbps data: 更复杂,Phase 1 不开

**Bit timing** 按 500 kbps 起步(工业 CAN 通用,稳健)。后续可调到 1 Mbps。

**Acceptance filter:** 全部 accept(开发期) → 后续收紧,只收 master ID 范围的帧。

---

## 3. Flash 布局(决策 1 选 B)

```
0x0800_0000  ┌────────────────────┐
             │  Bootloader stub   │  4 KB
             │  - clock / GPIO    │
             │  - check flag      │
             │  - jump to app     │
0x0800_1000  ├────────────────────┤
             │                    │
             │  App slot          │  120 KB
             │  (FOC firmware)    │
             │                    │
             │                    │
0x0801_F000  ├────────────────────┤
             │  OTA flag (1 B)    │  0x0801_F000 烧 Pending 时进入 swap routine
             │  Metadata (2 KB)   │  image_crc32 / version / size
0x0801_F800  └────────────────────┘
0x0801_F800 + 2 KB = end
```

**OTA 流程:**
1. Master 通过 UDS `0x34` RequestDownload 指定 image 大小,NodeId 1 应答 `0x74` 同意,block size = 4096。
2. Master 通过 UDS `0x36` TransferData 多次发 4096-byte block + 序号。
3. NodeId 1 把 block 写到 app slot 对应地址(单 bank in-place,bootloader stub 不动)。
4. 全部 block 写完后,master 发 `0x37` RequestTransferExit,NodeId 1 计算 CRC32,写 metadata,设置 OTA flag,发 `0x77` ack。
5. Master 发 `0x11 0x01` ECUReset(HardReset)。
6. NodeId 1 触发 NVIC system reset。
7. 启动 → bootloader stub 看 OTA flag → 跳到 app slot 起始。
8. (第一版不做 swap routine:假设下载成功 + CRC OK,直接跳到 app slot;新 image 是完整有效 firmware。)

**风险 & 缓解:**
- 下载中掉电:旧 image 的 bootloader stub 部分没动,下次启动 stub 检测 OTA flag 未置位(因为 metadata 在最后 2 KB 还没写),直接跳到旧 app slot = 旧 image 继续跑。✓
- 下载中写错地址(在 bootloader 4 KB 内):UDS handler 自己要做范围检查,拒绝写 `[0x0800_0000, 0x0800_1000)`。✓
- metadata 写失败(掉电):bootloader 检测到 flag 但 CRC 不匹配,fall back 跳到旧 image。✓(需要 stub 实现 fallback)
- stub 自身被破坏:无法恢复,需 SWD 重烧(任何方案的共同风险)。

**Stub 工作量:** ~80 行 C-like Rust(时钟/去 flash lock/查 flag/jump)。

---

## 4. CANopen profile

**Bit timing:** 500 kbps classic CAN(可调到 1 Mbps)。

**NodeId:** 1(hardcode,后续加 LSS)。

**COB-ID 分配:**

| COB-ID | 功能 | 方向 | 备注 |
|---|---|---|---|
| `0x000` | NMT command | master → slave | CiA 301 标准 |
| `0x080 + NodeId` | SYNC | producer / consumer | v1 不实现 |
| `0x180 + NodeId` | TPDO1 | slave → master | v1 不实现 |
| `0x580 + NodeId` | SDO server receive | master → slave | 标准 SDO |
| `0x600 + NodeId` | SDO server transmit | slave → master | |
| `0x700 + NodeId` | Heartbeat / boot-up | slave → master | 1 Hz |

SDO 是 indexing access 协议,client(slave 在我们的角色配置下) 读/写 master 来的对象。

**Object Dictionary (v1):**
| Index | Sub | Name | Type | Access | Reset |
|---|---|---|---|---|---|
| 0x1000 | 0 | DeviceType | u32 | RO | 0 |
| 0x1001 | 0 | ErrorRegister | u8 | RO | 0 |
| 0x1017 | 0 | HeartbeatProducerTime | u16 | RW | 1000(ms) |
| 0x1018 | 0 | Identity.VendorId | u32 | RO | 0x1234 (placeholder) |
| 0x1018 | 1 | Identity.ProductCode | u32 | RO | 0x5678 (placeholder) |
| 0x1018 | 2 | Identity.Revision | u32 | RO | 0x0001 |
| 0x1018 | 3 | Identity.Serial | u32 | RO | chip unique ID |
| 0x2000 | 0 | Vendor.heartbeat_active | u8 | RW | 1 |

**NMT state transitions:**
- 上电 → boot-up(发 `0x700` 帧) → Pre-operational
- Master 发 `0x000 0x01 0x00`(Start remote node) → Operational
- Master 发 `0x000 0x80 0x00`(Enter Pre-operational) → Pre-operational
- Master 发 `0x000 0x02 0x00`(Stop remote node) → Stopped
- Operational:SDO 可用。TPDO/RPDO 周期发送(本设计 v1 不实现 TPDO,所以 operational 跟 pre-operational 在 v1 行为一致,只是 SDO 仍可用)

**SDO transfer:**
- Expedited (≤ 4 bytes):`CCS=1, S=1, e=1, n=3`,一次往返
- Segmented (>4 bytes):`CCS=1, S=1, e=0, n=0`,client 发起 read;server 应答 `CCS=0, S=1`,带第一段;client 后续发 `CCS=0, S=0`(ack);循环直到 server `CCS=0, S=1, e=1`(最后一段带 toggle)
- Write 流程对称(CCS=0 init, CCS=1 response)

---

## 5. UDS profile(ISO 14229-3 over CAN-TP)

**COB-ID 分配:**
| COB-ID | 功能 |
|---|---|
| `0x7DF` | UDS Functional broadcast(master 测所有 ECU,v1 不响应,只忽略) |
| `0x7E0` | UDS Physical request(测试 master → node 1) |
| `0x7E8` | UDS Physical response(node 1 → master) |

**ISO-TP (ISO 15765-2) 单帧 / 多帧分片:**

| 类型 | PCI byte 0 | 描述 |
|---|---|---|
| Single Frame (SF) | `0x00, len` | len ≤ 7,1 帧装下整个 UDS 服务 |
| First Frame (FF) | `0x10, len_hi, len_lo` | 总长 > 7,首帧带 6 字节数据 |
| Consecutive Frame (CF) | `0x20 + sn, ...` | 后续帧,sn 0-15 循环 |
| Flow Control (FC) | `0x30, BS, STmin` | receiver 发,控制发送节奏 |

**v1 简化:**
- 节点只支持 Single Frame(短 UDS 消息 ≤ 7 字节;超过就回 `0x14` ResponseTooLong)
- 多帧 ISO-TP 在 v2 加(需要的时候)
- 这样 Phase 3 的 UDS 服务可以做得**很轻**

**支持的 UDS 服务 (v1,SF only):**

| SID | 服务 | 实现 |
|---|---|---|
| `0x10` | DiagnosticSessionControl | DefaultSession + ProgrammingSession |
| `0x11` | ECUReset | HardReset(触发 NVIC system reset) |
| `0x14` | ClearDiagnosticInformation | ack |
| `0x19` | ReadDTCInformation | subfunc 0x02 reportDTCByStatusMask: 永远回 0 DTCs |
| `0x22` | ReadDataByIdentifier | 支持 DIDs: `0xF186` ActiveDiagSession, `0xF190` VIN(= firmware version) |
| `0x2E` | WriteDataByIdentifier | 支持 DIDs: 同上 |
| `0x27` | SecurityAccess | seed=0xA5A5A5A5, key=seed+0x1234(明文,生产换 HMAC) |
| `0x3E` | TesterPresent | subfunc 0x00,回 `0x7E 0x00` |

**OTA 服务 (Phase 4,需要 ISO-TP 多帧):**
- `0x34 0x00 ...`:RequestDownload,memory address(写在哪个 flash 地址)+ size + format(无压缩)
- `0x36 0x01 ...`:TransferData,block sequence counter + data
- `0x37`:RequestTransferExit,清空 transfer state,触发 metadata 写

---

## 6. 文件级变更

### Phase 1(本次可完成)

**删除:**
- `bootloader/` 整目录
- `Cargo.toml` workspace `members` 里的 `bootloader`
- `Cargo.toml` `foc-common` 的 `flash-driver` feature(没人用了)
- `common/src/flash.rs` 全部内容
- `common/src/flag.rs` 全部内容
- `common/src/addresses.rs`(`APP_*_ADDRESS` 没人用了,后续 OTA 重新设计地址)
- `src/drivers/flash.rs`(本目录也有一个)
- `src/commands/ota.rs` 和 `commands/mod.rs` 里的 `pub mod ota;`
- `src/commands/shell.rs` 里的 `OtaUpdate` variant

**新增:**
- `src/drivers/can_console.rs`:`CanConsole: ConsoleTransport` 实现,跑 FDCAN1
- `src/drivers/can.rs`:FDCAN1 + 接受滤波器配置 helper
- (按 `console-transport-design.md` 一起做的话) `src/drivers/console.rs`,`src/drivers/uart_console.rs`

**修改:**
- `src/bsp.rs`:去掉 BufferedUart + TX/RX 缓冲区 + USART2 IRQ;加 FDCAN1 配置 + PB9/PA11 替代品;`BoardHandles` 改为只包含 motor_pwm
- `src/main.rs`:把 USART2 split → `UartConsole::new(tx, rx)` 换成 `CanConsole::new(can)`;删 USART2 IRQ bind;删 BufferedUart split
- `src/tasks/shell.rs`:依然 `shell_task<T: ConsoleTransport>(transport: T)`,签名不变;只改 `TxWriter06` 引用到 `ConsoleWriter06`
- `src/tasks/mod.rs`:去掉 heartbeat 之外的 task 不变(heartbeat 仍然 defmt-only 即可)
- `src/commands/shell.rs`:删 `OtaUpdate` variant + 处理;新增 `nmt`, `sdo`, `uds` 命令(Phase 2/3 用,Phase 1 先空)
- `src/metadata.rs`:删,换到 `src/ota/metadata.rs`
- `Cargo.toml`:workspace members 去掉 bootloader;`foc-common` dependencies 简化
- `foc-common/src/lib.rs`:大瘦身,只保留算法相关 type(目前是空的,不用动太多)
- `foc-common/Cargo.toml`:删 flash-driver, defmt-format 简化

**bootloader/ → foc-bootloader-stub/ 重命名(Phase 1 末):**
- 新建 `foc-bootloader-stub/` 4 KB crate
- 实现:clock init (复用 main.rs 的 170 MHz PLL 配置),看 OTA flag,fall-through 跳到 app
- 这次不实现完整 stub,只放 TODO stub 让 build 过

### Phase 2
- 新建 `src/canopen/`:`mod.rs`, `nmt.rs`, `sdo.rs`, `heartbeat.rs`, `od.rs`(object dictionary)
- `src/main.rs` 启动时 spawn 几个 task:NMT handler, SDO server, heartbeat producer

### Phase 3
- 新建 `src/uds/`:`mod.rs`, `services.rs`(每个 SID 一个函数)
- UDS frame dispatcher 在 FDCAN1 task 里(单帧 only,先简单)

### Phase 4
- 新建 `src/ota/`:`mod.rs`, `metadata.rs`, `transfer.rs`(0x34/0x36/0x37 state machine)
- `foc-bootloader-stub/` 完整实现(stub 跳到新 image,带 fallback 到旧 image)

---

## 7. Phase 1 详细执行计划

如果 4 个决策都确认选 B(bootloader stub 4 KB)+ CANopen 必选子集 + UDS 必选子集 + UDS 走独立 ISO-TP:

### 步骤

1. **先把 4 KB bootloader stub 写完 + 集成**:保证 link 完 app + stub 后能启动到 app。
2. **删旧 OTA**:git rm bootloader/, Cargo.toml 改 workspace,删 OTA 相关代码,删 foc-common flash-driver。
3. **加 FDCAN1 driver + CanConsole**:用 embassy `embassy_stm32::can::fdcan::Fdcan` 包成 `ConsoleTransport`。
4. **接线 main**:把 `UartConsole` 换成 `CanConsole`。
5. **跑通**:build → flash → 用 CAN analyzer(Saleae / PCAN)看 heartbeat + 收 NMT 命令。

### Phase 1 风险
- FDCAN1 在 embassy 0.6 的 API 可能跟我们看的略有不同(`FdcandConfig`、`filter` 设置等),需要实测
- CanConsole 的 SLIP/COBS framing:Phase 1 不需要(ConsoleTransport 假设 byte stream),FDCAN1 frame payload 本身就是 byte stream,直接转发即可
- ISO-TP 不在 Phase 1 范围

### Phase 1 测试手段
- 编译 + flash
- 串口 defmt 日志看 `FDCAN1 init ok, node id = 1`
- PCAN-View / can-utils `candump can0` 看 heartbeat
- `cansend can0 000#0100` 进 Operational
- `cansend can0 000#0200` 回 Pre-operational
- (Phase 1 不实现 SDO,UDS,所以只验证 NMT + heartbeat)

---

## 8. 关键决策点(请你拍板)

| # | 决策 | 我的推荐 | 替代 |
|---|---|---|---|
| 1 | bootloader 存根 | B: 4 KB stub,单 bank in-place 写 | A: 无 stub; C: 8 KB stub 做 swap |
| 2 | CANopen v1 | NMT + SDO + heartbeat;无 PDO/SYNC/EMCY | 加 TPDO; 加 SYNC |
| 3 | UDS v1 | session/DID/RW/TesterPresent/clear/reset/SecurityAccess | 加 RoutineControl |
| 4 | UDS transport | 独立 COB-ID + ISO-TP(v1 只 SF) | 走 CANopen SDO |

**如果这 4 个都同意,我开始 Phase 1。** 有不同意的,告诉我用哪一栏,或者说你想加什么 / 删什么。

---

## 9. Phase 1 后下一站

Phase 1 完成后:
- Phase 2 (CANopen) = 1 session 起步,1-2 commits
- Phase 3 (UDS) = 1 session,1-2 commits
- Phase 4 (OTA) = 1-2 sessions,因为涉及 metadata 持久化 + bootloader stub 完整实现

总预估 3-4 个 session,跟本 spec 一并维护。
