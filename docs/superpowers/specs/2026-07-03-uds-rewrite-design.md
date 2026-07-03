# Phase 5+ UDS 模块重构设计

**作者：** heng
**日期：** 2026-07-03
**状态：** 提议，待评审
**目标读者：** heng, Claude

---

## 1. 背景

### 1.1 当前实现

`foc-rust` 的 UDS 协议栈是 Phase 2-4 阶段在 src/can/uds.rs、src/can/sdo.rs、src/can/ota.rs 中手写的，共约 1700 行。功能上覆盖了 8 个 UDS 服务（`0x10`、`0x11`、`0x14`、`0x19`、`0x22`、`0x2E`、`0x27`、`0x3E`）以及 OTA 流程（`0x34`/`0x36`/`0x37`），共 **11 个 SID**（含 OTA）。

物理拓扑：**FDCAN1 上的 CANopen SDO 服务**（COB-ID `0x601` 收 / `0x581` 发），其中 OD 条目 `0x2F00.0` 被复用为 **UDS 网关**（写触发 UDS 请求、读返回 UDS 响应）。这个"SDO 隧道 UDS"的非标准但可工作的拓扑在 Phase 4 review 中已经被接受。

### 1.2 触发本次重构的原因

四个 audit 轮次累计修了 18 个 P0-class bug。Phase 1-3 的 bug 集中在 SDO/UDS **wire format** 层面（错误的字节位、错误的 SCS 码、缺失的 NRC 分支）；Phase 4 的 bug 集中在 **非 CAN 表面**（motor 控制、shell、bsp）。但当我们把目光放到 **UDS 协议本身的架构**层面时，还有几个根本性缺陷没解决：

1. **加新 DID 必须改 `uds.rs`**：当前 dispatcher 是硬编码 `match` arm，每个 DID 都是 `if did == X { ... }` 分支。ISO 14229 的标准 DID 集有几十个常用值（`0xF186` ActiveDiagSession、`0xF187` VehicleIdentificationNumber、`0xF188` SystemSupplierIdentifier、`0xF190` VIN、`0xF192` SystemSupplierECUHardwareVersionNumber 等），我们只支持其中 1 个。能加但每加一个就改一次 dispatcher 是反模式。

2. **无 0x78 ResponsePending 支持**：当 UDS 服务需要超过 P2 server time（典型 50 ms）才能响应时，**必须**返回 NRC `0x78 RequestCorrectlyReceivedResponsePending`，客户端应在 P2*（5 s）后再次轮询。OTA 烧写单帧 TransferData 数据量是 6 字节（CAN 8 字节 payload 减 2 字节 SID+seq），500 kbps CAN bus 上每帧约 144 µs，但 flash 写入 + flash 擦除（128 KB / 2 KB 页 = 64 页）每页 5-30 ms，单次 `0x34 RequestDownload` 配套的擦除可能耗 1-2 秒。如果实际产品接 P2 = 50 ms 的客户端（比如任何符合 ISO 14229 的诊断仪），早超时早放弃。reference（MiniUds）通过 `pending_queue` + `nrcRequestCorrectlyReceived_ResponsePending` 处理这类长任务；我们直接同步阻塞。

3. **固定 seed/key 是安全反模式**：`handle_security_access` 写死 `seed = 0xA5A5A5A5`、`key = 0xA5A5_B7D9`。固件是 open source（仓库就在 GitLab 上），任何持有镜像的人反汇编一下就拿到这俩常量。ISO 14229 没用 HSM 也能允许现场调试，但 "demo 凭证" 和 "生产凭证" 的区别必须是这个仓库能说清楚的。

4. **单一 SAL 等级**：`SECURITY` 是个 `AtomicU8` 状态，只能是 `0`（Locked）或 `1`（Unlocked）。ISO 14229 实践常用 3 级 SAL（reference MiniUds 也是 3 级），每级对应不同的诊断服务集（典型分层：SAL1 = 产线刷写、SAL2 = 4S 店诊断、SAL3 = 厂商后门）。我们只有 SAL1。

5. **session 切换无应用通知**：`handle_session_control` 改完 `SESSION` 原子就返回，应用代码（如果以后有诊断专属 UI 或日志门控）没法订阅"session 切到 Programming 了"这个事件。

6. **缺失两个常用 SID**：`0x28 CommunicationControl`（网络发送/接收开关，OTA 时用 "disable normal TX" 模式关掉心跳 + NMT ACK）和 `0x31 RoutineControl`（routine start/stop/result，OTA 流程的标准配套：erase、checkProgrammingDependencies、checkCRC）。

7. **缺失 5 个 NRC**：`0x7E` SubFunctionNotSupportedInActiveSession、`0x78` RequestCorrectlyReceivedResponsePending、`0x24` RequestSequenceError、`0x26` FailurePreventsExecutionOfRequestedAction、`0x7F` ServiceNotSupportedInActiveSession。当前用 `0x12` 顶所有"不支持"、用 `0x72` 顶所有"失败"，语义不精确。

8. **整体 reference 对比**：MiniUds（`~/work/xiaoyu_oem_pwrpilot/source/components/MiniUds/`，约 2000 行 C）是产品级的 UDS 实现，覆盖了我们上面列的所有点。代码风格完全适合作为我们重写的蓝本。

### 1.3 现状评估

能用。20 个烟雾测试场景全绿，wire format 4 轮 audit 后正确。但：

- **架构不可扩展**：加 DID 要改 dispatcher
- **不抗 real client**：固定 P2 timer 的诊断仪会超时
- **不抗攻击**：固定 seed/key
- **不符合标准**：缺 SID、NRC、机制

我判断当前实现是"Phase 4 的临时落地"，不是 Phase 5+ 的稳定形态。

---

## 2. 重构目标

### 2.1 必须达成（Phase 5a）

- [ ] **表驱动 DID 分发**：`config.rs::DidEntry` 数组 + dispatcher 在 `O(配置表长度)` 时间内查表
- [ ] **声明式 session/security 门控**：每个 DID entry 自带 `session_access` 位掩码 + `security_level`，dispatcher 统一检查
- [ ] **多 SAL 状态机**：SAL1/2/3、每级独立 `key_mask`、握手序列验证（`seed_sent` flag）
- [ ] **真实密钥派生**：`generate_key_from_seed(seed, mask)` 用 LFSR + 位反转，每 `RequestSeed` 用新的随机 seed
- [ ] **挂起队列 + NRC 0x78**：长任务（如 OTA）返回 `UdsResult::Pending`，dispatcher 排进队列，主循环后续 tick 自动发 `0x78 ResponsePending`，最终响应完成后再读
- [ ] **per-session 通知回调**：`config.rs::session_notify: Option<fn(Session)>`
- [ ] **完整 NRC 集合**：`0x10/0x11/0x12/0x13/0x14/0x21/0x22/0x24/0x25/0x26/0x31/0x33/0x34/0x35/0x36/0x37/0x70/0x71/0x72/0x73/0x78/0x7E/0x7F`

### 2.2 应当达成（Phase 5b）

- [ ] **`0x28 CommunicationControl`**：subfunc `0x03` disableNormalCommunication（OTA 期间关 TX）、subfunc `0x00` enableNormalCommunication。简化版（不实现 network type 字段）
- [ ] **`0x31 RoutineControl`**：subfunc `0x01` startRoutine、`0x02` stopRoutine、`0x03` requestRoutineResults。三张 routine 表（start/stop/result）
- [ ] **`0x3E TesterPresent` suppressPositiveResponse bit**：subfunc `0x80` 抑制正响应（master 用它保活而不想要响应）
- [ ] **`0x14 ClearDiagnosticInformation`** 与 **`0x19 ReadDTCInformation`** 拆到独立模块

### 2.3 不做（明确范围）

- **CAN TP 层（ISO 15765 / DoCAN）**：当前 SDO 分段能承载 ≤7 字节的 UDS payload，足够覆盖我们的所有服务（含 6 字节 seed 响应）。Phase 5+ 不上 TP 层。
- **KWP2000 兼容**：ISO 14229 已经覆盖我们的需求。
- **Bootloader 协议改动**：FDCAN1 OTA 链路保持现状。
- **多个 Network Interface**：当前只有 FDCAN1 一个，Phase 5+ 不加。

---

## 3. 参考实现分析：MiniUds

代码位置：`~/work/xiaoyu_oem_pwrpilot/source/components/MiniUds/`，约 2000 行 C。文件：

```
mini_uds.h          211 行  // 公共 API + 配置结构体声明
mini_uds_types.h    121 行  // SID / NRC / SAL 常量
mini_uds.c          461 行  // main_task / 状态机 / 收发
mini_uds_srv.c     1128 行  // 12 个 SID 的具体 handler
mini_uds_cfg.h       44 行  // 编译时配置（buffer size / log / PENDING timeout）
mini_uds_resp_code.h 54 行  // NRC 枚举
```

### 3.1 核心模式：表驱动 + 门控模板

```c
// 每个 DID 自带元数据 + 回调
typedef struct MINIUDS_SRV22_DIDLIST_STRU {
    uint16_t did;
    uint8_t  session_access;  // 位掩码：DEFAULT|PROGRAMMING|EXTEND
    uint8_t  security_level;  // 0=allpass, 1/2/3 = SAL1/2/3
    uint8_t (*func)(uint8_t *, uint32_t *);
} miniuds_srv22list_t;

// 每个 Routine 同上
typedef struct MINIUDS_SRV31_DIDLIST_STRU {
    uint16_t rid;
    uint8_t  session_access;
    uint8_t  security_level;
    uint8_t (*func)(uint8_t, uint8_t *, uint8_t *, uint32_t *);
} miniuds_srv31list_t;

// 配置结构体：所有表 + 回调
typedef struct MINIUDS_CONFIG_STRU {
    uint8_t *request_buf;
    uint32_t request_bufsize;
    uint8_t *response_buf;
    uint32_t response_bufsize;

    void (**pending_queue)(void *);
    uint32_t pending_queue_size;

    void (*send_data)(const uint8_t *, uint32_t);
    uint32_t (*get_curtick)(void);

    struct {
        void (*default_notify)(void);
        void (*programming_notify)(void);
        void (*extend_notify)(void);
    } srv10;

    struct { ... } srv11;     // SoftReset + HardReset

    struct { uint32_t num; const miniuds_srv22list_t *list; } srv22;
    struct { uint32_t (*get_random_seed)(void); } srv27;
    struct { uint32_t num; const miniuds_srv2elist_t *list; } srv2e;
    struct { ... start/stop/result ... } srv31;

    struct { ... } srv34;
    struct { ... } srv36;
    struct { ... } srv37;
} udscfg_t;
```

### 3.2 状态机：IDLE → PARSING → {IDLE, PENDING}

两条路径：

- **同步路径**：`IDLE → PARSING → IDLE`，handler 直接在 dispatch 同步完成（如 0x22 ReadDID、0x27 错误 key）
- **异步路径**：`IDLE → PARSING → PENDING → ... → PENDING → IDLE`，handler 把续延函数推进 pending queue，state 保持 PENDING；后续 tick 跑完最后一个续延函数后回 IDLE

```c
typedef enum MINIUDS_SERVICE_STATE_ENUM {
    UDS_SERVICE_IDLE = 0,      // 可以收新请求
    UDS_SERVICE_PARSING,       // 正在解析 / 处理
    UDS_SERVICE_PENDING,       // 长任务挂起，等 pending queue
} udssrv_st;

typedef enum MINIUDS_SENDER_STATE_ENUM {
    UDS_SENDER_IDLE = 0,       // 静默
    UDS_SENDER_WAIT,           // 有响应待发
} udssender_st;
```

`main_task` 主循环：

```c
void mini_uds_main_task(udscb_t *udscb) {
    mini_uds_do_pending(udscb);    // 1. 处理挂起队列
    mini_uds_sender_process(udscb); // 2. 发响应（如果有）
    mini_uds_do_services(udscb);    // 3. 解析 + 分发新请求
    mini_uds_sender_process(udscb); // 4. 再发一次（处理可能产生响应）
}
```

`do_services` 切换 state 到 PARSING，调具体 SID handler。Handler 可以直接 `send_positive/negative`，或者返回 `UDS_E_PENDING` —— 后者会把一个 "续延函数" 推进 pending_queue，state 保持 PENDING 不接受新请求。

### 3.3 挂起队列 + 0x78 自动续延

```c
bool mini_uds_push_pending_service(udscb_t *udscb, void (*func_addr)(void *)) {
    if (UDS_SERVICE_IDLE == udscb->srv_state) {
        return true;  // 没用 pending，直接 sync 完成
    }
    for (idx = 0; idx < pending_queue_size; idx++) {
        if (NULL == pending_queue[idx]) {
            pending_queue[idx] = func_addr;
            return true;
        }
    }
    return false;  // 满
}
```

`mini_uds_sender_process` 检测 elapsed > P2_server（默认 `32000000` ticks = 1 s）自动发 `0x78`：

```c
static void mini_uds_sender_process(udscb_t *udscb) {
    if (UDS_SENDER_WAIT == udscb->sender_state) {
        udscb->sender_state = UDS_SENDER_IDLE;
        udscb->srv_state = UDS_SERVICE_IDLE;
        send_data(response_buf, response_len);   // 发正/负响应
    } else if (UDS_SERVICE_IDLE != udscb->srv_state) {
        // 还在 PENDING 中，超时了就发 0x78
        if (elapsed > PENDING_TIME_TICKS) {
            send_data([0x7F, sid, 0x78], 3);  // 0x78 ResponsePending
        }
    }
}
```

效果：client 收到 `0x78` 后等 `P2*`（5 s）再来轮询；主循环在 client 等待期间继续 tick 挂起队列里的续延函数。

### 3.4 密钥派生：位旋转 LFSR + 反字节重组

```c
static uint8_t bit_change(uint8_t sec_val) {
    // 8-bit bit reversal
    sec_val = (sec_val & 0xaa) >> 1) | (sec_val & 0x55) << 1;
    sec_val = (sec_val & 0xcc) >> 2) | (sec_val & 0x33) << 2;
    sec_val = (sec_val & 0xf0) >> 4) | (sec_val & 0x0f) << 4;
    return sec_val;
}

static uint32_t generate_key_from_seed(uint32_t seed, uint32_t mask) {
    uint32_t seed_local = seed;
    for (int i = 0; i < 40; i++) {
        if (seed_local & 0x80000000) {
            seed_local = (seed_local << 1) ^ mask;
        } else {
            seed_local = seed_local << 1;
        }
    }
    uint32_t key = 0;
    for (int i = 0; i < 4; i++) {
        key |= bit_change(seed_local >> ((3 - i) << 3)) << (i << 3);
    }
    return key;
}
```

`mask` 是配置参数（典型 `0x30002212`），不同型号可不同。Seed 来自 `get_random_seed` 回调，每次 RequestSeed 都取新随机值。

### 3.5 双 SAL 协议：seed_sent 跟踪

```c
case UDS_SECU_ACC_REQ_SEED_SAL1:
    udscb->random_seed = udscb->ops->srv27.get_random_seed();
    udscb->seed_sent = true;
    // 返回 4 字节 seed
    
case UDS_SECU_ACC_SUB_KEY_SAL1:
    if (true != udscb->seed_sent) {
        nrc = nrcRequestSequenceError;  // 0x24
        break;
    }
    udscb->seed_sent = false;
    if (rx_key == generate_key_from_seed(udscb->random_seed, mask)) {
        // 解锁成功
    }
```

`seed_sent` flag 强制 `RequestSeed → SendKey` 顺序。如果 master 先发 SendKey 就 0x24。

### 3.6 session 切换 + 通知回调

```c
case 0x02:  // ProgrammingSession
    mini_uds_set_cur_session(udscb, UDS_SRV_PROGRAMMING_SESSION);
    if (NULL != udscb->ops->srv10.programming_notify) {
        udscb->ops->srv10.programming_notify();  // 应用代码钩子
    }
    break;
```

应用代码（main.rs 或某个专门的 diagnostics 模块）注册三个回调，session 切到对应状态时触发。

### 3.7 通用 NRC 模板

每个 SID handler 末尾都跑同一段：

```c
// 检查 session_access（如果配置了）
if (0u == (session_access & (1u << mini_uds_get_cur_session(udscb)))) {
    nrc_code = nrcSubFunctionNotSupportedInActiveSession;  // 0x7E
    break;
}

// 检查 security_level（如果配置了）
if (security_level > mini_uds_get_security_access_level(udscb)) {
    nrc_code = nrcSecurityAccessDenied;  // 0x33
    break;
}
```

`mini_uds_send_resp` 根据 `nrc_code` 自动选发正响应或负响应。如果是 `0x78`（pending），不发任何响应（等续延函数完成后再发）。

---

## 4. 提议架构

### 4.1 模块拆分

```
src/can/uds/
├── mod.rs           // 公共 API + dispatch 主循环 + 0x2F00 网关接入
├── state.rs         // Session, SecurityLevel, SrvState, PendingJob
├── config.rs        // UdsConfig, ServiceEntry, DidEntry, RoutineEntry
├── security.rs      // 0x27 SecurityAccess + LFSR 密钥派生
├── session.rs       // 0x10 DiagnosticSessionControl + 通知回调
├── reset.rs         // 0x11 ECUReset
├── dtc.rs           // 0x14 ClearDiagnosticInformation + 0x19 ReadDTCInformation
├── read_data.rs     // 0x22 ReadDataByIdentifier + DID 表查找
├── write_data.rs    // 0x2E WriteDataByIdentifier + DID 表查找
├── routine.rs       // 0x31 RoutineControl (start/stop/result 表)
├── comm_control.rs  // 0x28 CommunicationControl (TX/RX 开关)
├── download.rs      // 0x34/0x36/0x37 RequestDownload/TransferData/TransferExit
├── tester_present.rs// 0x3E TesterPresent (含 suppressPositiveResponse)
├── nrc.rs           // Nrc enum + Nrc::code() -> u8
└── pending.rs       // PendingQueue + 0x78 ResponsePending 自动续延

src/can/uds_config.rs // 静态配置：把所有表项写在这里
```

旧 `src/can/uds.rs` 拆掉，`src/can/sdo.rs` 保留但增加对 `uds::dispatch` 的新签名调用。

### 4.2 配置结构体

```rust
// src/can/uds/config.rs

use crate::can::uds::nrc::Nrc;
use crate::can::uds::state::{Session, SecurityLevel, SrvState};

/// 主服务表条目。一个条目对应一个 SID（含 0x31 的三个 subfunc）。
pub struct ServiceEntry {
    pub sid: u8,
    /// 位掩码：bit 0 = Default session, bit 1 = Programming, bit 2 = Extended
    pub session_access: u8,
    /// 最低 SAL：0 = allpass, 1/2/3 = SAL1/2/3
    pub security_level: u8,
    pub handler: ServiceHandler,
}

pub enum ServiceHandler {
    Session,         // 0x10
    EcuReset,        // 0x11
    ClearDtc,        // 0x14
    ReadDtc,         // 0x19
    ReadDataById,    // 0x22  (具体 DID 在 read_dids 表)
    WriteDataById,   // 0x2E
    CommControl,     // 0x28
    SecurityAccess,  // 0x27
    RoutineStart,    // 0x31 sub=0x01
    RoutineStop,     // 0x31 sub=0x02
    RoutineResult,   // 0x31 sub=0x03
    RequestDownload, // 0x34
    TransferData,    // 0x36
    TransferExit,    // 0x37
    TesterPresent,   // 0x3E
}

/// ReadDataByIdentifier (0x22) DID 表
///
/// `func` 是 `Box<dyn FnMut>` 而**不是** `fn` 指针。原因：fn 指针不能捕获
/// 环境（`static` 函数可以，但需要 flash-resident 状态查询的 handler
/// 比如读 active session / 读 flash 镜像版本等，需要 closure 捕获引用）。
/// `FnMut` 不需要 dyn-safe trait（只有 FnOnce 不行），`Box<dyn FnMut>` 合法。
pub struct DidReadEntry {
    pub did: u16,
    pub session_access: u8,
    pub security_level: u8,
    /// 调用者把响应数据写到 `out`，返回写入字节数
    pub func: Box<dyn FnMut(&mut [u8]) -> Result<usize, Nrc>>,
}

/// WriteDataByIdentifier (0x2E) DID 表
pub struct DidWriteEntry {
    pub did: u16,
    pub session_access: u8,
    pub security_level: u8,
    pub func: Box<dyn FnMut(&[u8]) -> Result<(), Nrc>>,
}

/// RoutineControl (0x31) 三张表
pub struct RoutineEntry {
    pub rid: u16,
    pub session_access: u8,
    pub security_level: u8,
    /// startRoutine / stopRoutine / requestRoutineResults 共用签名：
    /// 输入 payload, 输出响应数据 + 长度
    pub func: Box<dyn FnMut(&[u8], &mut [u8]) -> Result<usize, Nrc>>,
}

/// Pending queue 里的续延任务。
///
/// `func` 是 `Box<dyn FnMut>` 而不是函数指针（fn pointer 不能捕获环境，
/// 但 download handler 需要 `move |ctx| { let block = [...]; ... }` 抓
/// TransferData 的 6 字节 payload）。`FnOnce` 不能存为 dyn（FnOnce 不是
/// dyn-safe trait object），所以选 `FnMut`。
///
/// `complete` 标志续延函数是否单次完成，由 UdsContext 的 `complete` 字段
/// 传递到 `tick`，tick 据此决定是否把 job 放回 queue。
pub struct PendingJob {
    pub func: Box<dyn FnMut(&mut UdsContext)>,
}

/// 全局配置（在 src/can/uds_config.rs 里实例化）
pub struct UdsConfig {
    pub services: &'static [ServiceEntry],
    pub read_dids: &'static [DidReadEntry],
    pub write_dids: &'static [DidWriteEntry],
    pub routines_start: &'static [RoutineEntry],
    pub routines_stop: &'static [RoutineEntry],
    pub routines_result: &'static [RoutineEntry],

    pub request_buf: &'static mut [u8],
    pub response_buf: &'static mut [u8],
    pub pending_queue: &'static mut [Option<PendingJob>],

    /// Session 切换通知回调
    pub on_session_enter: Option<fn(Session)>,
    pub on_session_exit: Option<fn(Session)>,

    /// SecurityAccess 随机 seed 源（硬件 RNG / PRNG / 时间混合）
    pub random_seed: fn() -> u32,
    /// 密钥派生掩码（每个 SAL 级一个）
    pub key_masks: [u32; 3],

    /// P2 server timer：客户端等这么久后会发 0x78
    pub p2_server_ms: u32,
    /// P2* extended：超过这个时间 client 应该放弃
    pub p2_star_ms: u32,

    /// 计时器源（embassy-time millis 等价物）
    pub tick_ms: fn() -> u32,
}
```

### 4.3 状态机

```rust
// src/can/uds/state.rs

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Session {
    Default = 0,
    Programming = 1,
    Extended = 2,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum SecurityLevel {
    Locked = 0,
    Sal1 = 1,
    Sal2 = 2,
    Sal3 = 3,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SrvState {
    Idle,
    Parsing,
    Pending,
}

/// UDS 引擎所有运行时状态。RAM-resident（在 .data section）。
#[link_section = ".data"]
pub struct UdsState {
    pub session: Session,
    pub security: SecurityLevel,
    pub seed_sent: bool,
    pub current_seed: u32,

    pub state: SrvState,
    pub request_len: usize,
    pub response_len: usize,
    /// response_buf 里有可发的响应（同步完成 OR 0x78 OR 续延完成）。
    /// caller 通过 `take_response()` 拿走并自动清零。
    pub response_pending: bool,
    pub request_tick_ms: u32,

    /// OTA 状态
    pub download_active: bool,
    pub transfer_sn: u8,

    /// CommunicationControl 状态
    pub tx_disabled: bool,
    pub rx_disabled: bool,
}
```

### 4.4 NRC 集合

```rust
// src/can/uds/nrc.rs

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Nrc {
    GeneralReject                            = 0x10,
    ServiceNotSupported                      = 0x11,
    SubFunctionNotSupported                  = 0x12,
    IncorrectMessageLengthOrInvalidFormat    = 0x13,
    ResponseTooLong                          = 0x14,
    BusyRepeatRequest                        = 0x21,
    ConditionsNotCorrect                     = 0x22,
    RequestSequenceError                     = 0x24,
    NoResponseFromSubnetComponent            = 0x25,
    FailurePreventsExecutionOfRequestedAction = 0x26,
    RequestOutOfRange                        = 0x31,
    SecurityAccessDenied                     = 0x33,
    AuthenticationRequired                   = 0x34,
    InvalidKey                               = 0x35,
    ExceededNumberOfAttempts                 = 0x36,
    RequiredTimeDelayNotExpired              = 0x37,
    UploadDownloadNotAccepted                = 0x70,
    TransferDataSuspended                    = 0x71,
    GeneralProgrammingFailure                = 0x72,
    WrongBlockSequenceNumber                 = 0x73,
    IllegalByteCountInBlockTransfer          = 0x75,
    RequestCorrectlyReceivedResponsePending = 0x78,
    SubFunctionNotSupportedInActiveSession   = 0x7E,
    ServiceNotSupportedInActiveSession       = 0x7F,
}

impl Nrc {
    pub const fn code(self) -> u8 { self as u8 }
}
```

### 4.5 密钥派生

```rust
// src/can/uds/security.rs

fn reverse_bits(b: u8) -> u8 {
    let mut b = b;
    b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1);
    b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2);
    b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4);
    b
}

/// 标准 LFSR + 反字节重组密钥派生（兼容 MiniUds 实现）。
/// 不同 SAL 用不同 mask（`config.key_masks[sal - 1]`）。
pub fn generate_key(seed: u32, mask: u32) -> u32 {
    let mut state = seed;
    for _ in 0..40 {
        if state & 0x8000_0000 != 0 {
            state = (state << 1) ^ mask;
        } else {
            state <<= 1;
        }
    }
    let mut key = 0u32;
    for i in 0..4 {
        let byte = reverse_bits((state >> ((3 - i) * 8)) & 0xFF) as u32;
        key |= byte << (i * 8);
    }
    key
}
```

### 4.6 Dispatch 主循环

```rust
// src/can/uds/mod.rs

/// Dispatch 返回值。
/// - `Ready`：response_buf 里有可发的正/负响应，caller 调 `take_response()` 拿走
/// - `Pending`：长任务挂起，要么已经发 0x78 要么即将发，caller 不读 response
/// - `Ignore`：请求被忽略（tx_disabled、空请求），caller 不发任何东西
pub enum DispatchResult {
    Ready,
    Pending,
    Ignore,
}

/// Pending 队列里的续延函数上下文。`complete` 标志续延函数是否完成。
pub struct UdsContext<'a> {
    pub state: &'a mut UdsState,
    pub config: &'a UdsConfig,
    pub complete: bool,
}

/// 初始化 UdsState 到 default 状态。
/// 接 `&mut UdsState` 而不是 `&'static mut`，方便单元测试用 stack allocate。
/// 实际生产用法是 `static mut UDS_STATE: UdsState = ...; init(&mut UDS_STATE);`。
pub fn init(state: &mut UdsState) {
    *state = UdsState {
        session: Session::Default,
        security: SecurityLevel::Locked,
        seed_sent: false,
        current_seed: 0,
        state: SrvState::Idle,
        request_len: 0,
        response_len: 0,
        request_tick_ms: 0,
        response_pending: false,
        download_active: false,
        transfer_sn: 0,
        tx_disabled: false,
        rx_disabled: false,
    };
}

/// Called by the canopen task on every SDO request to 0x2F00.0.
///
/// **tx_disabled 行为**：
/// - 0x28 0x00 enableNormalCommunication 永远 bypass 检查（必须能解锁）
/// - 其他请求如果 tx_disabled=true，发 NRC 0x22 ConditionsNotCorrect
///   （不是直接 Ignore——master 至少要知道请求被拒了）
///   等价于"网络被关"的标准语义：可以通信，逻辑上没条件
pub fn dispatch(state: &mut UdsState, config: &UdsConfig, request: &[u8]) -> DispatchResult {
    if request.is_empty() {
        return DispatchResult::Ignore;
    }

    // 0x28 0x00 enable 永久 bypass（必须能解锁）
    let is_enable = request.len() >= 2
        && request[0] == 0x28
        && request[1] == 0x00;

    if state.tx_disabled && !is_enable {
        // 网络被关：发 0x22 而不是 Ignore，master 至少知道
        store_negative(state, config, request[0], Nrc::ConditionsNotCorrect);
        finalize(state, config);
        return DispatchResult::Ready;
    }

    if state.state != SrvState::Idle {
        // 已经在处理一个请求，发 0x78 提示 master 稍后重试（ISO 14229 §6.4.2.4）
        send_response_pending(state, config);
        return DispatchResult::Pending;
    }

    state.state = SrvState::Parsing;
    state.request_len = request.len();
    state.request_tick_ms = (config.tick_ms)();
    state.response_len = 0;
    state.response_pending = false;
    copy_to_buf(config.request_buf, request);

    let sid = request[0];
    let entry = config.services.iter().find(|e| e.sid == sid);

    let entry = match entry {
        Some(e) => e,
        None => {
            store_negative(state, config, sid, Nrc::ServiceNotSupported);
            finalize(state, config);
            return DispatchResult::Ready;
        }
    };

    // 通用门控
    if let Err(nrc) = check_session_access(state, entry.session_access) {
        store_negative(state, config, sid, nrc);
        finalize(state, config);
        return DispatchResult::Ready;
    }
    if let Err(nrc) = check_security_level(state, entry.security_level) {
        store_negative(state, config, sid, nrc);
        finalize(state, config);
        return DispatchResult::Ready;
    }

    // 分发
    match entry.handler {
        ServiceHandler::Session         => session::handle(state, config, request),
        ServiceHandler::EcuReset        => reset::handle(state, config, request),
        ServiceHandler::ClearDtc        => dtc::handle_clear(state, config, request),
        ServiceHandler::ReadDtc         => dtc::handle_read(state, config, request),
        ServiceHandler::ReadDataById    => read_data::handle(state, config, request),
        ServiceHandler::WriteDataById   => write_data::handle(state, config, request),
        ServiceHandler::CommControl     => comm_control::handle(state, config, request),
        ServiceHandler::SecurityAccess  => security::handle(state, config, request),
        ServiceHandler::RoutineStart    => routine::handle(state, config, request, RoutineSub::Start),
        ServiceHandler::RoutineStop     => routine::handle(state, config, request, RoutineSub::Stop),
        ServiceHandler::RoutineResult   => routine::handle(state, config, request, RoutineSub::Result),
        ServiceHandler::RequestDownload => download::handle_request(state, config, request),
        ServiceHandler::TransferData    => download::handle_transfer(state, config, request),
        ServiceHandler::TransferExit    => download::handle_exit(state, config, request),
        ServiceHandler::TesterPresent   => tester_present::handle(state, config, request),
    }

    finalize(state, config);
    if state.state == SrvState::Pending {
        DispatchResult::Pending
    } else {
        DispatchResult::Ready
    }
}

fn finalize(state: &mut UdsState, _config: &UdsConfig) {
    if state.state == SrvState::Pending {
        return;
    }
    state.state = SrvState::Idle;
    // 同步路径：handler 已经 store_positive/negative 填好 response_buf
    // caller 调 take_response 拿走
    state.response_pending = true;
}

/// 把挂起函数推进 pending queue。
/// 返回 false 表示队列满，caller 发 NRC 0x22 ConditionsNotCorrect。
///
/// 必须是 FnMut（不是 FnOnce）因为 tick 可能调同一个 job 多次（直到 ctx.complete = true）。
/// download handler 用 `move |ctx| { ... }` 捕获 6 字节 payload 也要求闭包是 FnMut。
pub fn push_pending<F>(state: &mut UdsState, config: &UdsConfig, f: F) -> bool
where
    F: FnMut(&mut UdsContext) + 'static,
{
    for slot in config.pending_queue.iter_mut() {
        if slot.is_none() {
            *slot = Some(PendingJob { func: Box::new(f) });
            state.state = SrvState::Pending;
            return true;
        }
    }
    false
}

/// Called by the canopen task each iteration (not on every frame)。
///
/// 1. 推进挂起队列（如果 state == Pending）
/// 2. 检查 P2 超时 → 发 0x78
/// 3. 续延函数完成后，置 response_pending = true，caller 下次循环会发
///
/// **借用规则**（关键）：
/// 本函数不能同时持 `config.pending_queue` 的 mut borrow 和 `config` 的
/// shared borrow（UdsContext 借的）。所以用 **drain → process → put-back**
/// 三段式：
///   1. drain：把 pending_queue 全部 take 到本地数组（释放 mut borrow）
///   2. process：本地数组迭代，可以 shared borrow config
///   3. put-back：没完成的 job 放回 pending_queue
///
/// 这样既避开了借用冲突，也保持了"续延函数可以多次跑直到 complete"的语义。
///
/// **签名特殊**：接 `&mut UdsConfig` 而不是 `&UdsConfig`，因为要 mutate
/// pending_queue 两次（drain + put-back）。caller 调用时传 `&mut UDS_CONFIG`。
pub fn tick(state: &mut UdsState, config: &mut UdsConfig) {
    if state.state != SrvState::Pending {
        return;
    }

    // 1. drain：把 pending_queue 全部 take 到本地（释放 config.pending_queue mut borrow）
    let mut jobs: [Option<PendingJob>; 4] = core::array::from_fn(|_| None);
    for (i, slot) in config.pending_queue.iter_mut().enumerate() {
        jobs[i] = slot.take();
    }

    // 2. process：本地数组迭代，可以 shared borrow config
    let mut any_complete = false;
    for slot in jobs.iter_mut() {
        if let Some(job) = slot.as_mut() {
            let mut ctx = UdsContext { state, config: &*config, complete: false };
            (job.func)(&mut ctx);
            if ctx.complete {
                any_complete = true;
                *slot = None;  // drop the job
            }
            // else: 续延函数还要再跑，job 保留在 jobs[i] 里，put-back 时回 queue
        }
    }

    // 3. put-back：没完成的 job 放回 pending_queue
    for (i, dst) in config.pending_queue.iter_mut().enumerate() {
        *dst = jobs[i].take();
    }

    // 4. 如果有续延完成，置 response_pending
    //
    // 设计要点：dispatch 路径里有 finalize() 把 state.state 切回 Idle，
    // 但 tick 路径里续延函数不经过 finalize（它不是 dispatch 的同步 handler）。
    // 续延函数 ctx.complete = true 表示"我干完了，response_buf 填好了"，
    // 剩下的 state.state 切换由 tick 兜底。
    //
    // **多 job 并存时**：只有当 queue 里**所有** job 都完成，state 才能
    // 切回 Idle。如果一个 job 完成、另一个没完成，state 必须保持 Pending，
    // 否则新请求会进 dispatcher 跟未完成的 job 抢资源。
    //
    // 这种"切 state 由 dispatcher/tick 统一管"的模式避免了续延函数本身需要
    // 知道状态机的负担，handler 只关心业务逻辑（写 response_buf + 标
    // complete），状态切换由框架负责。
    if any_complete {
        state.response_pending = true;
        let queue_empty = jobs.iter().all(|j| j.is_none());
        if queue_empty {
            state.state = SrvState::Idle;
        }
        // else: 仍有 job 没完成，state 保持 Pending
    }

    // 5. 检查 P2 超时（pending 状态超过 P2 就要发 0x78）
    if state.state == SrvState::Pending {
        let now = (config.tick_ms)();
        if now.saturating_sub(state.request_tick_ms) >= config.p2_server_ms {
            send_response_pending(state, config);
            // 推进时间戳，避免每个 P2 都连发
            state.request_tick_ms = now;
        }
    }
}

/// Caller 在 SDO read 0x2F00.0 之前调用，拿到 (response bytes, ready_to_send)。
/// 拿走后 response_len 自动清零、response_pending 复位。
///
/// **Caller 协议要求**：
/// - 每次 `dispatch` 返回 `Ready` 之后，caller **必须**在下次 dispatch 之前
///   调一次 `take_response`，否则 `dispatch` 入口会清 `response_len` 丢响应
/// - 多次 `take_response` 之间借用的 `&[u8]` 必须先 drop（标准 NLL 即可）
/// - 返回 `None` 不代表错误——可能响应还没准备好（0x78 还没发、或 tick 还没
///   推进完），caller 应当稍后重试；master 端 ISO 14229 标准行为是"等 P2*
///   后再次 poll"
pub fn take_response(state: &mut UdsState, config: &UdsConfig) -> Option<&[u8]> {
    if !state.response_pending {
        return None;
    }
    state.response_pending = false;
    if state.response_len == 0 {
        return None;
    }
    let len = state.response_len;
    state.response_len = 0;
    Some(&config.response_buf[..len])
}

/// 把负响应写到 response_buf。`[0x7F, sid, nrc]`
fn store_negative(state: &mut UdsState, config: &UdsConfig, sid: u8, nrc: Nrc) {
    let bytes = [0x7F, sid, nrc.code()];
    copy_to_resp(state, config, &bytes);
}

/// 把正响应写到 response_buf。
fn store_positive(state: &mut UdsState, config: &UdsConfig, bytes: &[u8]) {
    copy_to_resp(state, config, bytes);
}

fn copy_to_resp(state: &mut UdsState, config: &UdsConfig, bytes: &[u8]) {
    let n = bytes.len().min(config.response_buf.len());
    config.response_buf[..n].copy_from_slice(&bytes[..n]);
    state.response_len = n;
}

fn copy_to_buf(dst: &mut [u8], src: &[u8]) {
    let n = src.len().min(dst.len());
    dst[..n].copy_from_slice(&src[..n]);
}

/// 把 0x78 ResponsePending 写到 response_buf，caller 下次 `take_response` 拿到。
/// master 收到 0x78 后**必须**等 P2* 后再 poll 0x2F00.0。
fn send_response_pending(state: &mut UdsState, config: &UdsConfig) {
    if state.request_len == 0 {
        return;
    }
    let sid = config.request_buf[0];
    let bytes = [0x7F, sid, Nrc::RequestCorrectlyReceivedResponsePending.code()];
    copy_to_resp(state, config, &bytes);
    state.response_pending = true;
}
```

### 4.7 Session/Security 门控模板

```rust
// src/can/uds/mod.rs (helpers)

fn check_session_access(state: &UdsState, required: u8) -> Result<(), Nrc> {
    if required & (1 << state.session as u8) == 0 {
        return Err(Nrc::SubFunctionNotSupportedInActiveSession);  // 0x7E
    }
    Ok(())
}

fn check_security_level(state: &UdsState, required: u8) -> Result<(), Nrc> {
    if (state.security as u8) < required {
        return Err(Nrc::SecurityAccessDenied);  // 0x33
    }
    Ok(())
}
```

每次进入 service handler 之前由 dispatcher 调用，门控失败直接发负响应，不进 handler。

### 4.8 SecurityAccess 0x27 实现

```rust
// src/can/uds/security.rs

pub fn handle(state: &mut UdsState, config: &UdsConfig, req: &[u8]) {
    // req[0] = 0x27, req[1] = subfunc, req[2..] = optional key
    if req.len() < 2 {
        store_negative(state, config, 0x27, Nrc::IncorrectMessageLengthOrInvalidFormat);
        return;
    }
    let subfunc = req[1];

    // SAL 计算: odd subfunc = requestSeed, even = sendKey
    let sal = match subfunc {
        0x01 | 0x02 => 1,  // SAL1
        0x03 | 0x04 => 2,
        0x05 | 0x06 => 3,
        _ => {
            store_negative(state, config, 0x27, Nrc::SubFunctionNotSupported);
            return;
        }
    };

    match subfunc % 2 {
        1 => handle_request_seed(state, config, sal, subfunc),
        0 => handle_send_key(state, config, sal, subfunc, req),
        _ => unreachable!(),
    }
}

fn handle_request_seed(state: &mut UdsState, config: &UdsConfig, sal: u8, subfunc: u8) {
    if (state.security as u8) >= sal {
        // 已经解锁：返回零 seed（ISO 14229 标准）
        let resp = [0x67, subfunc, 0x00, 0x00, 0x00, 0x00];
        copy_to_resp(state, config, &resp);
        return;
    }
    state.current_seed = (config.random_seed)();
    state.seed_sent = true;
    let resp = [
        0x67, subfunc,
        (state.current_seed >> 24) as u8,
        (state.current_seed >> 16) as u8,
        (state.current_seed >> 8) as u8,
        state.current_seed as u8,
    ];
    copy_to_resp(state, config, &resp);
}

fn handle_send_key(state: &mut UdsState, config: &UdsConfig, sal: u8, subfunc: u8, req: &[u8]) {
    if !state.seed_sent {
        store_negative(state, config, 0x27, Nrc::RequestSequenceError);  // 0x24
        return;
    }
    if req.len() != 6 {
        store_negative(state, config, 0x27, Nrc::IncorrectMessageLengthOrInvalidFormat);
        return;
    }
    state.seed_sent = false;
    let rx_key = u32::from_be_bytes([req[2], req[3], req[4], req[5]]);
    let expected = generate_key(state.current_seed, config.key_masks[(sal - 1) as usize]);
    if rx_key != expected {
        store_negative(state, config, 0x27, Nrc::InvalidKey);  // 0x35
        return;
    }
    state.security = match sal {
        1 => SecurityLevel::Sal1,
        2 => SecurityLevel::Sal2,
        3 => SecurityLevel::Sal3,
        _ => unreachable!(),
    };
    let resp = [0x67, subfunc];
    copy_to_resp(state, config, &resp);
}
```

### 4.9 0x28 CommunicationControl

```rust
// src/can/uds/comm_control.rs

pub fn handle(state: &mut UdsState, config: &UdsConfig, req: &[u8]) {
    if req.len() != 3 {
        store_negative(state, config, 0x28, Nrc::IncorrectMessageLengthOrInvalidFormat);
        return;
    }
    let subfunc = req[1];
    let _network_type = req[2];  // 简化版忽略（只支持 0x01 = normalCommunicationNetwork）

    match subfunc {
        0x00 => {  // enableNormalCommunication
            state.tx_disabled = false;
            state.rx_disabled = false;
            store_positive(state, config, &[0x68, 0x00]);
        }
        0x01 => {  // enableRxDisableTxNormalCommunication
            state.tx_disabled = true;
            state.rx_disabled = false;
            store_positive(state, config, &[0x68, 0x01]);
        }
        0x02 => {  // enableTxDisableRxNormalCommunication
            state.tx_disabled = false;
            state.rx_disabled = true;
            store_positive(state, config, &[0x68, 0x02]);
        }
        0x03 => {  // disableNormalCommunication
            state.tx_disabled = true;
            state.rx_disabled = true;
            store_positive(state, config, &[0x68, 0x03]);
        }
        _ => {
            store_negative(state, config, 0x28, Nrc::SubFunctionNotSupported);
        }
    }
}
```

`tx_disabled = true` 时，dispatcher 不发任何响应。canopen 任务里检查 `state.tx_disabled` 决定是否发 heartbeat/NMT ACK 等主动帧；`rx_disabled = true` 时（未实现，目前 dispatcher 总是处理 RX），将来可扩展。

### 4.10 0x31 RoutineControl

```rust
// src/can/uds/routine.rs

pub enum RoutineSub { Start, Stop, Result }

pub fn handle(state: &mut UdsState, config: &UdsConfig, req: &[u8], sub: RoutineSub) {
    // req: [0x31, subfunc, rid_hi, rid_lo, ...payload]
    if req.len() < 4 {
        store_negative(state, config, 0x31, Nrc::IncorrectMessageLengthOrInvalidFormat);
        return;
    }
    let subfunc = req[1];
    let rid = u16::from_be_bytes([req[2], req[3]]);

    let table = match sub {
        RoutineSub::Start => config.routines_start,
        RoutineSub::Stop => config.routines_stop,
        RoutineSub::Result => config.routines_result,
    };
    let entry = match table.iter().find(|e| e.rid == rid) {
        Some(e) => e,
        None => {
            store_negative(state, config, 0x31, Nrc::RequestOutOfRange);
            return;
        }
    };

    // 通用门控
    if let Err(nrc) = check_session_access(state, entry.session_access) { ... }
    if let Err(nrc) = check_security_level(state, entry.security_level) { ... }

    // 调 callback
    let payload = &req[4..];
    let mut resp_buf = [0u8; 32];  // 或更大
    match (entry.func)(payload, &mut resp_buf) {
        Ok(resp_len) => {
            let mut out = [0u8; 36];
            out[0] = 0x71;
            out[1] = subfunc;
            out[2] = req[2];
            out[3] = req[3];
            out[4..4 + resp_len].copy_from_slice(&resp_buf[..resp_len]);
            store_positive(state, config, &out[..4 + resp_len]);
        }
        Err(nrc) => {
            store_negative(state, config, 0x31, nrc);
        }
    }
}
```

### 4.11 0x34/0x36/0x37 Download 流程（带 Pending）

```rust
// src/can/uds/download.rs

pub fn handle_request(state: &mut UdsState, config: &UdsConfig, req: &[u8]) {
    // req: [0x34, data_format, addr(4), size(4)]
    if req.len() != 10 || req[1] != 0x00 {
        store_negative(state, config, 0x34, Nrc::RequestOutOfRange);
        return;
    }
    let size = u32::from_be_bytes([req[6], req[7], req[8], req[9]]);
    if size == 0 || size > MAX_DOWNLOAD_SIZE {
        store_negative(state, config, 0x34, Nrc::RequestOutOfRange);
        return;
    }
    // ... 检查 session/SAL ...

    // 发起 erase，挂起到 pending_queue
    let data = [req[2], req[3], req[4], req[5], req[6], req[7], req[8], req[9]];  // 复制
    if !push_pending(state, config, move |ctx| {
        if ota::erase_app_region().is_err() {
            store_negative(ctx.state, ctx.config, 0x34, Nrc::GeneralProgrammingFailure);
            ctx.complete = true;
            return;
        }
        // 准备响应
        let resp = [0x74, 0x00, 0x00, 0x20, 0x00];  // lengthFormatIdentifier=0x00, maxNumberOfBlockLength=0x0020
        ctx.state.download_active = true;
        ctx.state.transfer_sn = 1;
        store_positive(ctx.state, ctx.config, &resp);
        ctx.complete = true;
    }) {
        // 队列满
        store_negative(state, config, 0x34, Nrc::ConditionsNotCorrect);
    }
}

pub fn handle_transfer(state: &mut UdsState, config: &UdsConfig, req: &[u8]) {
    if !state.download_active {
        store_negative(state, config, 0x36, Nrc::RequestSequenceError);
        return;
    }
    if req.is_empty() {
        store_negative(state, config, 0x36, Nrc::IncorrectMessageLengthOrInvalidFormat);
        return;
    }
    let seq = req[1];
    if seq != state.transfer_sn {
        store_negative(state, config, 0x36, Nrc::WrongBlockSequenceNumber);
        return;
    }

    // transfer_sn 下一个值：0x01..=0xFF 顺序递增，0xFF 之后 wrap 到 0x01
    // （0x00 是 ISO 14229 保留值，server 不会在响应里回 0x00 序列号）
    state.transfer_sn = if state.transfer_sn >= 0xFF { 1 } else { state.transfer_sn + 1 };

    // 写 flash —— 如果超过 P2 也走 pending
    let block = [req[0], req[1], req[2], req[3], req[4], req[5], req[6], req[7]];  // 复制
    if !push_pending(state, config, move |ctx| {
        if ota::write_transfer_data(&block[2..]).is_err() {
            store_negative(ctx.state, ctx.config, 0x36, Nrc::GeneralProgrammingFailure);
            ctx.complete = true;
            return;
        }
        store_positive(ctx.state, ctx.config, &[0x76, block[1]]);
        ctx.complete = true;
    }) {
        store_negative(state, config, 0x36, Nrc::ConditionsNotCorrect);
    }
}
```

### 4.12 静态配置（src/can/uds_config.rs）

```rust
use crate::can::uds::config::*;
use crate::can::uds::nrc::Nrc;

// 全局缓冲区
static mut REQUEST_BUF: [u8; 64] = [0; 64];
static mut RESPONSE_BUF: [u8; 64] = [0; 64];
static mut PENDING_QUEUE: [Option<PendingJob>; 4] = [None, None, None, None];

// DID 表
static READ_DIDS: &[DidReadEntry] = &[
    DidReadEntry {
        did: 0xF186,
        session_access: 0b001,  // Default only
        security_level: 0,        // allpass
        func: read_active_session,
    },
];

static WRITE_DIDS: &[DidWriteEntry] = &[
    // WriteDID 一般需要 Programming + SAL1
];

// Routine 表
static ROUTINES_START: &[RoutineEntry] = &[
    RoutineEntry {
        rid: 0xFF00,
        session_access: 0b011,  // Programming | Extended
        security_level: 1,
        func: routine_erase,
    },
    RoutineEntry {
        rid: 0xF001,
        session_access: 0b011,
        security_level: 1,
        func: routine_crc_check,
    },
];

// 服务主表
static SERVICES: &[ServiceEntry] = &[
    ServiceEntry {
        sid: 0x10, session_access: 0b111, security_level: 0,
        handler: ServiceHandler::Session,
    },
    ServiceEntry {
        sid: 0x11, session_access: 0b011, security_level: 0,
        handler: ServiceHandler::EcuReset,
    },
    // ...
];

pub static UDS_CONFIG: UdsConfig = UdsConfig {
    services: SERVICES,
    read_dids: READ_DIDS,
    write_dids: WRITE_DIDS,
    routines_start: ROUTINES_START,
    routines_stop: &[],
    routines_result: &[],
    request_buf: unsafe { &mut REQUEST_BUF },
    response_buf: unsafe { &mut RESPONSE_BUF },
    pending_queue: unsafe { &mut PENDING_QUEUE },
    on_session_enter: Some(on_session_enter),
    on_session_exit: Some(on_session_exit),
    random_seed: ota_get_random_seed,  // 复用 OTA 的随机源
    key_masks: [0x3000_2212, 0x524C_5E63, 0xA5C3_F11B],  // 三级 SAL 各一个掩码
    p2_server_ms: 50,
    p2_star_ms: 5000,
    tick_ms: || embassy_time::Instant::now().elapsed().as_millis() as u32,
};
```

### 4.13 与 SDO/UDS 网关的集成

**核心适配问题**：UDS request 长度不固定（2-10 字节），CAN SDO 8 字节 payload 装不下 0x34 RequestDownload（10 字节）这种长请求。

**适配方案**：sdo.rs 维护 segmented SDO 状态机（Phase 4 已实现），在内部把多段拼成一个完整 UDS request 字节流（栈 buffer，最大 64 字节），拼好后**一次性**调 `uds::dispatch(slice)`。uds::dispatch 接收的 `request: &[u8]` 是**完整 UDS request**，sdo 负责分段传输。

**适配责任划分**：
- sdo.rs：CAN bus 帧分段、Initiate/Segment 流控、拼成完整 UDS request
- uds.rs：接收完整 UDS request、dispatch、返回完整 UDS response

**Caller 协议**（canopen_task 必须遵循）：
1. SDO 0x2F00.0 write 触发 sdo.rs 接收 SDO 帧
2. sdo.rs 内部拼好完整 UDS request，调 `uds::dispatch(&mut state, &config, &full_request)`
3. canopen_task 拿到 `DispatchResult`：
   - `Ready`：caller 调 `uds::take_response(&mut state, &config)` 拿完整响应，sdo.rs 把它分多段 SDO 发回 master
   - `Pending`：caller 不动 response，0x78 由 tick 自动发（同 Ready 路径）
   - `Ignore`：caller 不发响应，sdo.rs 不拼 SDO 响应
4. master 端下次 SDO read 0x2F00.0 才会触发 sdo.rs 读 response
5. **每次 dispatch 之后 caller 必须 take_response**（dispatch 入口会清 response_pending 和 response_len），否则新响应覆盖旧响应

**为什么 master 要 poll SDO read 拿响应而不是 write 立即回？** 因为 pending + 0x78 路径下响应可能延迟（等 tick 推进），master 必须主动轮询。这也是 ISO 14229 的标准做法。

`canopen_task` 每个 loop iteration 顺序：
1. **调 `uds::tick(&mut uds_state, &mut UDS_CONFIG)`**：推进 pending queue（每帧多走一点）、检查 P2 超时发 0x78。注意签名是 `&mut UdsConfig` 不是 `&UdsConfig`，因为 tick 要 mutate pending_queue
2. **处理 RX frame**（NMT / SDO）
3. **SDO read 0x2F00.0** → `uds::take_response(&mut state, &config)`，如果拿到 `Some(bytes)` 就发回给 master
4. **SDO write 0x2F00.0** → `uds::dispatch(&mut state, &config, data)`：
   - `DispatchResult::Ready`：caller 调 `take_response()` 发回（同步完成的正/负响应，或者 tick 推送的 0x78 / 续延完成响应）
   - `DispatchResult::Pending`：caller 不发响应（master 等 P2* 后再次 poll，poll 时会拿到 0x78 或者最终响应）
   - `DispatchResult::Ignore`：caller 不发响应（tx_disabled / 空请求）
5. **发主动帧前**（heartbeat / NMT ACK）检查 `!state.tx_disabled`，如果 0x28 关了 TX 就不发

`tx_disabled = true` 期间，dispatch 拒绝新请求（直接 Ignore），但 0x78 续延仍会发（已经处于 pending 状态）。0x28 重新 enable 后 `tx_disabled = false`，dispatch 恢复。

**Master 端协议要求**：当 `dispatch` 返回 `Pending` 时，master **必须**遵循 ISO 14229 退避：收到 0x78 后等 P2*（5 s）再 poll。如果不等，dispatch 会因为 `state.state != Idle` 拒绝新请求并立即发 0x78。

---

## 5. 迁移计划

### Phase 5a：架构（~3-4 天）

**目标**：把 src/can/uds.rs 拆成新模块，表驱动分发 + 多 SAL + 真实密钥 + pending queue + 0x78 + 完整 NRC。

**任务清单**：
1. 新建 src/can/uds/ 目录
2. 写 state.rs / config.rs / nrc.rs / security.rs
3. 写 mod.rs 的 dispatcher 主循环
4. 写 pending.rs 的 PendingJob + 0x78 自动续延
5. 把 src/can/uds.rs 的现有 handler 搬到子模块（session, reset, dtc, read_data, write_data, tester_present）
6. 写 src/can/uds_config.rs（静态配置）
7. 改 src/can/sdo.rs 调用新的 `uds::dispatch` 签名
8. 烟雾测试更新（保持 20 个原场景通过）

**验证**：所有现有 20 个场景继续 pass。

### Phase 5b：服务补齐（~2 天）

**目标**：0x28 CommunicationControl + 0x31 RoutineControl + 补齐 NRC + 0x3E suppressPositiveResponse。

**任务清单**：
1. 写 comm_control.rs（enable/disable TX-RX）
2. 写 routine.rs（start/stop/result 表）
3. 改 tester_present.rs 支持 subfunc `0x80`（不响应）
4. 给 read_data / write_data / download / routine 加 `session_access` 字段
5. 把现有 `handle_session_control` / `handle_ecu_reset` 等调用 `config.on_session_enter/exit`

**验证**：新加 ~15 个场景（comm_control、routine、tester_present suppress、错误 session 的 0x7E）。

### Phase 5c：Download/OTA 接入新架构（~2 天）

**目标**：0x34/0x36/0x37 走 pending queue + 0x78，OTA 用真实随机 seed。

**任务清单**：
1. 写 download.rs 的三个 handler + OTA 接入 pending
2. 写 download::pending_complete（写到一半的 flash 错误怎么回滚）
3. 把 src/can/ota.rs 的 OTA 状态机移到 uds/download.rs（保留 OTA 状态、让 dispatch 知道何时 transition）
4. 重写 random_seed callback（用 embassy-time 计数器 + SysTick XOR + 一些 LFSR 噪声）

**验证**：烟雾测试加 `uds_pending_timeout`（fake clock 推进 P2 ms）、`uds_pending_completion`（等完成后再读）。

### Phase 5d：测试 + 文档（~2 天）

**目标**：36 个场景、全绿、spec 文档更新。

**任务清单**：
1. 烟雾测试新增 16 个场景
2. **改 Python 烟雾测试加 0x78 重试逻辑**：`SDO read 0x2F00.0` 拿到的响应如果是 `[0x7F, sid, 0x78]`，等 P2* 后再 poll 一次（最多 5 次循环）
3. 更新 docs/superpowers/specs/2026-07-02-can-ota-uds-design.md（标 deprecated，指向新 doc）
4. 写 README 段说明怎么加新 DID（"编辑 src/can/uds_config.rs，加一个 DidReadEntry"）

---

## 6. 测试计划

### 6.1 现有 20 个场景（必须保持通过）

包括 14 个 wire-format 场景（SDO/UDS/OTA）和 6 个状态机场景（toggle / size_field / replay / dl_initiate / dl_one_segment / dl_toggle / dl_stray / dl_size_mismatch）。

### 6.2 新增 16 个场景

| 场景 | 验证内容 |
|---|---|
| `uds_table_dispatch_add_did` | 加一个 DidReadEntry，不改 dispatcher 代码，master 能读 |
| `uds_session_notify` | session 切换时 on_session_enter 被调 |
| `uds_sal_sequence` | 0x27 完整 RequestSeed → SendKey → RequestSeed 第二次应回零 seed |
| `uds_pending_timeout` | 长任务超过 P2 后，SDO read 返回 [0x7F, 0x10, 0x78] |
| `uds_pending_completion` | pending 完成后 master poll 拿到正响应 |
| `uds_comm_control_disable` | 0x28 0x03 后 master 收不到任何 SDO 响应 |
| `uds_comm_control_enable` | 0x28 0x00 后恢复正常 |
| `uds_routine_erase` | 0x31 0x01 rid 0xFF00 触发擦除 callback |
| `uds_routine_crc` | 0x31 0x03 rid 0xF001 返回 CRC |
| `uds_did_session_gate` | 0x22 在 Default session 下不响应 registered-for-Programming DID |
| `uds_did_sal_gate` | 0x2E SAL 不足时 0x33 |
| `uds_tester_present_suppress` | 0x3E subfunc 0x80 不响应 |
| `uds_seed_random` | 两次 RequestSeed 拿到的 4 字节不同 |
| `uds_key_derivation_known` | 给定固定 seed/mask，能算出预期 key（防止派生算法悄悄改了） |
| `uds_pending_queue_full` | 挂起队列满返回 0x22 |
| `uds_inactive_session_nrc` | 0x12 vs 0x7E 区分（错误 session 用 0x7E，不是 0x12） |

### 6.3 加密测试

- 已知 seed/mask 算 key，确认算法没漂移
- 随机 seed 连续 1000 次不重复（用伪随机源）
- 跨 SAL 用不同 mask，key 不能互通

### 6.4 实时性测试

- pending_queue tick 必须在主循环 ≤ 1 ms 内完成
- 0x78 超时 ≤ P2_server_ms（默认 50 ms）

---

## 7. 风险分析

### 7.1 RAM

当前 RAM 用量：~9 KB / 32 KB。
新增内容：
- 配置结构体 ~500 B
- 三个 DID/routine 表 ~200 B（每个 entry ~16 B × ~30 项）
- 缓冲区 64+64 = 128 B
- pending_queue 4 项 × ~32 B = 128 B
- UdsState ~64 B

总新增 ~1 KB。RAM 余量从 23 KB 降到 22 KB，安全。

### 7.2 二进制大小

当前 .text ~80 KB / .data ~6.5 KB / .rodata ~24 KB。
新增代码估计 +15 KB .text。.text 余量从 25 KB 降到 10 KB，可以接受。

`static SERVICES: &[ServiceEntry] = &[...]` 这种 const 数组在 Rust 里**默认就在 .rodata**（`&'static [T]` 数组是只读静态），不会进 .text。无需手动移动。`.text` 增量主要是 dispatcher 主循环 + handler 拆分后的代码副本。

### 7.3 回归风险

- 现有 20 个场景必须全绿。任何一条 fail 都说明 wire format 被破坏。
- SDO 0x2F00 网关的接口不能变（master 协议不变）。
- 现有 OTA 流程（0x34 → 0x36 N 次 → 0x37）端到端兼容，但**响应时机可能变化**（pending + 0x78）。现有 master (Python 烟雾测试) 是阻塞 read，0x78 路径上需要 poll 第二次。

### 7.4 状态机正确性

- 进入 DefaultSession **不**清空 SecurityAccess（默认 session 不锁安全是 ISO 14229 的标准行为，但 reference 是"进入 default 就 clear"，行为不同）
- 进入 ProgrammingSession **要**清空 SecurityAccess（reference 行为）
- 进入 ExtendedSession **要**清空 SecurityAccess（reference 行为）
- seed_sent 在 SendKey 之后清空，RequestSeed 之后置位
- transfer_sn 范围 0x01..=0xFF，0xFF 之后 **wrap 到 0x01**（skip 0x00，因为 ISO 14229 §11.4.3 规定 0x00 是"保留"序列号，server 永远不期待 0x00 也不应在响应里回 0x00）
- 续延函数 `ctx.complete = true` 表示单次续延完成，false 表示还要再跑（multi-block 续延场景）

### 7.5 兼容性问题

- **0x2F00.0 SDO 网关**保持不变（这条是协议级约束）
- **wire format 不变**：现有 14 个 wire-format 场景都是按字节检查，重构必须保持字节级一致
- **OTA 流程不变**：但 pending + 0x78 是新行为，master 必须支持

### 7.6 不做列表（明确排除）

- 不实现 CAN TP 层（DoCAN）
- 不实现 KWP2000
- 不修改 bootloader
- 不添加 flash 双 bank（A/B 切换）

---

## 8. 决策点（已确认）

| 决策点 | 选择 | 说明 |
|---|---|---|
| SAL 等级 | **SAL1/2/3** | 三级完整支持，每级独立 `key_masks`、独立服务集 |
| P2 server timer | **50 ms** | ISO 14229 标准值。P2* = 5000 ms |
| 0x28 CommunicationControl | **标准版** | 全部 subfunc：`0x00`/`0x01`/`0x02`/`0x03` |
| Pending 队列大小 | **4 项** | 覆盖 TransferData + TransferExit + waiting × 2 |
| 静态配置位置 | **`src/can/uds_config.rs`** | 集中所有 const 表 |

### 8.1 SAL 等级

| SAL | 用途（参考 MiniUds 实践） | 典型服务 |
|---|---|---|
| SAL1 | 产线刷写 | 0x34/0x36/0x37（OTA）、部分 0x31 routine |
| SAL2 | 4S 店诊断 | 0x22/0x2E 大量 DID、0x31 诊断 routine |
| SAL3 | 厂商后门 | 仅厂商测试用 DID，0x10 extended session 才能进入 |

`key_masks: [u32; 3]` 数组每级一个掩码，握手验证逻辑同 SAL1。

### 8.2 P2 server timer

`P2_SERVER_MS = 50`，`P2_STAR_MS = 5000`。OTA 长任务超过 50 ms 自动发 `0x78`，master 等 5 s 后放弃。

### 8.3 0x28 CommunicationControl 标准版 subfunc

| subfunc | 含义 | TX | RX |
|---|---|---|---|
| `0x00` | enableNormalCommunication | ON | ON |
| `0x01` | enableRxDisableTx | OFF | ON |
| `0x02` | enableTxDisableRx | ON | OFF |
| `0x03` | disableNormalCommunication | OFF | OFF |

注：`network_type` 字段保留字节（按 ISO 14229 `0x01` = normalCommunicationNetwork 即可），不实现 type-specific filtering。

---

## 9. 总结

**这次重构值得做**。原因：

1. 当前 UDS 是"凑出来"的产物，4 轮 audit 累计 18 个 bug 没碰到架构层 —— 不是因为没 bug，是因为没找到 bug
2. 加新 DID 改代码 = 不可扩展
3. OTA 同步阻塞 = 不抗 P2 timer 50 ms 的真实诊断仪
4. 固定 seed/key = 安全反模式
5. 缺 0x78 / 缺 0x28 / 缺 0x31 / 缺 NRC = 标准覆盖不全

**估计工作量**：~1150 行新增/重写代码 + 350 行烟雾测试 + 1 周集中工作。回报是 **架构层面消除了一整类风险**（固定 seed、加 DID 改代码、pending 卡死主循环）。

**风险**：回归（wire format 必须保持兼容）。4 轮 audit 已经验证了字节级一致性，重构过程中**任何 wire-format 场景 fail 立刻回滚**。

**前置条件**：
- 决策 §8 的 5 个选项（建议采纳我的推荐）
- 接受"加新 DID 改 uds_config.rs 而不是改 uds.rs"作为长期约定

**下一步**：等你拍板 §8，然后我开始 Phase 5a 的具体实现。

---

## 附录 A：reference MiniUds 关键源码摘要

### A.1 主循环（mini_uds.c:39-45）

```c
void mini_uds_main_task(udscb_t *udscb)
{
    mini_uds_do_pending(udscb);     // 处理挂起队列
    mini_uds_sender_process(udscb); // 发响应
    mini_uds_do_services(udscb);    // 解析 + dispatch
    mini_uds_sender_process(udscb); // 再发一次
}
```

### A.2 0x78 自动续延（mini_uds.c:408-461）

```c
static void mini_uds_sender_process(udscb_t *udscb)
{
    if (UDS_SENDER_WAIT == udscb->sender_state) {
        udscb->sender_state = UDS_SENDER_IDLE;
        udscb->srv_state = UDS_SERVICE_IDLE;
        send_data(response_buf, response_len);
    } else if (UDS_SERVICE_IDLE != udscb->srv_state) {
        // PENDING 状态超时 → 发 0x78
        if (elapsed > PENDING_TIME_TICKS) {
            response_buf[0] = 0x7F;
            response_buf[1] = request_buf[0];
            response_buf[2] = nrcRequestCorrectlyReceived_ResponsePending;
            send_data(response_buf, 3);
        }
    }
}
```

### A.3 密钥派生（mini_uds_srv.c:1082-1128）

```c
static uint32_t generate_key_from_seed(uint32_t seed, uint32_t mask)
{
    uint32_t state = seed;
    for (int i = 0; i < 40; i++) {
        if (state & 0x80000000) {
            state = (state << 1) ^ mask;
        } else {
            state <<= 1;
        }
    }
    uint32_t key = 0;
    for (int i = 0; i < 4; i++) {
        key |= bit_change(state >> ((3 - i) << 3)) << (i << 3);
    }
    return key;
}
```

### A.4 配置示例（ota_cfg.c:92-178）

```c
const udscfg_t ota_uds_config = {
    .request_buf = ota_request_databuf,
    .response_buf = ota_response_databuf,
    .pending_queue = ota_pendfing_func_queue,
    .send_data = ota_user_send_data,
    .get_curtick = srvtimer_get_tick,
    .srv10 = {
        .default_notify = NULL,
        .programming_notify = NULL,
        .extend_notify = NULL,
    },
    .srv22 = {
        .num = ARRAY_SIZE(ota_srv22_table),
        .list = ota_srv22_table,
    },
    .srv27 = { .get_random_seed = ota_get_random_seed },
    .srv34 = { ... },
    // ...
};
```

## 附录 B：现有 src/can/uds.rs 与新设计的对应关系

| 现有函数 | 新位置 | 备注 |
|---|---|---|
| `dispatch()` | `uds::dispatch()` | 重写为表驱动 |
| `handle_session_control` | `uds::session::handle` | 拆出 |
| `handle_ecu_reset` | `uds::reset::handle` | 拆出 |
| `handle_clear_dtc` | `uds::dtc::handle_clear` | 拆出 |
| `handle_read_dtc` | `uds::dtc::handle_read` | 拆出 |
| `handle_read_did` | `uds::read_data::handle` | 改用 DID 表查找 |
| `handle_write_did` | `uds::write_data::handle` | 改用 DID 表查找 |
| `handle_security_access` | `uds::security::handle` | 真实密钥派生 |
| `handle_tester_present` | `uds::tester_present::handle` | 加 suppressPositiveResponse |
| (新) | `uds::comm_control::handle` | 新增 |
| (新) | `uds::routine::handle` | 新增 |
| `store_positive` / `store_negative` | `uds::mod::store_*` | 移到公共模块 |
| `LAST_RESPONSE` 静态存储 | `config.response_buf` | 移到配置 |
| `SESSION` / `SECURITY` AtomicU8 | `UdsState.session` / `.security` | 普通字段 |

## 附录 C：现有 src/can/sdo.rs 改动清单

| 位置 | 改动 |
|---|---|
| `dispatch()` | 调用 `uds::dispatch()` 替代 `super::uds::dispatch()` |
| `od::write` 中 0x2F00.0 分支 | 调用 `uds::dispatch()` 替代 `super::uds::dispatch()` |
| 0x2F00.0 read | 调 `uds::last_response(config)` 替代 `super::uds::load_response()` |
| 集成处加 `tx_disabled` 检查 | 0x28 disableNormalCommunication 后不发主动帧 |

## 附录 D：烟雾测试改动清单

| 文件 | 改动 |
|---|---|
| `scripts/smoke_test.py` | 加 15 个新场景；现有 20 个场景的 wire 字节期望可能需要更新（dispatcher 行为变了但 wire 应该不变） |
| 加 `UDS` Python 端的 Session access 测试 | 验证 master 在错误 session 收到 0x7E（不是 0x12） |

---

*文档结束。下一步等用户决策 §8 后开始 Phase 5a 实现。*