# 里程碑 1:TIM1 互补 PWM + 开环旋转电压矢量

**日期:** 2026-07-01
**状态:** 待审核
**目标平台:** ST B-G431B-ESC1 (STM32G431CBU6, 170MHz)
**前置依赖:** [基础初始化 spec](./2026-06-25-b-g431b-esc1-initialization-design.md)、[Shell + OTA 设计](./2026-06-25-shell-and-ota-design.md)
**参考工程:** `~/download/122`(STM32CubeMX + MCSDK v6.4.1,同板同电机)

## 背景

`foc-algo` 已实现完整的 FOC 纯算法(Clarke/Park、SVPWM、PID、级联环、前馈解耦、退磁保护、SMO 观测器),但**尚未接入固件主程序**——`main.rs` 目前只跑 shell + heartbeat + OTA,控制环从未被调用(因此被 DCE 剥离,flash 占用极低)。

本里程碑是「真正把 FOC 跑起来」这一大目标的**第一个增量**。整体拆成 6~7 个可独立验证的里程碑,本 spec 只覆盖第一个:

> **TIM1 互补 PWM 驱动 + BSP 引脚/时钟/死区配置 + 开环旋转电压矢量(复用 `foc-algo` 的 SVPWM),电机能空载平滑转起来。全程走 embassy-stm32 高层驱动,不碰 ADC、不下沉 PAC。**

后续里程碑(顺序,均不在本 spec 范围):三电阻 ADC 注入采样 + OPAMP → 20kHz 电流环 ISR → 无感角度(SMO)→ 启停状态机 → 硬件过流刹车。

## 硬件事实(从参考工程 `.ioc` 提取)

**外设映射(B-G431B-ESC1)**

| 功能 | 外设 / 引脚 |
|------|-----------|
| 三相 PWM | TIM1 互补,中心对齐 20kHz。CH1 PA8 / PC13(N)、CH2 PA9 / PA12(N)、CH3 PA10 / PB15(N) |
| 死区 | 硬件栅极驱动 800ns;定时器 SW 死区 750ns |
| 调试串口 | USART2 PB3(TX)/PB4(RX)(**已实现,不动**) |
| 母线电压 | PA0 (ADC1_IN1),分压系数 0.0963(→ 24V 标称)*(本里程碑不用)* |

**电机参数(M1 "122")** *(本里程碑仅记录,不用于开环)*:Rs=0.32Ω,Ls=0.47mH(Ld=Lq),磁链 λ=0.0367Wb,极对数 4,额定/最大电流 5A,母线 24V,最高 6420rpm,退磁电流 -5A。

**时钟**:TIM1 挂 APB2,`bsp::clocks()` 中 `apb2_pre=DIV1` → TIM1 时钟 **170MHz**。中心对齐 20kHz → ARR = 170e6/(2·20000) = **4250**。

## 范围

### 包含

- 扩展 `bsp.rs`:配置 TIM1 6 路互补 PWM(中心对齐、20kHz、死区),初始**主输出关断**;`board_init()` 额外返回 PWM 句柄。
- 新增 `drivers/motor_pwm.rs`:`ComplementaryPwm<TIM1>` 薄封装(`enable`/`disable`/`apply(Duty)`/`arr`)。
- 新增 `control/open_loop.rs`:开环旋转矢量生成器(角度积分 + 极坐标→αβ 纯函数 + `foc-algo::Svpwm` + `foc-algo::Ramp` 软启动)。
- 新增 `tasks/motor.rs`:Ticker@10kHz 任务,读共享指令 → 驱动 PWM。
- 扩展 `commands/shell.rs`:新增 `spin <freq_hz> <voltage>` / `stop` 两条命令,写共享指令。
- 共享指令 `static`:`Mutex<CriticalSectionRawMutex, Cell<OpenLoopCmd>>`。
- `open_loop.rs` 纯函数的 host 单测。

### 不包含(YAGNI / 留给后续里程碑)

- ADC / 电流采样 / OPAMP 配置
- 电流环、闭环、级联环
- 无感观测器(SMO)
- 故障处理 / BKIN 硬件刹车
- 温度采样、母线电压采样
- 启停按钮(PC10 EXTI)
- V/f 斜坡(本里程碑用恒幅恒频旋转矢量,频率/电压由用户在 shell 手动调)
- TIM1 update 中断 / 任何 ISR(开环用异步 Ticker 任务)

## 架构

### 模块布局

```
src/
├── bsp.rs              扩展:TIM1 引脚/时钟/20kHz/死区/初始关断;board_init 返回 MotorPwm
├── drivers/
│   ├── debug_uart.rs   (不动)
│   └── motor_pwm.rs    新增:ComplementaryPwm<TIM1> 薄封装
├── control/
│   ├── mod.rs          新增
│   └── open_loop.rs    新增:角度积分 + 极坐标→αβ + Svpwm + Ramp
├── tasks/
│   ├── heartbeat.rs    (不动)
│   ├── shell.rs        (不动)
│   ├── mod.rs          扩展:导出 motor_task
│   └── motor.rs        新增:Ticker@10kHz 任务
└── commands/
    └── shell.rs        扩展:spin / stop 命令
```

### 组件职责与接口

**`drivers/motor_pwm.rs` — `MotorPwm`**

- *做什么*:把 3 相占空(`foc_algo::Duty`,[0,1])写进 TIM1 三通道;管理主输出使能。
- *怎么用*:`enable()`(MOE=1)、`disable()`(MOE=0,输出安全关断)、`apply(Duty)`(内部 `duty.to_timer_counts(arr)` → 三次 `set_duty`)、`arr() -> u16`。
- *依赖*:`embassy_stm32::timer::complementary_pwm::ComplementaryPwm<'static, TIM1>`、`foc_algo::Duty`。

**`control/open_loop.rs` — 开环生成器**

- *做什么*:维护电角度 θ;每拍推进 θ 并输出旋转电压矢量 (v_alpha, v_beta)。
- *纯函数(可 host 单测)*:
  - `advance_angle(theta, freq_hz, dt) -> theta'`:`θ + 2π·f·dt`,wrap 到 [0, 2π)。
  - `voltage_vector(voltage, theta) -> (v_alpha, v_beta)`:`(V·cosθ, V·sinθ)`。
- *有状态部分*:`OpenLoop { theta, voltage_ramp: foc_algo::Ramp, svpwm: foc_algo::Svpwm }`,`step(cmd, dt) -> Duty`,内部软启动电压、推进角度、`svpwm.update` → `Duty`。
- *依赖*:`foc_algo::{Svpwm, Ramp, Duty}`、`libm`(cos/sin;`foc-algo` 已启用 `libm-trig` 或本地用 `libm` crate — 见下)。

**`tasks/motor.rs` — `motor_task`**

- *做什么*:`Ticker` 每 100µs 触发一次;读共享 `OpenLoopCmd`;驱动 `OpenLoop::step` 和 `MotorPwm`。
- *使能沿*:检测 `enabled` false→true 调 `pwm.enable()`;true→false 时先把电压斜坡归零再 `pwm.disable()`。
- *定期* defmt 打印 θ / 占空(降频,如每 2000 拍一次)。
- *依赖*:`MotorPwm`、`OpenLoop`、共享指令 static、`embassy_time::Ticker`。

**`commands/shell.rs` — 扩展**

- `spin <freq_hz> <voltage>`:解析两个 f32 参数;`voltage` clamp 到 `MAX_OPENLOOP_V`(超限告警);写 `OpenLoopCmd{enabled:true, ...}`。
- `stop`:写 `enabled:false`。
- 复用现有 `write_u32` 风格的无分配输出;f32 解析用简易实现或 `core::str::parse`(需确认 no_std 下可用,否则手写定点解析)。

### 共享指令

```rust
#[derive(Clone, Copy)]
pub struct OpenLoopCmd { pub enabled: bool, pub freq_hz: f32, pub voltage: f32 }

static OPEN_LOOP_CMD: Mutex<CriticalSectionRawMutex, Cell<OpenLoopCmd>>
    = Mutex::new(Cell::new(OpenLoopCmd { enabled: false, freq_hz: 0.0, voltage: 0.0 }));
```

shell 与 motor 任务同执行器单线程,`CriticalSectionRawMutex` 足够;不需要 async Mutex 的 await。

### 数据流

1. **上电**:`bsp::clocks()`(不动)→ `board_init()` 除 USART2 外再建 TIM1 PWM,**MOE=0 电机安全无输出**。
2. spawn heartbeat / shell / **motor** 三任务。
3. `spin 10 2.0` → shell 解析 → 写指令。
4. **motor 任务 @10kHz**(dt=100µs)每拍:读指令 → 使能沿处理 → `OpenLoop::step`(软启动电压、推进 θ、算 αβ、`svpwm.update`)→ `MotorPwm::apply`。
5. `stop` → 斜坡停机 → `disable()`。

## 参数默认值

| 常量 | 值 | 说明 |
|------|-----|------|
| `PWM_FREQ_HZ` | 20_000 | 中心对齐 |
| `MOTOR_TICK_US` | 100 | 10kHz 更新 |
| `DEAD_TIME_NS` | 750 | 定时器死区(硬件另有 800ns) |
| `MAX_OPENLOOP_V` | 3.0 | 开环电压硬上限(母线 24V,无过流保护,压低) |
| `VOLTAGE_RAMP_V_PER_S` | 5.0 | 软启动斜率 |

## 安全(开环无电流反馈,重点)

- **上电即关断**:MOE=0,只有 `spin` 才使能。
- **电压硬上限** `MAX_OPENLOOP_V=3.0V`:`spin` 超限则 clamp 并告警。
- **软启动/停**:电压经 `Ramp` 渐变,杜绝突加冲击电流。
- `stop` → 斜坡归零后关输出。
- 死区(750ns)保证上下桥不直通。

## 验证

- `foc-algo::Svpwm` 已有 host 单测覆盖。
- `open_loop.rs` 纯函数 host 单测:角度 wrap 正确、矢量幅值 = V、频率×dt→角增量。
- **上板**:`cargo build` 通过并烧录;`version`/`info` 仍正常;`spin`/`stop` 有响应;示波器观察 PWM 引脚为中心对齐互补波、占空随角度旋转;接电机应能空载平滑转动;motor 任务定期 defmt 打印 θ/占空。
- 驱动层本身无 host 单测(依赖硬件)。

## 待实现时确认的技术细节(非阻塞)

- **三角函数来源**:`open_loop.rs` 的 cos/sin 用 `libm` crate(app 直接依赖)还是复用 `foc-algo` 的 `Trig`/`LibmTrig`。倾向后者以保持一致。
- **MOE 初始关断的确切 API**:embassy `new_inner` 会调 `enable_outputs()`;需在 `MotorPwm::new` 里紧接 `set_master_output_enable(false)` 确认输出确实关断,并把三通道 `enable(Channel)`。
- **f32 参数解析**:确认 no_std + core 下 `str::parse::<f32>()` 可用;不可用则手写。
