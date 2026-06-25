# STM32G431CBU6 128KB Flash 容量评估

**日期:** 2026-06-25
**状态:** 调研性分析
**MCU:** STM32G431CBU6 (128KB flash, 32KB SRAM)
**目标栈:** Embassy + FOC + shell(embedded-cli)+ zencan(CANopen)+ OTA

## 结论先说

**128 KB flash 不够"从容"地装下完整栈。** 全部 feature 启用时,可用空间仅剩 0-15 KB 余量,任何一行新代码都可能 link 失败。**强烈建议分阶段实施或换芯片**。

## 各模块 ROM 占用估算

数字均为工程经验估计,非实测。最终以 `cargo build --release` 的 size report 为准。

| 模块 | 估计 | 备注 |
|---|---|---|
| embassy-stm32 HAL + executor + time | 15-25 KB | HAL PAC 占比最大;`defmt`/`time-driver-any` feature 必开,其余按需关 |
| defmt + defmt-rtt + panic-probe | 2-4 KB | |
| **OTA bootloader**(独立段) | 8-16 KB | 视 bootloader 复杂程度;简陋 swap 协议 ~8 KB,带签名验签 ~16 KB |
| FOC 主算法 | 8-20 KB | Clarke/Park/SVPWM/PI/反正切;用 CORDIC + hardfloat 压低 |
| 电机参数持久化(flash 读写) | 2-4 KB | STM32G4 flash 编程 + 简单 wear-level |
| `embedded-cli` shell | 10-15 KB | Arduino Nano 实测 16 KB,加 wrapper 略多 |
| `zencan` CANopen | 15-30 KB | 对象字典规模决定;CiA 402 完整 ~30 KB,精简 5-10 个对象 ~10 KB |
| 链接器开销 + .data + bss | 3-5 KB | |
| **小计** | **~63-119 KB** | |

## 关键约束:OTA flash 分割

STM32G431CBU6 无 dual-bank,bootloader 与 app 共享同一片 flash:

```
+----------------+ 0x08000000
|  bootloader    |  ~12 KB  (典型)
+----------------+ 0x08003000
|  app slot A    |
|                |  ~100 KB
+----------------+ ~0x0801F000
|  metadata      |  ~4 KB  (version, size, signature)
+----------------+ 0x08020000  end
```

**app 实际可用 ≈ 110-115 KB**,扣业务 5 项(50-75 KB),**留给 zencan + shell 共 35-65 KB**。

## 三种实施情境

| 情境 | shell | zencan | FOC | OTA | 总占用 | 结论 |
|---|---|---|---|---|---|---|
| **最简**:FOC + shell,无 CANopen,无 OTA | 12 | 0 | 12 | 0 | ~50 KB | ✓ 轻松 |
| **中等**:+ zencan(精简对象字典) | 12 | 12 | 15 | 12 | ~80 KB | ✓ 紧但可装(`opt-level="z"` + LTO) |
| **完整**:+ zencan(完整 CiA 402)+ OTA | 15 | 25 | 18 | 14 | ~115 KB | ⚠️ **卡边,余量 0-15 KB** |

## 三条解法路径

| 方向 | 代价 | 收益 | 推荐度 |
|---|---|---|---|
| **A. 砍功能** | 去掉 zencan / 简化 shell / 暂不做 OTA | 30-50 KB 余量,部署简单 | ⭐⭐ 适合学习/原型 |
| **B. 砍体积** | `opt-level="z"` + LTO + CORDIC + 精简 zencan 对象字典 + 关闭冗余 embassy features | 省 15-20 KB | ⭐⭐⭐ 性价比高但调试更难、CI 慢 |
| **C. 换芯片** | STM32G4**73**RE(同 G4 生态,512 KB)或 STM32H723(1 MB+,Cortex-M7) | 直接 4-8× 余量,无功能妥协 | ⭐⭐⭐⭐ 长远最优 |

## 推荐方案:C(换芯片)或 A+B(留在 G431)

**选 C(换芯片)的理由:**

- B-G431B-ESC1 是学习板,你会持续加 feature(无线、IMU、SD 卡、更多传感器...)
- 128 KB 在第一阶段就撞墙
- STM32G473RE 是同 G4 生态,embassy / PAC / 编程模型 100% 兼容
- 512 KB flash + 128 KB SRAM,Cortex-M4F 170 MHz,引脚兼容需要做小改动
- 或者直接上 STM32H723 / STM32H733,Cortex-M7 @ 550 MHz,1+ MB flash,FPU + DSP

**选 A+B(留在 G431)的理由:**

- 不想换 PCB
- 学习阶段,先验证算法,OTA / CANopen 可以延后
- 这个板子 B-G431B-ESC1 是 ST 官方有,学习资料多

## 留在 G431CB 时的分阶段实施建议

| 阶段 | 范围 | 估计 flash |
|---|---|---|
| 1 | FOC + shell(无 CANopen,无 OTA) | ~50 KB |
| 2 | + OTA(bootloader 吃 12 KB) | ~80 KB |
| 3 | + zencan(精简对象,只跑 NMT + SDO + 1 个 TPDO/RPDO) | ~95 KB |
| 4 | 链接失败 → 换芯片 G473 或砍功能 | — |

## 不确定项 / 后续验证点

- 实际 flash 占用需等每阶段实施后用 `cargo size` 或 `cargo bloat` 验证
- `zencan` 的对象字典大小需根据实际 profile(是否要 PDO mapping、是否要 manufacturer-specific 区段)决定
- `embedded-cli` 实际大小需在 shell 集成 spec 实施时测量
- 编译选项优化效果需在 release profile 下验证

## 相关文档

- [主 spec: 2026-06-25-b-g431b-esc1-initialization-design.md](./2026-06-25-b-g431b-esc1-initialization-design.md) — 当前 spec,在 风险与未决项 章节引用了本文档
- [实施计划: 2026-06-25-b-g431b-esc1-init.md](../plans/2026-06-25-b-g431b-esc1-init.md) — 阶段 1 实施计划
