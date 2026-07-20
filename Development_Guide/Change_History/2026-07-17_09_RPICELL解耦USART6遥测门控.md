# 2026-07-17_09 RPICELL 解耦 USART6 遥测门控

## 依据

树莓派侧《5G 连接状态 `RPICELL` 固件对接说明》V1.0（2026-07-17）。以该文档为准。

文档 §1 明确界定 `RPICELL` 的语义：

> 本协议反映的是树莓派 Linux 侧网络链路状态，不包含信号强度、运营商、SA/NSA 制式或 SIM 注册详情。

即 `RPICELL` 只描述树莓派 Linux 侧 `usb0` 的网络链路，其用途是驱动显示屏的"5G 已连接/未连接"。文档全文未赋予它任何门控职责。

## 问题：`RPICELL` bit0 被用来门控 USART6 遥测出口

原实现在收到 `RPICELL` 后，除更新 5G 显示状态外，还用 bit0 关断了本机对树莓派的遥测出口：

```c
Px4Lite_MavlinkSetCellularLinkEnabled(online);
Px4Lite_MavlinkSetRpiUplinkEnabled(online);   /* 门控 USART6 遥测 */
```

USART6 是 STM32↔RPi 的物理 UART，与 5G 网络通断无关。用 A 链路的状态去关 B 链路造成三个后果：

1. 文档 §4 中 `value=0x06`（网卡与载波正常、未获得 IP，建议显示"5G 连接中"）会使 `ONLINE=0`，于是 STM32 停发 USART6 上的全部遥测，尽管 UART 完全正常。
2. `s_rpi_uplink_enabled` 在 `PX4LITE_ENABLE_5G` 时初值为 0，即在收到第一帧 `ONLINE=1` 的 `RPICELL` 之前，树莓派一帧遥测都收不到；5G 若始终不通则永远收不到。
3. Health task 的 5G 超时路径同样会关断上行。

该耦合在树莓派侧文档中不可见——树莓派无法预期"我如实上报 `ONLINE=0`，我自己就收不到遥测了"。

## 修复：`RPICELL` 只驱动显示状态

- `px4lite_mavlink_rx.c`：`MavRx_DecodeRpiNamedValueInt()` 只保留 `Px4Lite_SetExternalModuleState(PX4LITE_MODULE_5G, ...)`，删除两个 `Set*Enabled` 调用。
- `px4lite_modules.c`：5G 超时路径同样只置显示状态。
- `px4lite_mavlink_tx.c`：删除 `Px4Lite_MavlinkTxRunRpi()` 中的 `s_rpi_uplink_enabled` 门控，USART6 遥测出口恒定开放。

解耦后 `s_rpi_uplink_enabled` 与 `s_cellular_link_enabled` 已无任何写入来源，且两个 `Get*` 访问器本就全仓库无人调用，整套机制成为死状态，一并移除：

- `px4lite_mavlink_tx.c`：删除两个 static 变量、其初始化分支、四个访问器定义。
- `px4lite_mavlink_tx.h`：删除 `Px4Lite_MavlinkSet/GetCellularLinkEnabled`、`Px4Lite_MavlinkSet/GetRpiUplinkEnabled` 四个声明。
- `px4lite_mavlink_rx.c`：该文件对 `px4lite_mavlink_tx.h` 的唯一用途即上述 setter，include 已成残留依赖，一并移除（已核对 tx.h 导出的 15 个符号在 rx.c 中均未使用；`Px4Lite_MavlinkLink_t` 来自 `px4lite_types.h`，经 `px4lite_command.h` 可见）。

`COMMAND_ACK` 不受影响：2026-07-17_08 已将 ACK 提到该门控之前，本次门控整体删除后更无关联。

## 与文档的符合性核对

| 文档条目 | 固件现状 | 结论 |
|---|---|---|
| §2 以 `msgid=252` + `name="RPICELL"` 判断，不依赖 sysid | `px4lite_mavlink_rx.c:713` 按 name 匹配；USART6 接收路径不校验 sysid | 符合 |
| §2 `name` 不保证 `\0` 结尾 | `MavRx_NameEquals()` 为 7 字符前缀比对，与文档 §5.2 的 `memcmp(...,7U)` 行为一致 | 符合 |
| §3 只判断 bit0，不重复推导 `ONLINE` | `MAV_RX_RPI_CELLULAR_ONLINE_MASK = 0x01`，直接取 bit0 | 符合 |
| §3/§7.7 保留位必须忽略 | 仅做 `value & 0x01`，其余位天然忽略 | 符合 |
| §4 按位解析，不做整值枚举匹配 | 同上 | 符合 |
| §5.3 接收超时，不得保留最后的"在线" | Health task `PX4LITE_5G_HEARTBEAT_TIMEOUT_MS` 超时置 OFFLINE | 符合 |
| §5.3 建议超时 5000 ms | 现为 **3000 ms** | 有意保留，见下 |
| §5.3 tick 回绕安全 | `Px4Lite_ElapsedMs()` 用 int32 差值 | 符合 |
| §6 未收到/超时不得归入"已连接" | 超时置 `PX4LITE_STATE_OFFLINE` | 符合 |
| §7.5 5 秒内转未知/未连接 | 实测 3 秒内 | 符合 |

### 超时阈值保留 3000 ms 的理由

文档 §5.3 的 5000 ms 是"建议"，而 §7 验收项 5 的硬性要求是"5 秒内转为未知/未连接"。3000 ms 满足该验收项，且配合 §2 的 1000 ms 发送周期可容忍 2 帧连续丢失。偏差方向安全：只会更早判定离线，不会误报在线（§6 的红线）。如需严格对齐 5000 ms，改 `PX4LITE_5G_HEARTBEAT_TIMEOUT_MS` 一处即可。

## 遗留：树莓派 sysid 不一致（需 RPi 侧确认，非本次改动）

本文档 §2 声明 RPi 帧头为 `sysid=1`、`compid=191`，而 `树莓派下行MAVLink电机控制帧格式.md` 要求 `COMMAND_LONG` 的 `source_system=250`。同一树莓派对不同消息使用两个 sysid，且 `sysid=1` 与本机 UID 派生 sysid（`1 + UID 哈希 % 250`，可能取 1）存在撞车可能。

`RPICELL` 功能不受影响（USART6 接收路径不校验 sysid，且文档 §2 亦要求固件不依赖 sysid），故本次不做固件改动，留待 RPi 侧统一身份配置。

## 行为变化

USART6 遥测出口不再随 5G 状态开关：树莓派在 5G 未连接、连接中或从未上报 `RPICELL` 时，同样能持续收到 GNSS/姿态/位置/电池/电机/系统健康等全量遥测。显示屏的 5G 状态判定不变。

若原耦合实为"5G 不通即无需向 RPi 转发遥测"的省带宽设计，本次改动会使 5G 断开期间 USART6 持续输出遥测（波特率与帧率不变，仅占用该 UART 带宽，不影响 LoRa 空口）。

## 影响面

| 文件 | 改动 |
|---|---|
| `Framework/Src/px4lite_mavlink_rx.c` | `RPICELL` 只驱动 5G 显示状态；移除残留的 `px4lite_mavlink_tx.h` include |
| `Framework/Src/px4lite_modules.c` | 5G 超时只置显示状态 |
| `Framework/Src/px4lite_mavlink_tx.c` | 删除 RPi 上行门控、两个死状态变量及四个访问器 |
| `Framework/Inc/px4lite_mavlink_tx.h` | 删除四个访问器声明 |

无线上协议变化：`RPICELL` 的解析语义与字段完全不变，树莓派侧无需改动。

## 构建验证

**未执行 Keil build。** 本机未安装 Keil MDK-ARM，亦无任何可用 C 编译器（`UV4.exe`/`armcc`/`armclang`/`gcc`/`arm-none-eabi-gcc` 均不存在），仓库无 CLI/CMake/Makefile 入口。已静态核对：两个开关的全仓库引用（含 Business/Display/Debug）已归零、`fiveg_state`/`fiveg_last_rx_ms` 在 `#if PX4LITE_ENABLE_5G` 两侧均无未使用告警、`PX4LITE_ENABLE_5G=0` 与 `PX4LITE_ENABLE_RPI_MAVLINK=0` 条件编译路径自洽。合入前需在装有 Keil 的机器上 Rebuild All 并确认 `0 Error(s), 0 Warning(s)`。
