# 2026-07-20_01 树莓派身份帧 BASIC_ID 与 ESP32 在位解耦

## 现象

树莓派侧抓包（`/dev/ttyUSB5`）显示 USART6 上持续有大量有效 MAVLink2 帧：HEARTBEAT、HUMIDITY、MOTOR12/34、LORASTAT/LORATX/LORARX、RIDSTAT 均可读，**唯独没有 `OPEN_DRONE_ID_BASIC_ID`（msgid 12900，线路头小端 `64 32 00`）**。RPi 无法从 `uas_id` 提取 vendor_id，按现有设计不创建 MQTT 客户端，故连不上 broker。

## 根因

BASIC_ID 原先并非独立发送，而是"搭 ESP32 出口的便车"镜像给 RPi：

`px4lite_remoteid_tx.c` 的 `RemoteId_SendPrepared()` 在发往 UART4 前调用 `Px4Lite_MavlinkTxMirrorToRpi()`。而该函数的整条调用链 `Px4Lite_RemoteIdTxRun()` 被 ESP32 在位状态门控：

```c
/* px4lite_modules.c */
if (remoteid_present == 0U) {                 /* remoteid_present 来自 PC8 硬件电平 */
  Px4Lite_SetStatus(PX4LITE_MODULE_REMOTE_ID, PX4LITE_STATE_FAILED, ...);
} else {
  ...
  (void)Px4Lite_RemoteIdTxRun(now_ms);        /* 只有这里才会编码 BASIC_ID */
}
```

**ESP32-S3 未插/未上电（PC8 低）→ TxRun 一次不跑 → BASIC_ID 从未被编码 → 无帧可镜像。** RIDSTAT 由 `px4lite_mavlink_tx.c` 的 RPi 专属目录独立发送，与 ESP32 无关，故仍可见——与抓包现象完全吻合。

这是一个隐性跨模块依赖：RemoteID 广播（ESP32）与 5G/MQTT 上云本是两件事，却被绑死，拔掉 ESP32 即上不了云。

## 修改

新增一条**不受 ESP32 在位影响**的 RPi 身份通路，`s_uas_id`/`s_id_or_mac` 在 `Px4Lite_RemoteIdTxInit()` 开机时已填好，与 ESP32 无关，故可独立发送。

| 文件 | 修改 |
|---|---|
| `Framework/Src/px4lite_remoteid_tx.c` | 抽出 `RemoteId_PackBasicId()`，ESP32 出口与 RPi 出口共用，保证载荷逐字节一致；新增 `Px4Lite_RemoteIdTxRunRpiIdentity()`，自带 1Hz 节拍 `s_next_rpi_basic_id_ms`，只经 `MirrorToRpi` 写 USART6，不碰 UART4；`RemoteId_SendPrepared()` 的镜像条件排除 BASIC_ID，避免 ESP32 在位时重复发送。 |
| `Framework/Inc/px4lite_remoteid_tx.h` | 声明 `Px4Lite_RemoteIdTxRunRpiIdentity()`，注明必须在在位判定之外调用。 |
| `Framework/Src/px4lite_modules.c` | 在 `remoteid_present` 判定**之前**无条件调用该函数。 |

## 对树莓派侧的影响：无

按要求"发给树莓派的数据不能变"：

| 维度 | 改前（ESP32 在位时） | 改后 |
|---|---|---|
| msgid | 12900 | 12900 |
| sysid | `Px4Lite_IdentityGetMavlinkSystemId()` | 同 |
| compid | 193（MirrorToRpi 改写） | 同 |
| 载荷字段 | `RemoteId_SendBasicId` 所packed | 共用 `RemoteId_PackBasicId()`，同 |
| 周期 | 1Hz（`PX4LITE_REMOTEID_BASIC_ID_PERIOD_MS`） | 同 |
| 序号 | `s_rpi_tx_seq` | 同 |

ESP32 在位时 RPi 收到的 BASIC_ID 与改前逐字节一致、频率一致，仅发送来源从"镜像"变为"独立调度"；**ESP32 不在位时由无变为照常 1Hz**，即本次要修的缺口。

ESP32/UART4 出口行为、其余 OPEN_DRONE_ID_*（LOCATION/SYSTEM/OPERATOR_ID/SELF_ID）镜像、LoRa 侧均未改动。

## 遗留

其余 OPEN_DRONE_ID_* 仍随 ESP32 在位门控（与改前一致）。若 RPi 后续还需要 LOCATION/SYSTEM 等在 ESP32 缺席时可用，需另行评估。

## 构建验证

**未执行 Keil build。** 本机无任何可用 C 编译器（`UV4.exe`/`armcc`/`armclang`/`gcc`/`arm-none-eabi-gcc` 均不存在），仓库无 CLI/CMake/Makefile 入口。已静态核对：新函数在 `#if PX4LITE_ENABLE_REMOTE_ID` 内定义、`#else` 分支有同名桩；`PX4LITE_ENABLE_RPI_MAVLINK` 关闭时函数体走 `(void)now_ms` 空实现，无未使用变量；`s_next_rpi_basic_id_ms` 已在 Init 中初始化；`RemoteId_PackBasicId` 定义先于两处引用；`px4lite_modules.c` 已 include `px4lite_remoteid_tx.h`。

合入前需在装有 Keil 的机器上 Rebuild All 确认 `0 Error(s), 0 Warning(s)`，并实测：
1. **拔掉 ESP32**，RPi 抓包应看到 `64 32 00` 身份帧 1Hz，MQTT 客户端正常创建；
2. **插上 ESP32**，BASIC_ID 仍为 1Hz **不得翻倍**（验证镜像排除生效），且 ESP32 广播正常。
