# LoRa 通信与远端显示同步修复说明

## 1. 文档目的

本文记录 2026-07-13 对 LoRa 通信、远端显示、消息日志、电机显示和告警同步链路的修复。改动目标是：

- LoRa 按 `fj-lora` 参考工程的在位与收发方式恢复。
- 树莓派 USART6 下行解析与 LoRa 空口解析分离，互不干扰。
- 电机、消息日志、告警信息能通过 LoRa 正常同步到远端显示端。
- 远端模式下，电机页和告警页显示的数据来源与当前显示模式一致。

## 2. 总体链路

LoRa 链路仍由 `comm` 任务统一驱动：

```text
Px4Lite_CommWorkRun()
  -> Px4Lite_RpiMavlinkService()
  -> Px4Lite_MavlinkRxRunRpi()
  -> Px4Lite_LoRaService()
  -> Px4Lite_MavlinkRxRun()
  -> Px4Lite_MavlinkTxRun()
  -> Px4Lite_MavlinkTxRunRpi()
```

关键点：

- RPi 解析只消费 USART6 帧，入口为 `Px4Lite_MavlinkRxRunRpi()`。
- LoRa 解析只消费 E22/USART3 空口帧，入口为 `Px4Lite_MavlinkRxRun()`。
- `Px4Lite_CopyCommRxFrame()` 只返回 LoRa 接收帧，避免树莓派数据混入 LoRa 远端遥测。
- RPi 专属上行仍走 `Px4Lite_MavlinkTxRunRpi()`，不受 LoRa 空口忙闲影响。

## 3. LoRa 在位与热插拔

LoRa 在位逻辑已按实板 AUX 电气特性完成修正：

- 本机 LoRa 灯色只反映 E22 本地硬件可用性。
- 是否收到远端帧只影响远端数据新鲜度和丢包统计，不把本机 LoRa 灯直接打红。
- `PF0/AUX` 常态使用无上下拉输入，避免内部下拉与模块弱上拉形成 `1.x V` 分压。
- 在位探测只在输入模式下完成“短时下拉释放电荷 -> 无上下拉采样”，不再主动驱动 AUX。
- E22 拔出后停止发送并变红；重新插入后由 LoRa 服务周期推进初始化、变绿并恢复发送。
- 上电已插、上电未插、运行中拔出、运行中插入和重复热插拔均已实板验证。

最终电气模型、状态流程、分层边界和验证记录见：

- `Development_Guide/Change_History/2026-07-14_01_LoRa热插拔主动在位探测.md`

涉及文件：

- `Sensor/Src/lora_e22.c`
- `Sensor/Inc/lora_e22.h`
- `Framework/Src/px4lite_modules.c`

## 4. LoRa 发送调度修复

之前电机、消息日志、告警信息不同步的主要原因是：

- `Px4Lite_CommWorkRun()` 只有在 `Px4Lite_LoRaService()` 返回 `PX4LITE_OK` 时才运行 `Px4Lite_MavlinkTxRun()`。
- LoRa 半双工收发过程中，服务层经常返回 `PX4LITE_BUSY`。
- 状态层已把 `BUSY` 视为正常半双工状态，但发送调度被 `OK` 门槛挡住。
- 电机、日志、告警属于调度表靠后的扩展帧，最容易长期排不上发送。

修复后：

```c
if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY)) {
  tx_result = Px4Lite_MavlinkTxRun(now_ms);
}
```

发送器内部仍会检查 `Px4Lite_LoRaIsTxIdle()`，所以该改动不会抢占正在发送的 DMA，也不会破坏半双工收发。

涉及文件：

- `Framework/Src/px4lite_modules.c`

## 5. LoRa 与 RPi 解析分离

树莓派下行解析与 LoRa 解析已经分开：

- `Px4Lite_MavlinkRxRun()`：处理 LoRa 空口远端遥测。
- `Px4Lite_MavlinkRxRunRpi()`：只处理树莓派下行控制类帧。

RPi 下行当前只处理：

- `COMMAND_LONG`
- `COMMAND_ACK`

RPi 下行不会解析或覆盖 LoRa 的远端遥测字段，例如电机、日志、告警、环境、姿态等，避免两条链路互相干扰。

涉及文件：

- `Framework/Src/px4lite_mavlink_rx.c`
- `Framework/Inc/px4lite_mavlink_rx.h`
- `Framework/Src/px4lite_platform_f407.c`
- `Framework/Inc/px4lite_platform.h`
- `Framework/Src/px4lite_modules.c`

## 6. LoRa 发送帧格式

LoRa 发送保持 `fj-lora` 兼容格式：

| 数据 | MAVLink 类型 | 名称/标识 | 说明 |
|---|---|---|---|
| 第二电池摘要 | `NAMED_VALUE_INT` | `BAT2STAT` | 电压、电量百分比、低压标志 |
| 告警最高项 | `NAMED_VALUE_INT` | `ALRMHI` | 最高故障码、来源、严重度 |
| 告警活动位图 | `NAMED_VALUE_INT` | `ALRMMSK` | 活动告警模块位图 |
| 完整告警表 | `TUNNEL` | `payload_type=0x8001` | 每条告警的 source/fault/severity |
| 电机 1/2 | `NAMED_VALUE_INT` | `MOTOR12` | 1、2 路油门、运行状态、速度等级 |
| 电机 3/4 | `NAMED_VALUE_INT` | `MOTOR34` | 3、4 路油门、运行状态、速度等级 |
| 电机脉宽 | `SERVO_OUTPUT_RAW` | `port=0` | `servo1_raw`~`servo4_raw` 为四路 PWM 高电平脉宽，单位 us |
| 消息日志 | `NAMED_VALUE_INT` | `LOGSYNC` | 单条日志增量同步 |

告警发送采用三段轮询：

```text
ALRMHI -> ALRMMSK -> TUNNEL 0x8001 -> ALRMHI ...
```

这样既保留 `fj-lora` 摘要兼容，又能让新接收端显示完整告警列表。

涉及文件：

- `Framework/Src/px4lite_mavlink_tx.c`
- `Framework/Src/px4lite_mavlink_rx.c`

## 7. 电机远端显示修复

问题现象：

- LoRa 已经收到远端电机数据。
- 但电机页滑条不显示远端值。

原因：

- 电机页存在特例：当前页是电机页时强制调用 `App_CopyMotor()` 读取本机 Control topic。
- 因此即使处于远端模式，电机页仍看本机数据。

修复：

- 电机页和其它页面统一调用 `App_GetDisplayMotor()`。
- 本地模式显示本机电机，远端模式显示远端电机。

涉及文件：

- `Display/Src/display.c`
- `Business/Src/app_display_model.c`
- `Business/Src/app_data_api.c`

## 8. 告警远端显示修复

问题现象：

- 远端告警只能显示一条或显示不全。

原因：

- 远端聚合快照 `App_CopyRemoteDisplaySnapshot()` 原来只把最高告警填入 `alarms[0]`。
- `ALRMHI/ALRMMSK` 本身也只能表达最高告警和位图，不能提供每条告警的故障码和严重度。

修复：

- LoRa 发送端补发完整告警表 `TUNNEL 0x8001`。
- 接收端已有 `MavRx_DecodeTunnel()` 解码完整表，写入：
  - `alarm_active_mask`
  - `alarm_fault_code[source]`
  - `alarm_severity[source]`
- `App_CopyRemoteDisplaySnapshot()` 按 `alarm_active_mask` 展开每个活动来源，填充 `out->alarms[]`。
- 在完整表尚未收到时，仍使用最高告警作为兜底，避免上电短暂空白。

涉及文件：

- `Framework/Src/px4lite_mavlink_tx.c`
- `Framework/Src/px4lite_mavlink_rx.c`
- `Business/Src/app_data_api.c`
- `Display/Src/display.c`

## 9. 消息日志同步

消息日志 LoRa 同步继续使用 `LOGSYNC`：

- 本机业务日志由 `App_MessageLogUpdate()` 产生。
- 日志进入 Framework 本机日志缓冲。
- `MavTx_SendMessageLog()` 从本机日志缓冲按序号取出并发送 `LOGSYNC`。
- 远端接收端按 `sequence` 去重，避免周期重发造成重复行。

本次同步卡住的根因同样受 LoRa 发送调度门槛影响；允许 `BUSY` 状态下进入发送调度后，日志帧能更快排上发送。

涉及文件：

- `Business/Src/app_message_log.c`
- `Framework/Src/px4lite_mavlink_tx.c`
- `Framework/Src/px4lite_mavlink_rx.c`
- `Business/Src/app_display_model.c`

## 10. RAM 与 MAVLink 通道约束

为了避免 SRAM 不足：

- MAVLink 发送侧保持 `MAVLINK_COMM_NUM_BUFFERS = 1`。
- RPi 不再启用第二个 MAVLink 全局 channel buffer。
- RPi 帧复用 `COMM_0` 编码后，发送前重贴 RPi 独立序号和 `compid=193`，并重算 CRC。
- LoRa 和 RPi 解析状态分布在不同源文件的静态解析器中，逻辑分离，不依赖第二个 MAVLink 全局 buffer。

该策略解决了启用第二通道后出现的 `L6406E No space in execution regions` 链接错误。

涉及文件：

- `Framework/Src/px4lite_mavlink_tx.c`
- `Framework/Src/px4lite_platform_f407.c`

## 11. 验证结果

已执行 Keil Rebuild All：

```text
D:\keil5\UV4\UV4.exe -r MDK-ARM/formal_framework.uvprojx -t "Target 1" -j0
```

构建结果：

```text
Program Size: Code=292372 RO-data=100556 RW-data=1544 ZI-data=129376
0 Error(s), 0 Warning(s)
```

## 12. 后续注意事项

- 如果后续增加 5G 在线判定，应按模块状态接入，不要复用 LoRa/RPi 的解析路径。
- LoRa 空口新增字段时，应优先在 `px4lite_mavlink_tx.c` 和 `px4lite_mavlink_rx.c` 成对维护。
- RPi 专属数据继续放在 `Px4Lite_MavlinkTxRunRpi()` 的专属目录，不要混入 LoRa 调度表。
- 电机页、告警页、消息日志页应继续通过 `app_display_model` 或 `app_data_api` 取数，不要直接读 Framework 远端遥测结构。
- 若再次出现“收到但不显示”，优先检查显示模式、本地/远端取数入口和 `valid_mask/stale_mask`，再查协议收发。
