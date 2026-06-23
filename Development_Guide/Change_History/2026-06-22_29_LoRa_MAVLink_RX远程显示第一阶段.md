# 2026-06-22 变更记录：LoRa MAVLink RX 远程显示第一阶段

## 变更内容

- 新增 Framework `px4lite_remote_telemetry` 模块，维护 `LOCAL` / `REMOTE` 显示源模式、目标 `sysid`、远端字段有效位和远端遥测快照。
- 新增 Framework `px4lite_mavlink_rx` 模块，在 `REMOTE` 模式下解码 LoRa 接收到的 MAVLink 消息，并写入 `RemoteTelemetry`。
- MAVLink RX 第一阶段支持 `HEARTBEAT`、`GPS_RAW_INT`、`GLOBAL_POSITION_INT`、`ATTITUDE`、`SYS_STATUS`、`BATTERY_STATUS`、`SCALED_PRESSURE` 和 `STATUSTEXT`。
- Comm 任务新增 KEY0 边沿切换：默认 `LOCAL`，按 KEY0 切换到 `REMOTE`；`LOCAL` 只主动发送本机 MAVLink 遥测，`REMOTE` 停止本机周期遥测并开始处理远端 MAVLink 数据。
- 平台适配层新增 `Px4Lite_LoRaCopyRxFrame()`，Framework 模块不直接包含 LoRa 驱动头文件，保持平台适配边界。
- Business 层新增 `App_GetRemoteDisplayMode()` 和 `App_CopyRemoteTelemetry()`，Display 继续通过 `app_data_api.h` 读取数据。
- Display 头部左上角新增 `LOCAL` / `REMOTE` 标识；REMOTE 模式下页面主字段从远端快照加载，远端过期时清空远端字段，不用本机传感器数据冒充远端数据。
- Keil 工程 `MDK-ARM/formal_framework.uvprojx` 已加入 `px4lite_mavlink_rx.c` 和 `px4lite_remote_telemetry.c`。
- 新增桌面侧单元测试 `Tests/Unit/test_mavlink_rx_remote_telemetry.c`，覆盖远端 `sysid` 过滤、MAVLink 解码、远端快照写入、LOCAL 模式忽略远端数据和超时返回。

## 边界说明

- 第一阶段不引入自定义 LoRa 裸帧格式，不实现 ACK、重发、绑定握手或分时双工调度。
- 远程设备识别第一阶段使用 MAVLink `sysid`；Remote ID 和 `link_device_id` 后续再接入。
- REMOTE 模式停止的是本机周期遥测主动发送，不代表后续必要 ACK 或控制确认帧永远禁止发送。
- 当前远端时间、湿度、电机状态、完整模块状态和完整告警表仍需第二阶段通过 MAVLink 扩展或其他载荷补齐。
- 当前只记录 RX、过滤和未支持消息统计，尚未把模式切换、远端超时等事件写入日志模块。

## 验证记录

- 桌面侧单元测试已通过：`Tests/Unit/test_mavlink_rx_remote_telemetry.c`，输出 `mavlink rx remote telemetry tests passed`。
- Keil MDK-ARM Rebuild All 已通过：`MDK-ARM/build_lora_remote_mavlink_rx.log`，结果为 `0 Error(s), 0 Warning(s)`。
