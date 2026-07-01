# 2026-07-01 LoRa / RemoteID 同时启用状态闭环

## 改动

- RemoteID 按当前产品需求与 LoRa 同时启用：LoRa 继续负责板间 MAVLink 主从遥测、远端选择和显示闭环；RemoteID 通过 UART4 TX DMA 向 ESP32-S3 提交 MAVLink/OpenDroneID 消息。
- 主从身份宏收口为单一烧录入口：`PX4LITE_PAIR_MASTER_ID` 表示配对主机，默认 1；`PX4LITE_BOARD_UNIT_ID` 表示当前板编号，1 自动为主机，2/3/4 自动为从机。`PX4LITE_MASTER_NODE_ID`、`PX4LITE_UNIT_ID`、`PX4LITE_NODE_ROLE`、`PX4LITE_NODE_ID` 和 `PX4LITE_MAVLINK_SYSTEM_ID` 均由这两个宏派生。
- BSP RemoteID 增加本地 ready 事实，未初始化时拒绝发送；Framework 记录最近成功提交、忙和错误时间。
- RemoteID descriptor 接入 registry recovery。Health 侧 recover 只置位请求，真正 abort、BSP init 和发送调度复位由 `comm` task 执行。
- RemoteID 状态口径明确为 STM32 本地发送通道健康状态：初始化后 STARTING，成功提交帧后 ONLINE，提交失败或 DMA 忙超时后 OFFLINE 并请求恢复。
- Business 本机消息日志入口收口到 `app_data_api.h`，普通 Business 日志模块不再直接调用 Framework `px4lite_local_msglog`。
- LoRa Driver 丢包率最小样本阈值改为 Driver 本地常量，避免 Driver 层依赖 Framework 配置宏。

## 边界

- RemoteID ONLINE 不代表 ESP32-S3 已收到、已解析或已完成 BLE/Wi-Fi 广播；真实广播确认必须后续增加 GPIO presence、UART RX ACK 或 ESP32 心跳。
- RemoteID 不新增任务、不扩大 `PX4LITE_STACK_COMM`；发送缓冲、统计和 scratch 均保持文件级静态存储。
- LoRa 与 RemoteID 物理链路独立：LoRa 走 USART3/E22 半双工空口，RemoteID 走 UART4 到 ESP32-S3，不共用 DMA 或空口预算。

## 验证

- Keil Rebuild All 已通过：`.\Objects\formal_framework.axf - 0 Error(s), 0 Warning(s)`。
- Program Size：`Code=268900 RO-data=91540 RW-data=1368 ZI-data=128504`。
- Build Time Elapsed：`00:00:32`。
