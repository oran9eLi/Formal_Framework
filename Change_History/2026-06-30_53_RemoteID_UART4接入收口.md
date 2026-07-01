# 2026-06-30 RemoteID UART4 接入收口

## 变更摘要

- RemoteID 对外身份改为独立配置：`PX4LITE_REMOTEID_UAS_ID`、`PX4LITE_REMOTEID_OPERATOR_ID` 和 `PX4LITE_REMOTEID_MAVLINK_SYSTEM_ID`。
- LoRa `node_id/system_id` 不再承担对外身份职责，仅保留调试、内部路由和遥测链路识别用途。
- 新增 UART4 PC10/PC11、DMA1 Stream4 TX 的 ESP32-S3 RemoteID BSP 发送链路。
- 新增 Framework RemoteID TX 调度器，复用 `comm` task 发送 MAVLink/OpenDroneID 消息。
- Keil 工程 source list 已加入 `Bsp/Src/bsp_remoteid.c` 和 `Framework/Src/px4lite_remoteid_tx.c`。

## 架构边界

- BSP 只做 UART4、DMA、NVIC 和原始发送，不解析 RemoteID 协议。
- Platform Adapter 是 Framework 到 BSP 的唯一转换入口。
- Framework 逐字段编码 MAVLink/OpenDroneID，不发送 C struct 内存。
- ESP32-S3 负责 BLE/Wi-Fi 广播和监管格式适配，STM32 只提交身份、位置和系统事实。

## 栈和内存

- 本次没有新增 FreeRTOS task。
- 本次没有扩大 `PX4LITE_STACK_COMM`。
- RemoteID frame buffer、身份字段、MAVLink message、Navigation scratch 和统计计数均为文件级静态存储。
- `comm` task 每次调度最多提交一帧，UART4 DMA busy 时按固定重试周期退让。

## 验证结果

- Keil Rebuild All 日志：`MDK-ARM/build_remoteid_uart4.log`
- Program Size: `Code=266988 RO-data=91488 RW-data=1344 ZI-data=127928`
- 结果：`0 Error(s), 0 Warning(s)`

## 后续未完成项

- 国家/ASTM 完整合规字段、实名注册号、操作者真实位置仍需结合产品合规要求补齐。
- ESP32 广播状态回读和接收端实测验证仍需后续联调。
- RemoteID 显示页只应展示身份和广播状态，不应绕过 App API 读取底层链路。
