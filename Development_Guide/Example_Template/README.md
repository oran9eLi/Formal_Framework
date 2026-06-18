# 数据完整性例程模板

本目录提供可复制到正式模块的基础例程，默认面向 STM32F407 +
FreeRTOS + ARMCC5。

## 文件功能

| 文件 | 功能 |
|---|---|
| `integrity_types.h` | 统一结果、状态、测量头和统计信息 |
| `integrity_snapshot.*` | 小型低频数据的完整快照发布/复制 |
| `integrity_fifo.*` | 高频数据固定 FIFO，满时丢最旧并计数 |
| `integrity_frame.*` | 帧编码、CRC、流式接收和完整帧判定 |
| `integrity_tx.*` | 处理底层部分发送、超时和完成确认 |
| `sensor_module_template.*` | 新传感器驱动和发布流程模板 |

## 使用边界

- Snapshot：GNSS、气压、电源、状态等小结构体。
- FIFO：IMU 等高频且每个样本都需要处理的数据。
- Frame：LoRa、5G、UART、Linux 链路和日志记录。
- 大于 256 字节的数据不使用 Snapshot 复制，改用固定块内存池。

模板中的 BSP 函数是明确的适配点，开发者必须替换为实际硬件实现。
