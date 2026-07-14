# 树莓派 USART6 下行接收 RAM 优化说明

## 1. 背景

在补齐树莓派 USART6 下行接收链路后，Keil 链接阶段出现 RAM 空间不足错误：

```text
Error: L6406E: No space in execution regions with .ANY selector
Error: L6407E: Sections of aggregate size 0x9a4 bytes could not fit into .ANY selector(s).
".\Objects\formal_framework.axf" - 102 Error(s), 0 Warning(s).
Target not created.
```

该错误不是 C 语法错误，也不是 MAVLink 帧格式错误，而是链接器无法把 `.data` / `.bss` 运行时变量区放入当前 SRAM 执行区。

## 2. 原因分析

当前 Keil scatter 文件只把普通 RW/ZI 数据放入 128KB SRAM1：

```text
RW_IRAM1 0x20000000 0x00020000
```

也就是：

```text
起始地址：0x20000000
大小：0x20000 = 128KB
```

虽然 STM32F407 还有 64KB CCM RAM：

```text
0x10000000 起始，大小 64KB
```

但当前自动生成 scatter 文件没有把普通变量分配到 CCM，因此链接器只能使用 128KB SRAM1。

补齐 USART6/RPi 下行接收链路时，新增了以下静态 RAM 消耗：

```text
1. USART6 RX 软件环形缓冲
2. RPi MAVLink 解析状态
3. RPi MAVLink 接收缓存
```

旧实现中，RPi MAVLink 接收缓存使用 `Px4Lite_CommRxFrame_t[8]` 队列。由于单个 `Px4Lite_CommRxFrame_t` 内含 255 byte payload 副本，8 帧队列会额外占用约 2KB 以上 SRAM。工程原本 SRAM1 余量已经很小，因此触发 `L6406E`。

## 3. 优化原则

本次优化遵循以下原则：

```text
1. 不改 LoRa、LVGL、FreeRTOS、Storage、Sensor 等既有模块。
2. 不移动 DMA buffer，避免违反 DMA 只能使用 0x20000000 SRAM 的规则。
3. 保留树莓派 USART6 下行接收、MAVLink 解析、31011/31090 电机控制和 ACK 回包能力。
4. 只压缩本次新增 RPi 下行接收链路的静态 RAM。
5. 树莓派下行命令按低频请求/ACK 模式使用，不支持高频突发批量积压。
```

## 4. 修改内容

### 4.1 USART6 RX 环形缓冲缩小

文件：

```text
Bsp/Inc/bsp_config.h
```

修改：

```c
#define BSP_RPI_RX_BUF_SIZE   128U
```

原设计为 512 byte。考虑到当前下行只接收 MAVLink `COMMAND_LONG` 命令，典型帧长度远小于 128 byte，且通信任务周期为 10ms，128 byte 软件缓冲足够覆盖正常命令下发。

### 4.2 RPi MAVLink 接收队列改为单帧待处理缓存

文件：

```text
Framework/Src/px4lite_platform_f407.c
```

优化前：

```text
static Px4Lite_CommRxFrame_t s_rpi_rx_queue[8];
```

优化后：

```text
static mavlink_status_t s_rpi_parse_status;
static mavlink_message_t s_rpi_parse_msg;
static uint32_t s_rpi_pending_rx_ms;
static uint8_t s_rpi_pending_frame;
```

行为变化：

```text
1. RPi MAVLink parser 收到完整帧后，直接将该帧保存在 s_rpi_parse_msg。
2. 设置 s_rpi_pending_frame = 1。
3. Px4Lite_RpiMavlinkCopyRxFrame() 被调用时，把 s_rpi_parse_msg 转换为 Px4Lite_CommRxFrame_t。
4. 转换完成后清除 s_rpi_pending_frame。
5. 若上一帧尚未被通信任务取走，Px4Lite_RpiMavlinkService() 暂停继续解析新帧，避免覆盖待处理命令。
```

这样取消了 8 帧 `Px4Lite_CommRxFrame_t` 静态队列，释放约 2KB 以上 SRAM。

## 5. 对功能的影响

### 5.1 不受影响的功能

```text
1. 本机屏幕滑条控制电机。
2. 树莓派 USART6 下发 31011 设置四路电机油门。
3. 树莓派 USART6 下发 31090 急停。
4. 本机电机页显示 Control 输出进度条。
5. ACK 通过 USART6 返回树莓派。
6. LoRa 远端数据接收和远端查看显示逻辑。
7. RPi 全量遥测 TX 出口。
```

### 5.2 使用边界

优化后的 RPi 下行接收链路适合“命令请求/ACK”模式：

```text
树莓派发送一条命令
  -> 等待 COMMAND_ACK
  -> 再发送下一条命令
```

不建议树莓派在极短时间内连续突发多条控制命令。若后续需要高频批量下行，应从工程级 RAM 布局入手，例如显式使用 CCM RAM 放置非 DMA 缓冲，而不是重新扩大 SRAM1 内的静态队列。

## 6. 验证记录

已完成：

```text
1. ARMCC 5.06 单文件编译检查通过：
   - Bsp/Src/bsp_uart.c
   - Framework/Src/px4lite_platform_f407.c
   - Framework/Src/px4lite_modules.c
   - Framework/Src/px4lite_mavlink_tx.c

2. git diff --check 通过，没有空白格式错误。
```

待复核：

```text
Keil UV4 命令行在当前环境中会卡住或不刷新日志。
请在 Keil μVision 中手动执行 Rebuild All，并确认：

0 Error(s), 0 Warning(s)
```

## 7. 后续建议

如果后续继续增加功能导致 SRAM1 再次接近上限，建议按以下顺序处理：

```text
1. 优先检查新增静态数组、接收队列、日志缓存和 LVGL 缓冲。
2. DMA buffer 必须留在 0x20000000 SRAM，不能放入 CCM。
3. 非 DMA、大块、低实时敏感缓存可考虑显式放入 0x10000000 CCM RAM。
4. 若引入 CCM 分区，应使用手写 scatter 文件并同步更新工程文档。
5. 不建议通过盲目关闭已有功能来解决 RAM 问题，除非产品功能确认不再需要。
```
