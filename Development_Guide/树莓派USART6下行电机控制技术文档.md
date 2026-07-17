# 树莓派 USART6 下行电机控制技术文档

## 1. 文档目的

本文档说明树莓派通过 USART6 向 STM32F407 下发 MAVLink 电机控制命令的完整技术方案，包括物理链路、MAVLink 帧格式、固件接收解析链路、电机控制行为、本机屏幕显示策略、ACK 回包规则和联调验证方法。

本方案用于满足以下控制需求：

```text
1. 本机屏幕滑条可以直接控制四路电机 PWM。
2. 树莓派可以通过 USART6 下发 MAVLink 命令控制四路电机 PWM。
3. 最近一次有效控制永久保持，直到被新的本机滑条命令、树莓派命令、急停、保护逻辑或复位覆盖。
4. 本机电机页进度条显示本机 Control 输出，用于确认树莓派下行命令是否实际生效。
5. 本机电机页显示策略不影响远端数据接收和远端查看显示逻辑。
```

## 2. 总体链路

### 2.1 下行控制链路

```text
树莓派
  -> USART6 PC7(RX) 原始字节
  -> Bsp/Src/bsp_uart.c 环形缓冲
  -> Framework/Src/px4lite_platform_f407.c MAVLink 组帧
  -> Framework/Src/px4lite_modules.c 优先取 RPi 接收帧
  -> Framework/Src/px4lite_mavlink_rx.c COMMAND_LONG 解码
  -> Framework/Src/px4lite_command.c 命令分发
  -> Framework/Src/px4lite_control.c 目标油门锁存和 PWM 输出
  -> Business/Inc/app_data_api.h 本机电机快照
  -> Display/Src/display.c 本机电机页进度条显示
```

### 2.2 ACK 回包链路

```text
px4lite_command.c
  -> Px4Lite_MavlinkQueueCommandAck()
  -> Framework/Src/px4lite_mavlink_tx.c 判断 target_component/source_component
  -> component id 为 193 的 ACK 走 RPi 专属 USART6 出口
  -> 树莓派接收 COMMAND_ACK
```

## 3. 物理接口参数

| 项目 | 配置 |
|---|---|
| MCU 串口 | USART6 |
| STM32 TX | PC6 |
| STM32 RX | PC7 |
| 波特率 | 115200 |
| 数据格式 | 8N1 |
| 电平 | 3.3V TTL |
| 流控 | 无 |
| RX 模式 | USART6 中断单字节接收 + 128 byte 软件环形缓冲 |
| TX 模式 | 阻塞发送 |

接线要求：

```text
树莓派 TX -> STM32 PC7(USART6_RX)
树莓派 RX <- STM32 PC6(USART6_TX)
GND       <-> GND
电平      = 3.3V TTL
```

## 4. MAVLink 身份约定

树莓派发送 `COMMAND_LONG` 时建议使用：

| 字段 | 建议值 | 说明 |
|---|---:|---|
| source_system | 250 | 必须非 0，且不能等于 STM32 本机 system id |
| source_component | 193 | RPi 专属 component id，用于 ACK 路由 |
| target_system | 0 或本机 system id | 0 表示广播 |
| target_component | 0 或 191 | 191 为 STM32 MAVLink component id |

关键规则：

```text
1. source_system 不能为 0。
2. source_system 不能等于 STM32 本机 system id，否则会被接收层当作本机帧忽略。
3. source_component 建议固定为 193；这样 ACK 会从 USART6 返回树莓派。
4. 联调阶段 target_system=0,target_component=0 最省事。
```

## 5. 下行命令定义

### 5.1 命令 31011：设置四路电机油门百分比

`31011` 用于一次性设置四路电机目标油门百分比。

| COMMAND_LONG 字段 | 值 | 说明 |
|---|---:|---|
| command | 31011 | 设置四路电机油门百分比 |
| param1 | 0.0..100.0 | 电机 1 目标油门百分比 |
| param2 | 0.0..100.0 | 电机 2 目标油门百分比 |
| param3 | 0.0..100.0 | 电机 3 目标油门百分比 |
| param4 | 0.0..100.0 | 电机 4 目标油门百分比 |
| param5 | 0.0 | 当前未使用 |
| param6 | 0.0 | 当前未使用 |
| param7 | 0.0 | 当前未使用 |

控制语义：

```text
1. 参数单位是百分比，不是微秒脉宽。
2. 固件将 0..100% 映射为 ESC 1000..2000us。
3. 命令有效后目标油门锁存保持，不要求树莓派周期重复发送。
4. 后续本机滑条或树莓派新命令按“最后一次有效命令生效”覆盖旧目标。
```

PWM 映射关系：

```text
pulse_us = 1000 + (2000 - 1000) * percent / 100

0%   -> 1000 us
50%  -> 1500 us
100% -> 2000 us
```

示例：四路电机设置为 50%。

```text
msgid            = COMMAND_LONG(76)
source_system    = 250
source_component = 193
target_system    = 0
target_component = 0
command          = 31011
confirmation     = 0
param1           = 50.0
param2           = 50.0
param3           = 50.0
param4           = 50.0
param5           = 0.0
param6           = 0.0
param7           = 0.0
```

### 5.2 命令 31090：电机急停

`31090` 用于锁存急停并立即撤销四路电机输出。

| COMMAND_LONG 字段 | 值 | 说明 |
|---|---:|---|
| command | 31090 | 电机急停 |
| param1..param7 | 0.0 | 当前未使用 |

急停行为：

```text
1. 锁存急停状态。
2. 清零四路目标油门。
3. ESC 状态退回未解锁。
4. 立即调用 Px4Lite_MotorDisarmAll()。
5. 后续非 0 油门命令会被临时拒绝，直到恢复/复位逻辑解除急停。
```

## 6. 固件实现落点

### 6.1 BSP UART

文件：

```text
Bsp/Inc/bsp_config.h
Bsp/Inc/bsp_uart.h
Bsp/Src/bsp_uart.c
```

职责：

```text
1. USART6 仍保持阻塞 TX，用于树莓派全量遥测和 ACK 出口。
2. USART6 RX 使用中断单字节接收。
3. 接收到的原始字节写入 BSP 软件环形缓冲。
4. BSP 不解析 MAVLink，不理解电机命令。
```

### 6.2 Core 中断和 MSP

文件：

```text
Core/Src/stm32f4xx_hal_msp.c
Core/Src/stm32f4xx_it.c
```

职责：

```text
1. 初始化 USART6 PC6/PC7 GPIO。
2. 启用 USART6 NVIC。
3. 新增 USART6_IRQHandler()，转发到 BSP_RpiUART_IrqHandler()。
```

### 6.3 Platform Adapter

文件：

```text
Framework/Inc/px4lite_platform.h
Framework/Src/px4lite_platform_f407.c
```

职责：

```text
1. 从 BSP_RpiUART_Read() 读取 USART6 原始字节。
2. 使用 MAVLink parser 解析完整 MAVLink 帧。
3. 将 MAVLink message 转换为 Px4Lite_CommRxFrame_t。
4. 使用 RPi 单帧邮箱缓存已解析帧；树莓派控制命令按低频请求/ACK 模式发送，不要求高频批量突发。
```

### 6.4 Comm Task

文件：

```text
Framework/Src/px4lite_modules.c
```

职责：

```text
1. 每个通信周期先调用 Px4Lite_RpiMavlinkService(now_ms)。
2. Px4Lite_CopyCommRxFrame() 优先取 RPi 接收帧。
3. 如果没有 RPi 帧，再回退读取 LoRa 接收帧。
4. LoRa 远端数据接收逻辑保留。
```

### 6.5 Command 和 Control

文件：

```text
Framework/Src/px4lite_mavlink_rx.c
Framework/Src/px4lite_command.c
Framework/Src/px4lite_control.c
Framework/Inc/px4lite_control.h
```

职责：

```text
1. MAVLink RX 解码 COMMAND_LONG。
2. Command 层识别 31011/31090。
3. 31011 调用 Px4Lite_ControlSetMotorThrottlePercent()。
4. 31090 调用 Px4Lite_ControlEmergencyStop()。
5. Control 层是电机输出唯一业务 owner。
```

### 6.6 Display

文件：

```text
Display/Src/display.c
```

职责：

```text
1. 本机电机页固定读取 App_CopyMotor()。
2. 电机页显示本机 Control 输出，不跟随远端查看模式切换数据源。
3. 其他页面仍保留 App_GetDisplayMotor() 的本机/远端显示策略。
```

## 7. ACK 规则

STM32 接收并处理属于本机的 `COMMAND_LONG` 后，会排队发送 `COMMAND_ACK`。

| result | 数值 | 含义 |
|---|---:|---|
| MAV_RESULT_ACCEPTED | 0 | 命令已接受 |
| MAV_RESULT_TEMPORARILY_REJECTED | 1 | 当前急停锁存或 Control 忙 |
| MAV_RESULT_DENIED | 2 | 参数非法，例如油门超出 0..100 |
| MAV_RESULT_UNSUPPORTED | 3 | 未知 command id |
| MAV_RESULT_FAILED | 4 | 底层执行失败 |

ACK 路由规则：

```text
1. 树莓派下发命令时 source_component=193。
2. 固件排队 ACK 时 target_component 会等于树莓派 source_component。
3. MAVLink TX 发现 pending ACK 的 target_component 为 193 时，从 USART6/RPi 出口发送。
4. 非 RPi ACK 不会被 RPi 专属发送循环消费。
```

## 8. 树莓派发送示例

### 8.1 设置四路电机为 50%

```python
from pymavlink import mavutil

RPI_SYSID = 250
RPI_COMPID = 193

TARGET_SYSTEM = 0
TARGET_COMPONENT = 0

master = mavutil.mavlink_connection(
    "/dev/ttyAMA0",
    baud=115200,
    source_system=RPI_SYSID,
    source_component=RPI_COMPID,
)

master.mav.command_long_send(
    TARGET_SYSTEM,
    TARGET_COMPONENT,
    31011,
    0,
    50.0,
    50.0,
    50.0,
    50.0,
    0.0,
    0.0,
    0.0,
)
```

### 8.2 急停

```python
master.mav.command_long_send(
    TARGET_SYSTEM,
    TARGET_COMPONENT,
    31090,
    0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
)
```

### 8.3 等待 ACK

```python
while True:
    msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
    if msg is None:
        print("ACK timeout")
        break

    if msg.target_system not in (0, RPI_SYSID):
        continue
    if msg.target_component not in (0, RPI_COMPID):
        continue

    if msg.command in (31011, 31090):
        print(f"command={msg.command}, result={msg.result}")
        break
```

## 9. 联调检查清单

1. 树莓派发送 MAVLink `COMMAND_LONG(76)`，不要直接发 JSON 到 STM32。
2. USART6 使用 115200 8N1。
3. 树莓派 TX 接 STM32 PC7，树莓派 RX 接 STM32 PC6，并共地。
4. `source_system` 非 0，且不等于 STM32 本机 system id。
5. `source_component` 固定为 193，便于 ACK 从 USART6 返回。
6. 联调阶段先使用 `target_system=0,target_component=0`。
7. `param1..param4` 必须在 `0.0..100.0`。
8. 下发 `31011, 50,50,50,50` 后，本机电机页四路进度条应显示 50。
9. 只下发一次 50% 后目标应保持，不需要树莓派持续重发。
10. 下发本机滑条新目标后，本机滑条目标应覆盖树莓派旧目标。
11. 再次下发树莓派新目标后，树莓派目标应覆盖本机滑条旧目标。
12. 下发 `31090` 后四路目标清零并急停锁存，非 0 油门应被拒绝。

## 10. 验证记录

已完成检查：

```text
1. ARMCC 5.06 单文件编译检查通过：
   - Bsp/Src/bsp_uart.c
   - Core/Src/stm32f4xx_it.c
   - Core/Src/stm32f4xx_hal_msp.c
   - Framework/Src/px4lite_platform_f407.c
   - Framework/Src/px4lite_modules.c
   - Framework/Src/px4lite_mavlink_tx.c

2. git diff --check 通过，没有空白错误。
```

待复核：

```text
完整 Keil UV4 Rebuild All 本轮命令行未刷新构建日志。
正式烧录前应在本机 μVision 中执行 Rebuild All，并确认 0 Error(s), 0 Warning(s)。
```

## 11. 安全边界

```text
1. BSP 只负责原始 UART 收发，不解析 MAVLink。
2. Platform Adapter 只做字节流到 Framework 通信帧的转换。
3. MAVLink RX 只做协议解码。
4. Command 层负责命令校验、分发和 ACK。
5. Control 层负责目标锁存、急停、安全保护和最终 PWM 输出。
6. Display 只显示 Control 输出，不直接控制 BSP PWM。
7. 任何电机命令不得绕过 Px4Lite_ControlSetMotorThrottlePercent() 或 Px4Lite_ControlEmergencyStop()。
```
