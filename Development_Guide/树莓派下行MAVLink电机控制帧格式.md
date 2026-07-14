# 树莓派下行 MAVLink 电机控制帧格式

## 1. 适用范围

本文定义树莓派向 STM32 下发电机控制命令时应发送的 MAVLink 帧格式。当前固件只解析 MAVLink，不解析 JSON；树莓派侧应把上层 JSON、MQTT 或 UI 指令转换成 MAVLink `COMMAND_LONG` 后再发送。

当前已落地的控制语义：

```text
树莓派 COMMAND_LONG
  -> STM32 MAVLink RX 解码
  -> Framework/Src/px4lite_command.c 分发命令
  -> Framework/Src/px4lite_control.c 锁存电机目标油门
  -> Display/Src/display.c 在本机电机页显示进度条
```

注意：本文定义的是 MAVLink 协议帧内容。当前固件已经把 USART6/RPi RX 字节流接入 MAVLink 解析和 `Px4Lite_CommRxFrame_t`/Command 处理链路；树莓派可通过 USART6 直连下发本文定义的 `COMMAND_LONG` 帧。RPi 接收队列优先于 LoRa 接收队列取帧；若本周期没有 RPi 帧，仍按原逻辑处理 LoRa 远端帧。

## 2. 物理链路参数

树莓派与 STM32 的 RPi MAVLink 物理口按当前配置为：

```text
接口：USART6
STM32 引脚：PC6(TX) / PC7(RX)
波特率：115200
数据格式：8N1
电平：3.3V TTL
流控：无
```

如果命令不是走 USART6，而是走已有 LoRa/通信接收帧通道，则 MAVLink 帧内容不变，只是承载链路不同。

## 3. MAVLink 头字段约定

树莓派发送时建议使用 MAVLink 2；MAVLink 1 的 `COMMAND_LONG` 也能按同一 payload 语义解析。

| 字段 | 建议值 | 说明 |
|---|---:|---|
| `msgid` | `76` | `COMMAND_LONG`。 |
| `source_system` / `sysid` | `250` 或其他非 0 值 | 必须与 STM32 本机 `system_id` 不同；若冲突，换一个 1..250 的值。 |
| `source_component` / `compid` | `193` | 建议使用 `PX4LITE_RPI_MAVLINK_COMPONENT_ID`。 |
| `target_system` | `0` 或 STM32 本机 `system_id` | `0` 表示广播，固件会接受；精确控制时填本机 system id。 |
| `target_component` | `0` 或 `191` | `0` 表示广播，`191` 是 STM32 主 MAVLink component id。 |
| `confirmation` | `0` | 首次发送填 0；重复同一命令可递增，但当前固件不依赖该字段。 |

重要规则：

1. `source_system` 不能为 `0`。
2. `source_system` 不能等于 STM32 本机 `system_id`，否则接收层会把它当成本机帧忽略。
3. `target_system=0,target_component=0` 是最省事的广播写法。
4. 若要点名控制某块板，`target_system` 必须等于该板由 UID 派生出的 MAVLink system id，`target_component` 填 `191`。
5. 若希望 `COMMAND_ACK` 从 USART6 回到树莓派，`source_component` 应使用 `193`；固件按该 component id 将 ACK 路由到 RPi 专属出口。

## 4. 命令 31011：设置四路电机油门

### 4.1 语义

`31011` 用于一次性设置四路电机目标油门百分比。该命令是锁存式控制：例如下发 50 后，目标会一直保持 50，直到本机屏幕滑条、树莓派新命令、急停、低压保护或恢复复位覆盖它。

当前固件接收的是 0..100 的油门百分比，不是直接的微秒脉宽。控制层再按配置映射到 ESC PWM：

```text
pulse_us = 1000 + (2000 - 1000) * percent / 100

0   -> 1000 us
50  -> 1500 us
100 -> 2000 us
```

### 4.2 COMMAND_LONG 字段

| COMMAND_LONG 字段 | 值 | 说明 |
|---|---:|---|
| `command` | `31011` | 设置四路电机油门百分比。 |
| `param1` | `0.0 .. 100.0` | 电机 1 目标油门百分比。 |
| `param2` | `0.0 .. 100.0` | 电机 2 目标油门百分比。 |
| `param3` | `0.0 .. 100.0` | 电机 3 目标油门百分比。 |
| `param4` | `0.0 .. 100.0` | 电机 4 目标油门百分比。 |
| `param5` | `0.0` | 当前固件未使用，固定填 0。 |
| `param6` | `0.0` | 当前固件未使用，固定填 0。 |
| `param7` | `0.0` | 当前固件未使用，固定填 0。 |

参数非法时的处理：

```text
param1..param4 任一值 <0 或 >100  -> COMMAND_ACK: MAV_RESULT_DENIED
急停已锁存且油门非 0             -> COMMAND_ACK: MAV_RESULT_TEMPORARILY_REJECTED
正常接收并写入 Control 目标       -> COMMAND_ACK: MAV_RESULT_ACCEPTED
```

### 4.3 示例：四路都设置为 50%

逻辑字段：

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

`COMMAND_LONG` payload 字节布局为 MAVLink 标准 little-endian：

```text
offset  size  field
0       4     param1 float
4       4     param2 float
8       4     param3 float
12      4     param4 float
16      4     param5 float
20      4     param6 float
24      4     param7 float
28      2     command uint16
30      1     target_system uint8
31      1     target_component uint8
32      1     confirmation uint8
```

上面示例的 payload 十六进制为：

```text
00 00 48 42  00 00 48 42  00 00 48 42  00 00 48 42
00 00 00 00  00 00 00 00  00 00 00 00  23 79  00 00 00
```

说明：

```text
50.0f  = 00 00 48 42
31011  = 0x7923 -> little-endian 23 79
```

完整 MAVLink 帧还包含 STX、长度、序号、sysid、compid、msgid 和 CRC，建议由 MAVLink 库生成，不建议手写。

## 5. 命令 31090：电机急停

### 5.1 语义

`31090` 用于锁存急停并立即撤销四路电机输出。急停后，非 0 油门命令会被拒绝，直到固件恢复/复位逻辑解除急停锁存。

### 5.2 COMMAND_LONG 字段

| COMMAND_LONG 字段 | 值 | 说明 |
|---|---:|---|
| `command` | `31090` | 电机急停。 |
| `param1..param7` | `0.0` | 当前固件不使用，固定填 0。 |
| `target_system` | `0` 或本机 system id | 同 31011。 |
| `target_component` | `0` 或 `191` | 同 31011。 |
| `confirmation` | `0` | 首次发送填 0。 |

### 5.3 示例：急停

逻辑字段：

```text
msgid            = COMMAND_LONG(76)
source_system    = 250
source_component = 193
target_system    = 0
target_component = 0
command          = 31090
confirmation     = 0
param1..param7   = 0.0
```

payload 十六进制为：

```text
00 00 00 00  00 00 00 00  00 00 00 00  00 00 00 00
00 00 00 00  00 00 00 00  00 00 00 00  72 79  00 00 00
```

说明：

```text
31090 = 0x7972 -> little-endian 72 79
```

## 6. ACK 回包

STM32 接收到属于本机的 `COMMAND_LONG` 后，会排队发送 `COMMAND_ACK`。

ACK 逻辑字段：

| COMMAND_ACK 字段 | 说明 |
|---|---|
| `msgid` | `77`，`COMMAND_ACK`。 |
| `sysid` | STM32 本机 system id。 |
| `compid` | `191`。 |
| `command` | 被确认的命令号，例如 `31011` 或 `31090`。 |
| `result` | MAVLink 标准 `MAV_RESULT_*`。 |
| `target_system` | 树莓派发送命令时使用的 `source_system`。 |
| `target_component` | 树莓派发送命令时使用的 `source_component`。 |

结果码：

| result | 数值 | 含义 |
|---|---:|---|
| `MAV_RESULT_ACCEPTED` | `0` | 命令已接受。 |
| `MAV_RESULT_TEMPORARILY_REJECTED` | `1` | 当前急停锁存或 Control 忙。 |
| `MAV_RESULT_DENIED` | `2` | 参数非法，例如油门百分比超出 0..100。 |
| `MAV_RESULT_UNSUPPORTED` | `3` | 未知 command id。 |
| `MAV_RESULT_FAILED` | `4` | 底层执行失败。 |

树莓派端应以 `command + result + target_system + target_component` 判断本次命令是否成功。只要 `result == MAV_RESULT_ACCEPTED`，即可认为 STM32 已接受命令；电机页进度条随后会显示本机 Control 输出目标。

## 7. pymavlink 发送示例

### 7.1 设置四路电机为 50%

```python
from pymavlink import mavutil

RPI_SYSID = 250
RPI_COMPID = 193

TARGET_SYSTEM = 0       # 广播；若要点名，填 STM32 本机 system id
TARGET_COMPONENT = 0    # 广播；若要点名，填 191

master = mavutil.mavlink_connection(
    "/dev/ttyAMA0",
    baud=115200,
    source_system=RPI_SYSID,
    source_component=RPI_COMPID,
)

master.mav.command_long_send(
    TARGET_SYSTEM,
    TARGET_COMPONENT,
    31011,      # command: set motor throttle percent
    0,          # confirmation
    50.0,       # motor1 percent
    50.0,       # motor2 percent
    50.0,       # motor3 percent
    50.0,       # motor4 percent
    0.0,
    0.0,
    0.0,
)
```

### 7.2 急停

```python
master.mav.command_long_send(
    TARGET_SYSTEM,
    TARGET_COMPONENT,
    31090,      # command: emergency stop
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

### 7.3 等待 ACK

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

## 8. 联调检查清单

1. 树莓派发送 `COMMAND_LONG(76)`，不要发送自定义 JSON 到 STM32。
2. `source_system` 必须非 0，且不能等于 STM32 本机 system id。
3. `source_component` 建议固定 `193`。
4. `target_system,target_component` 可先用 `0,0` 广播联调。
5. `param1..param4` 必须在 `0.0..100.0`，当前不是微秒脉宽。
6. 下发 `31011, 50,50,50,50` 后，本机电机页四路进度条应显示 50。
7. 下发一次后目标会保持，不需要树莓派周期重发。
8. 下发 `31090` 后，四路目标清零并急停锁存；后续非 0 油门会收到临时拒绝。
9. 如果树莓派直连 USART6 发帧但 STM32 无动作，先检查 PC6/PC7 交叉接线、3.3V TTL 电平、共地、115200 8N1、`source_system` 不等于本机 system id，以及 `source_component=193`。
