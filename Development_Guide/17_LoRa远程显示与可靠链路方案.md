# LoRa 远程显示与可靠链路方案

本文用于记录 LoRa 远程显示模式的现状、目标和后续实现清单。内容对照当前代码编写，只把已经存在的能力写为已完成；还没有代码入口的内容均列为待实现。

## 1. 目标口径

远程模式下，本机主要作为显示端使用。屏幕主数据来自 LoRa 接收到的远端 MAVLink 遥测解析结果，不覆盖本机传感器 topic，也不直接参与远端控制。

断链时不自动切回本地模式，屏幕继续停留在远程模式，并显示远端设备超时或数据过期。本机仍负责按钮、屏幕刷新、本地链路统计、超时判断和必要的显示端运行状态。

远程模式下，电机滑块只作为远端电机占空比的只读显示控件使用。滑块触摸输入必须禁用或被忽略，不得调用本机电机控制 API，也不得经 LoRa 生成远端电机控制命令。只有 `LOCAL` 模式允许滑块提交本机控制。

模式切换由 KEY0 触发，屏幕左上角显示当前模式。默认 `LOCAL`：显示本机数据，LoRa 只主动发送本机 MAVLink 遥测，不处理远端 MAVLink 数据。切到 `REMOTE` 后：停止主动发送本机周期遥测，开始接收和解析远端 MAVLink 遥测。`REMOTE` 下后续仍允许发送必要的链路控制帧或 MAVLink 控制确认，因此“停止发送”只表示停止主动遥测发送，不表示 UART 物理发送完全禁用。

当前阶段不强制引入自定义 LoRa 帧格式。第一版先复用现有 MAVLink 发送链路，新增 MAVLink RX、远端快照和显示源切换，完成两台设备的 `LOCAL` 发送、`REMOTE` 接收显示验证。后续如果需要严格 ACK、绑定确认、分时双工或远程控制，再在 MAVLink 扩展消息或独立 LoRa Link 层中增强。

## 2. 当前代码已经完成的部分

| 能力 | 当前代码位置 | 状态 | 说明 |
|---|---|---|---|
| LoRa 硬件配置 | `Bsp/Inc/bsp_config.h` | 已完成 | E22 使用 USART3，PB10/PB11，UART 为 9600 8N1，当前外部配置的 E22 空中速率按 `BSP_LORA_AIR_BPS=2400` 记录；RX DMA 为 `DMA1_Stream1`，TX DMA 为 `DMA1_Stream3`，M0/M1/AUX 接到 PF1/PF2/PF0。 |
| LoRa BSP 收发 | `Bsp/Src/bsp_lora.c` | 已完成 | 已实现 UART DMA RX 环形缓冲、IDLE 推进、TX DMA 单帧发送、AUX 判断、TX abort、USART 错误恢复，并向驱动暴露 UART 波特率和空中速率配置值。 |
| LoRa 驱动发送状态机 | `Sensor/Src/lora_e22.c` | 已完成 | 已实现 `IDLE -> WAIT_AUX -> SENDING` 非阻塞发送状态机，单帧在飞，发送完成以 UART TX DMA 完成回调为准；AUX 等待超时按最大帧空中耗时计算，UART DMA 卡死超时按当前帧串口耗时计算。 |
| MAVLink 接收帧识别 | `Sensor/Src/lora_e22.c` | 基础完成 | 已用 `mavlink_parse_char()` 识别完整 MAVLink 帧，并保存最近一帧到 `s_rx_frame`。 |
| LoRa 调试统计 | `Sensor/Inc/lora_e22.h`、`Framework/Src/px4lite_modules.c` | 已完成 | 已统计 RX/TX 帧数、发送忙、CRC/解析错误、溢出、最近收发时间和最近消息 ID。 |
| Comm 任务调度 | `Framework/Src/px4lite_modules.c` | 已完成 | `Px4Lite_CommWorkRun()` 周期调用 `Px4Lite_LoRaService()` 和 `Px4Lite_MavlinkTxRun()`，并更新 LoRa 模块状态。 |
| MAVLink 遥测发送 | `Framework/Src/px4lite_mavlink_tx.c` | 已完成 | 已按槽位发送 `HEARTBEAT`、`GPS_RAW_INT`、`GNSS_SAT`、`ATTITUDE`、`GLOBAL_POSITION_INT`、`SYS_STATUS`、`BATTERY_STATUS`、`SCALED_PRESSURE`、`STATUSTEXT`，并轮转 `TIME_LOC`、`DATE_LOC`、`HUMIDITY`、`MOTOR12`、`MOTOR34`、`MODSTAT` 等远程显示扩展；电机 PWM 变化时会优先发送并短时重复。 |
| MAVLink 配置开关 | `Framework/Inc/px4lite_config.h` | 已完成 | 已提供各类 MAVLink 消息 enable 和 period 配置。 |
| 本地显示数据链路 | `Display/Src/display.c`、`Business/Inc/app_data_api.h` | 已完成 | 当前显示通过 `App_CopyNavigation()`、`App_CopyDateTime()`、`App_CopySystem()`、`App_CopyAlarm()`、`App_CopyEnvironment()`、`App_CopyMotor()` 读取本机应用快照。 |
| MAVLink RX 分发 | `Framework/Src/px4lite_mavlink_rx.c` | 已完成 | 已在 REMOTE 模式下解析远端标准遥测和 `NAMED_VALUE_INT` 扩展，非目标 `sysid` 不写远端快照。 |
| RemoteTelemetry 远端快照 | `Framework/Src/px4lite_remote_telemetry.c` | 已完成 | 已保存目标 `sysid`、远端设备表、字段有效位、字段级更新时间、字段 stale 位和远端只读显示字段。 |
| 远程显示数据源 | `Display/Src/display.c` | 已完成 | REMOTE 模式主字段来自远端快照；字段未收到时无效，已收到但短暂过期时保留最后值并由 `stale_mask` 标记。 |
| 远程电机只读保护 | `Display/Src/display.c`、`Business/Src/app_data_api.c` | 已完成 | REMOTE 模式禁用电机滑块触摸和急停触摸写入，Business 控制 API 也拒绝写本机 Control。 |
| 电机状态灯口径 | `Display/Src/display.c`、`Framework/Src/px4lite_mavlink_tx.c` | 已完成 | 当前硬件没有 ESC/电机真实存在检测，四个电机状态灯固定绿灯；该灯不表示真实电机在线，只表示此项不参与故障判定。 |

## 3. 当前代码尚未完成的部分

| 缺口 | 当前现象 | 后续要求 |
|---|---|---|
| 远端设备列表 UI | Framework 已有远端设备观测表，但 Display 尚未绘制设备列表或选择控件。 | 后续 UI 阶段展示 `sysid`、状态、最近心跳、最近有效遥测和统计；第二阶段仍支持固定目标配置。 |
| 设备强身份 | 当前仍依赖手动配置 MAVLink `sysid`，相同 `sysid` 的设备不能仅靠心跳区分。 | STM32 Unique ID 派生 `sysid` 或协议层身份先保留不做，后续评估兼容性后再接入。 |
| 完整模块状态 | 当前 `SYS_STATUS` 只覆盖 GNSS/IMU/Baro 主要健康位。 | 远程显示需要 GNSS、IMU、Baro、Battery、LoRa、Storage、Display、Control 等模块状态和故障码。 |
| 完整告警表 | 当前 `STATUSTEXT` 只发送最高活动告警。 | 告警页和消息日志需要活动告警表、最高告警、更新时间和来源。 |
| 模式与链路事件日志 | 当前远程模式切换、目标变化和远端字段过期未写入日志模块。 | 后续写入消息日志或独立链路事件记录，便于现场回放。 |
| ACK / 重发 | 当前 LoRa 遥测发送不等待对端 ACK。 | 第一版远程显示先不强制 ACK；后续如需可靠传输，再基于 MAVLink 扩展或 LoRa Link 实现 ACK、重发和去重。 |
| LoRa 控制 | `PX4LITE_ENABLE_COMMAND` 当前为 0，无命令执行器。 | 当前阶段不做控制；后续控制必须经过权限、ACK、超时和本地急停门禁。 |

## 4. 推荐的后续分层

第一阶段不新增自定义裸帧协议，优先沿用现有 MAVLink 数据面。后续不应把远程显示、设备选择和控制逻辑堆到现有 `lora_e22.c` 或 Display 中。建议第一阶段新增分层如下：

```text
BSP LoRa
  -> E22 Driver
    -> MAVLink RX
      -> Remote Telemetry
        -> Business Remote API
          -> Display Source Selector
```

各层职责：

| 层级 | 职责 | 禁止事项 |
|---|---|---|
| BSP LoRa | UART、DMA、IDLE、GPIO、AUX、错误恢复 | 不解析协议，不判断设备身份 |
| E22 Driver | 非阻塞收发、MAVLink 字节流解析基础、统计 | 不解释业务命令，不写 Display/Business 状态 |
| MAVLink RX | 按 `msgid` 解码远端 MAVLink 消息，更新远端快照和 RX 统计 | 不做 KEY0 模式切换，不直接渲染 Display，不执行控制 |
| Remote Telemetry | 保存远端只读快照、字段有效位、目标 `sysid` 和超时状态 | 不覆盖本机 Navigation/System/Environment topic |
| Business Remote API | 向 Display 暴露远端只读快照 | 不绕过设备过滤和超时判断 |
| Display Source Selector | 根据本地/远程模式选择显示数据源 | 不在显示层解析 LoRa 字节流或 MAVLink payload |

后续如果必须做可靠 ACK、绑定确认、分时双工或控制权限，可在 `MAVLink RX/TX` 旁新增独立 `LoRa Link` 或 `LoRa Scheduler`，也可以优先用 MAVLink `COMMAND_LONG/COMMAND_ACK`、`TUNNEL`、`V2_EXTENSION` 或自定义 dialect 承载扩展，不把自定义裸帧作为第一版前提。

## 5. 远程设备选择流程

第一版远程模式可以先使用固定目标 `sysid` 或简单设备列表。远程模式不自动连接最近设备，必须有明确的目标设备口径。

MAVLink `HEARTBEAT` 在未绑定前只能用于发现和临时区分通信层设备：当多台设备的 `sysid` 不同时，本机可以把它们记录为不同远端设备；但 `HEARTBEAT` 不提供强身份认证，也不能证明双方已经建立可信连接。如果多台设备使用相同 `sysid`，仅靠 `HEARTBEAT` 无法区分。第二阶段文档和 UI 文案不得把收到心跳称为“连接成功”，应使用“发现设备”“目标设备”“目标有效遥测”等口径。

```text
LOCAL 模式
  -> KEY0 切换到 REMOTE
    -> 左上角显示 REMOTE
      -> 停止主动发送本机周期遥测
        -> 开始处理远端 MAVLink HEARTBEAT
          -> 记录在线设备 sysid
            -> 选择或匹配目标 sysid
              -> 只接收目标 sysid 的远端遥测
                -> 更新 RemoteTelemetry
                  -> 进入远程显示模式
```

再次按 KEY0 切回 `LOCAL` 后，应停止远端 MAVLink 数据处理，恢复本机数据显示和本机周期遥测主动发送。

设备心跳第一版使用 MAVLink `HEARTBEAT`。设备表建议记录：

| 字段 | 说明 |
|---|---|
| `sysid` | MAVLink system id，第一版作为远端设备 ID 和过滤依据。 |
| `compid` | MAVLink component id，用于区分同一设备内组件。 |
| `last_heartbeat_ms` | 最近收到心跳的本机毫秒时间，用于在线/超时判断。 |
| `last_msg_ms` | 最近收到任意有效遥测的本机毫秒时间。 |
| `mav_type` | MAVLink 设备类型，来自 `HEARTBEAT`。 |
| `base_mode` / `system_status` | MAVLink 心跳状态字段，用于显示远端基础状态。 |
| `rx_count` / `filtered_count` / `unsupported_count` | 目标有效遥测、过滤消息和未支持消息统计。 |
| `remote_id` | 后续 Remote ID 接入后填充，当前可为空。 |

第二阶段远端设备状态口径：

| 状态 | 含义 |
|---|---|
| `DISCOVERED` | 收到某个 `sysid` 的 `HEARTBEAT`，仅表示发现通信层设备。 |
| `TARGETED` | 本机配置或选择该 `sysid` 作为远程显示目标。 |
| `ACTIVE` | 已收到目标设备的有效遥测，`RemoteTelemetry` 正在由目标数据更新。 |
| `STALE` | 目标设备曾经有效，但最近心跳或有效遥测已超时。 |
| `BOUND` | 后续完成绑定握手、ACK 或可信身份确认后才能使用；第二阶段不实现。 |

绑定成功的严格握手不作为第一版和第二阶段必要条件。第二阶段只要求选择或配置目标 `sysid` 后，未匹配设备的数据不会进入远程显示快照；收到目标有效遥测后进入 `ACTIVE` 显示。后续如果需要更强的绑定确认，再增加 `COMMAND_LONG/COMMAND_ACK`、`TUNNEL` 或 LoRa Link 绑定流程。

当前版本不设置 `group_id`。如果后续出现多组同时教学或多套设备共场运行的需求，优先考虑通过 LoRa 信道、空中速率、E22 参数或协议层 `network_id` 隔离，不在第一版远程显示协议中提前引入分组字段。

Remote ID 后续接入时复用同一套设备身份源，但不直接替代第一版 `sysid` 过滤。Remote ID 更适合做法规或产品要求的公开身份字段，MAVLink `sysid` 继续承担通信过滤和远端显示选择。

## 6. MAVLink 优先与后续可靠性增强

当前阶段优先复用 MAVLink，不设计新的自定义裸帧格式。第一版需要新增的是 MAVLink RX 消息分发和远端快照，不是 LoRa Link 帧头。

第一版建议解析的 MAVLink 消息：

| MAVLink 消息 | 用途 |
|---|---|
| `HEARTBEAT` | 设备在线、`sysid` 识别、基础系统状态。 |
| `GPS_RAW_INT` | GNSS fix、经纬度、高度、HDOP、卫星数。 |
| `GLOBAL_POSITION_INT` | 全局位置、高度、航向。 |
| `ATTITUDE` | roll、pitch、yaw 和角速度。 |
| `SYS_STATUS` | 基础系统状态、电压、电流、电量。 |
| `BATTERY_STATUS` | 电池状态、电压、电流和剩余电量。 |
| `SCALED_PRESSURE` | 气压和温度。 |
| `STATUSTEXT` | 最高告警或文本状态。 |
| `NAMED_VALUE_INT` | 当前已用于 `GNSS_SAT`、`TIME_LOC`、`DATE_LOC`、`HUMIDITY`、`MOTOR12`、`MOTOR34` 等轻量扩展。 |

后续可靠性增强可分三种路径：

| 路径 | 适用场景 | 说明 |
|---|---|---|
| MAVLink 标准 ACK | 绑定、模式切换、控制命令 | 使用 `COMMAND_LONG` / `COMMAND_ACK`，保持协议生态兼容。 |
| MAVLink 扩展载荷 | 远端完整快照、电机状态、告警表 | 使用 `TUNNEL`、`V2_EXTENSION` 或自定义 MAVLink dialect。 |
| LoRa Link / Scheduler | 严格 stop-and-wait、分时双工、多机调度 | 仅在 MAVLink 扩展不足或需要统一链路层可靠性时引入。 |

后续分时双工升级时，可以在独立 LoRa Scheduler 中扩展：

```text
WAIT_SLOT
  -> OWN_SLOT
    -> SEND_FRAME
      -> YIELD_CHANNEL
```

Display、Remote Telemetry 和 Business API 不应依赖具体调度算法，避免后续升级分时双工时改动显示业务。

## 7. 远程显示需要携带的数据

远程模式主显示字段应全部来自远端快照。第一版优先从现有 MAVLink 消息还原，缺失字段先标记无效，不用本机数据冒充。

| 载荷 | 第一版来源 | 字段 |
|---|---|---|
| 身份 | `HEARTBEAT` | `sysid`、`compid`、设备类型、系统状态、`remote_id` 预留。 |
| 时间 | `NAMED_VALUE_INT` 扩展 | `TIME_LOC`、`DATE_LOC`，用于 REMOTE 模式显示远端本地日期时间。 |
| 导航 | `GPS_RAW_INT` / `GLOBAL_POSITION_INT` | fix、经纬度、高度、速度、航向、HDOP、卫星数、导航质量。 |
| 姿态 | `ATTITUDE` | roll、pitch、yaw、roll rate、pitch rate、yaw rate。 |
| 环境与电源 | `SCALED_PRESSURE` / `SYS_STATUS` / `BATTERY_STATUS` / `NAMED_VALUE_INT` 扩展 | 气压、温度、湿度、电压、电流、电量百分比、低电压标志。 |
| 系统状态 | `HEARTBEAT` / `SYS_STATUS` | 系统 ready、基础模块状态、故障码预留、状态版本。 |
| 告警 | `STATUSTEXT` | 最高告警或文本状态；完整活动告警表后续扩展。 |
| 电机状态 | `NAMED_VALUE_INT` 扩展 | `MOTOR12`、`MOTOR34` 携带四路占空比、run_state、speed_level。REMOTE 模式下只用于滑块只读显示，不作为控制命令。 |
| 链路统计 | 本机 RX 统计 | 接收帧数、解析错误、过滤丢弃、重复消息、最后接收时间、目标 `sysid`。 |

本机数据使用原则：

- 远程模式主字段不使用本机 Navigation、Environment、System、Alarm、Motor 快照。
- 本机 RTC 可用于计算远端数据年龄和屏幕自身时间，但不能冒充远端设备时间。
- 远程模式下本机电机滑块触摸必须禁用；滑块位置只由远端电机状态载荷刷新，远端电机字段未收到时显示无效，已收到但过期时保留最后值并通过 stale 状态提示。
- 如果显示端也需要显示自身电量，应新增“本机/显示端电量”字段，不复用远端电量字段。

## 8. 实现清单

### 8.1 第一阶段：MAVLink RX 远程显示

1. 新增显示源模式状态，默认 `LOCAL`。
2. 接入 KEY0 边沿检测，用于 `LOCAL` / `REMOTE` 模式切换，并触发显示头部刷新。
3. 在显示屏左上角绘制 `LOCAL` / `REMOTE` 模式标识。
4. 新增 `px4lite_mavlink_rx.c/h`，消费 `Lora_E22_CopyRxFrame()` 输出的 `mavlink_message_t`。
5. 在 `REMOTE` 模式下解析 `HEARTBEAT`、`GPS_RAW_INT`、`GLOBAL_POSITION_INT`、`ATTITUDE`、`SYS_STATUS`、`BATTERY_STATUS`、`SCALED_PRESSURE`、`STATUSTEXT`、`NAMED_VALUE_INT`。
6. `LOCAL` 模式只主动发送本机 MAVLink 遥测，不处理远端 MAVLink 数据。
7. `REMOTE` 模式停止主动发送本机周期遥测，开始处理远端 MAVLink 数据。
8. 新增 `RemoteTelemetry` 快照和字段有效位，记录目标 `sysid`、最近心跳、最近遥测和超时状态。
9. 新增 Business 只读 API，向 Display 暴露远端快照，不改变现有本机 App API 语义。
10. 新增 Display 本地/远程数据源选择，远程模式读取远端快照。
11. 远程模式断链后显示远端超时，不自动切回本地。
12. 日志记录模式切换、目标设备变化、远端超时、解析错误、设备过滤丢弃和远端数据过期。


### 8.1.1 第一阶段实现状态

已完成：

- `LOCAL` / `REMOTE` 模式状态，默认 `LOCAL`。
- KEY0 边沿切换模式。
- 显示屏头部左上角 `LOCAL` / `REMOTE` 标识。
- `px4lite_mavlink_rx.c/h` 接收分发模块。
- `RemoteTelemetry` 远端快照、字段有效位、目标 `sysid` 和超时状态。
- `LOCAL` 模式只主动发送本机 MAVLink 遥测，不处理远端 MAVLink 数据。
- `REMOTE` 模式停止本机周期遥测，处理远端 MAVLink 数据并写入远端快照。
- Business 远端只读 API 和 Display 本地/远程数据源选择。
- 远端断链或过期时显示远端无效，不自动切回本机数据。
- 桌面侧单元测试和 Keil Rebuild 验证已通过。

未完成，保留到后续阶段：

- 远程设备列表 UI 和人工选择目标设备。
- 完整模块状态和完整告警表补齐。
- ACK、重发、绑定握手、分时双工和远程控制权限。
- 模式切换、远端超时、非法设备等事件写入日志模块。

### 8.2 第二阶段：设备选择与字段补齐

1. 根据 `HEARTBEAT` 建远程设备表，显示 `sysid`、在线/超时、基础状态和链路质量。
2. 明确设备发现不等于绑定或连接成功：`HEARTBEAT` 只进入 `DISCOVERED`，收到目标有效遥测后才进入 `ACTIVE` 显示；`BOUND` 留到后续可靠链路阶段。
3. 增加目标设备选择或固定目标配置，未匹配 `sysid` 的数据不进入远端快照。
4. 补齐远端时间载荷，当前使用 `NAMED_VALUE_INT` 的 `TIME_LOC` / `DATE_LOC`。
5. 补齐湿度和电机状态，当前使用 `HUMIDITY`、`MOTOR12`、`MOTOR34`；完整模块状态和完整告警表保留后续阶段。
6. 将 REMOTE 模式电机滑块定义为只读显示控件：禁用触摸写入，滑块位置仅来自远端电机占空比；远端字段未收到时无效，已收到但过期时保留最后值并标记 stale。
7. 在 Display 触摸层和 Business 控制入口形成双层保护，确保 REMOTE 模式不会调用 `App_SetMotorThrottlePercent()`、不会写本机 PWM、不会发送远端控制帧。
8. 配置低速链路发送周期，按 UART 波特率和 E22 空中速率分别计算发送预算，避免空中速率低于串口速率时持续发送导致接收堆积或显示数据过期。
9. 增加远程数据字段级有效位和 stale 位，避免部分字段短暂过期时归零，同时让显示层可区分“未收到”和“旧值”。

### 8.2.1 第二阶段实现状态

已完成：

- `Px4Lite_RemoteTelemetryCopyDevices()` 远端设备观测表，记录 `DISCOVERED`、`TARGETED`、`ACTIVE`、`STALE` 状态；`BOUND` 仅预留。
- 手动 `sysid` 目标配置入口：`PX4LITE_REMOTE_TARGET_SYSID_DEFAULT` 和 `Px4Lite_RemoteTelemetrySetTargetSysId()`。
- 目标过滤：非目标 `sysid` 的遥测不写入远端显示快照，只更新过滤统计或设备表。
- 远端时间、湿度、电机占空比通过 `NAMED_VALUE_INT` 轻量扩展轮转发送和接收，电机占空比变化时优先发送并短时重复。
- 远端快照字段级更新时间、字段级 stale 判断和过期旧值保留。
- REMOTE 模式电机滑块只读显示；Display 触摸层和 Business 控制 API 双层拒绝写本机电机控制。
- 桌面侧 `test_mavlink_rx_remote_telemetry` 和 Keil Rebuild 验证已通过。

未完成，保留到后续阶段：

- 远端设备表 UI 和屏幕上的人工选择控件。
- STM32 Unique ID 派生 `sysid` 或更强身份方案。
- 完整模块状态、完整告警表和链路事件日志。

### 8.3 第三阶段：可靠性与控制预留

1. 评估是否需要 ACK 和重发；如果显示只要求最新值，优先保持无 ACK 遥测流。
2. 绑定、模式切换或控制命令优先使用 MAVLink `COMMAND_LONG` / `COMMAND_ACK`。
3. 远端完整快照、电机状态和告警表可使用 `TUNNEL`、`V2_EXTENSION` 或自定义 MAVLink dialect。
4. 只有在 MAVLink 扩展不足或需要统一链路层可靠性时，才新增 LoRa Link 帧头、stop-and-wait、去重和重发。
5. 保留时分双工调度字段或调度模块入口，例如 `slot_id`、`window_ms`、`link_epoch`，但不在第一阶段实现。
6. 不在远程显示阶段执行电机控制命令。
7. 后续启用控制前，必须补充本地急停、命令白名单、权限超时和执行 ACK。

## 9. 验收要点

| 验收项 | 通过条件 |
|---|---|
| LOCAL 模式 | 显示本机数据，左上角显示 `LOCAL`，LoRa 只主动发送本机 MAVLink 遥测，不处理远端 MAVLink 数据。 |
| REMOTE 模式 | KEY0 切换后左上角显示 `REMOTE`，停止主动发送本机周期遥测，开始处理远端 MAVLink 数据。 |
| MAVLink RX | 能解析远端 `HEARTBEAT`、定位、姿态、电池、气压温度和状态文本等消息。 |
| 设备发现 | `HEARTBEAT` 只将设备登记为 `DISCOVERED`，不得显示为绑定或连接成功；相同 `sysid` 设备无法仅靠心跳区分。 |
| 设备过滤 | 只将目标 `sysid` 的数据写入远程显示快照，其他设备只更新统计或设备表；收到目标有效遥测后才进入 `ACTIVE` 显示。 |
| 远程显示 | 屏幕主字段来自 `RemoteTelemetry`，不读取本机传感器快照冒充远端数据。 |
| 远程断链 | 停留远程模式并显示远端超时，不自动切回本地。 |
| 数据有效位 | 未收到字段显示无效；已收到但过期字段保留最后远端值并标记 stale，不静默使用本机字段替代。 |
| 带宽 | UART 9600 bps、E22 空中速率 2400 bps 下接收帧数、解析错误、过滤丢弃和数据过期统计可观察，发送周期和链路超时可配置。 |
| 控制安全 | 当前阶段没有远程电机控制执行路径；REMOTE 模式下电机滑块为只读显示，触摸不会调用本机电机控制 API，也不会发送远程控制帧。 |
