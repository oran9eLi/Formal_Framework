# LoRa 远程显示与可靠链路方案

本文用于记录 LoRa 远程显示模式的现状、目标和后续实现清单。内容对照当前代码编写，只把已经存在的能力写为已完成；还没有代码入口的内容均列为待实现。

## 1. 目标口径

远程模式下，本机主要作为显示端使用。屏幕主数据来自 LoRa 接收到的远端 MAVLink 遥测解析结果，不覆盖本机传感器 topic，也不直接参与远端控制。

断链时不自动切回本地模式，屏幕继续停留在远程模式，并显示远端设备超时或数据过期。本机仍负责按钮、屏幕刷新、本地链路统计、超时判断和必要的显示端运行状态。

模式切换由 KEY0 触发，屏幕左上角显示当前模式。默认 `LOCAL`：显示本机数据，LoRa 只主动发送本机 MAVLink 遥测，不处理远端 MAVLink 数据。切到 `REMOTE` 后：停止主动发送本机周期遥测，开始接收和解析远端 MAVLink 遥测。`REMOTE` 下后续仍允许发送必要的链路控制帧或 MAVLink 控制确认，因此“停止发送”只表示停止主动遥测发送，不表示 UART 物理发送完全禁用。

当前阶段不强制引入自定义 LoRa 帧格式。第一版先复用现有 MAVLink 发送链路，新增 MAVLink RX、远端快照和显示源切换，完成两台设备的 `LOCAL` 发送、`REMOTE` 接收显示验证。后续如果需要严格 ACK、绑定确认、分时双工或远程控制，再在 MAVLink 扩展消息或独立 LoRa Link 层中增强。

## 2. 当前代码已经完成的部分

| 能力 | 当前代码位置 | 状态 | 说明 |
|---|---|---|---|
| LoRa 硬件配置 | `Bsp/Inc/bsp_config.h` | 已完成 | E22 使用 USART3，PB10/PB11，9600 8N1，RX DMA 为 `DMA1_Stream1`，TX DMA 为 `DMA1_Stream3`，M0/M1/AUX 接到 PF1/PF2/PF0。 |
| LoRa BSP 收发 | `Bsp/Src/bsp_lora.c` | 已完成 | 已实现 UART DMA RX 环形缓冲、IDLE 推进、TX DMA 单帧发送、AUX 判断、TX abort 和 USART 错误恢复。 |
| LoRa 驱动发送状态机 | `Sensor/Src/lora_e22.c` | 已完成 | 已实现 `IDLE -> WAIT_AUX -> SENDING` 非阻塞发送状态机，单帧在飞，发送完成以 UART TX DMA 完成回调为准。 |
| MAVLink 接收帧识别 | `Sensor/Src/lora_e22.c` | 基础完成 | 已用 `mavlink_parse_char()` 识别完整 MAVLink 帧，并保存最近一帧到 `s_rx_frame`。 |
| LoRa 调试统计 | `Sensor/Inc/lora_e22.h`、`Framework/Src/px4lite_modules.c` | 已完成 | 已统计 RX/TX 帧数、发送忙、CRC/解析错误、溢出、最近收发时间和最近消息 ID。 |
| Comm 任务调度 | `Framework/Src/px4lite_modules.c` | 已完成 | `Px4Lite_CommWorkRun()` 周期调用 `Px4Lite_LoRaService()` 和 `Px4Lite_MavlinkTxRun()`，并更新 LoRa 模块状态。 |
| MAVLink 遥测发送 | `Framework/Src/px4lite_mavlink_tx.c` | 已完成 | 已按槽位发送 `HEARTBEAT`、`GPS_RAW_INT`、`GNSS_SAT`、`ATTITUDE`、`GLOBAL_POSITION_INT`、`SYS_STATUS`、`BATTERY_STATUS`、`SCALED_PRESSURE`、`STATUSTEXT`。 |
| MAVLink 配置开关 | `Framework/Inc/px4lite_config.h` | 已完成 | 已提供各类 MAVLink 消息 enable 和 period 配置。 |
| 本地显示数据链路 | `Display/Src/display.c`、`Business/Inc/app_data_api.h` | 已完成 | 当前显示通过 `App_CopyNavigation()`、`App_CopyDateTime()`、`App_CopySystem()`、`App_CopyAlarm()`、`App_CopyEnvironment()`、`App_CopyMotor()` 读取本机应用快照。 |

## 3. 当前代码尚未完成的部分

| 缺口 | 当前现象 | 后续要求 |
|---|---|---|
| 接收帧消费 | `Lora_E22_CopyRxFrame()` 只有定义和声明，框架层没有调用方。 | Comm 任务需要消费接收帧，并交给 `px4lite_mavlink_rx` 解码。 |
| MAVLink RX 分发 | 当前只有 MAVLink TX，没有 Framework 层 RX 解码模块。 | 新增 `px4lite_mavlink_rx.c/h`，按 `msgid` 解码远端遥测并写入 `RemoteTelemetry`。 |
| 接收缓存 | 当前只有一个 `s_rx_frame` 和 `s_rx_ready`。 | 第一版可在 Comm 任务内及时消费完整帧；如果两台设备实测出现覆盖，再改为有界接收队列。 |
| 模式切换 | 当前没有本地/远程显示模式。 | 新增 `LOCAL` / `REMOTE` 状态，KEY0 边沿切换，显示头部左上角显示当前模式。 |
| 设备识别 | 当前 MAVLink system id 为固定配置，不能区分很多设备。 | 第一版先使用 MAVLink `sysid` 作为远端设备 ID；后续再接入 `link_device_id` 或 Remote ID。 |
| 设备选择 | 当前没有远程设备列表和目标过滤。 | 远程模式下根据 `HEARTBEAT` 建在线设备表，选择或固定目标 `sysid`，只显示目标设备数据。 |
| 远程快照 | 当前没有 `RemoteTelemetry` topic 或 Business API。 | 需要新增远端快照，保存远端导航、姿态、环境、电源、系统、告警、电机状态和时间。 |
| 显示数据源切换 | 当前 Display 只读取本机 App 快照。 | 需要增加本地/远程显示模式；远程模式主字段读取远端快照，本地只用于链路、按钮和显示端状态。 |
| 远端时间 | 当前 MAVLink GPS 时间使用启动后时间，显示时间来自本机 RTC/GNSS 时间服务。 | 远程模式应优先显示远端时间；远端未提供或过期时显示无效，不能静默替换为本机时间。 |
| 远端湿度 | 当前 `SCALED_PRESSURE` 只携带气压和温度。 | 远程显示若需要湿度，应后续通过 MAVLink 扩展消息、`NAMED_VALUE_*`、`TUNNEL` 或自定义 dialect 携带。 |
| 远端电机状态 | 当前 MAVLink 不发送 `App_MotorSnapshot_t`。 | Motor 页远程模式需要四路 PWM/百分比、run_state、speed_level；第一版可先预留，后续通过扩展载荷补齐，只显示不接控制。 |
| 完整模块状态 | 当前 `SYS_STATUS` 只覆盖 GNSS/IMU/Baro 主要健康位。 | 远程显示需要 GNSS、IMU、Baro、Battery、LoRa、Storage、Display、Control 等模块状态和故障码。 |
| 完整告警表 | 当前 `STATUSTEXT` 只发送最高活动告警。 | 告警页和消息日志需要活动告警表、最高告警、更新时间和来源。 |
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
| `rx_count` / `parse_error_count` | 接收统计和解析错误统计。 |
| `remote_id` | 后续 Remote ID 接入后填充，当前可为空。 |

绑定成功的严格握手不作为第一版必要条件。第一版只要求选择或配置目标 `sysid` 后，未匹配设备的数据不会进入远程显示快照。后续如果需要更强的绑定确认，再增加 `COMMAND_LONG/COMMAND_ACK`、`TUNNEL` 或 LoRa Link 绑定流程。

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
| `NAMED_VALUE_INT` | 当前已用于 GNSS 卫星数等轻量扩展。 |

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
| 时间 | 后续扩展 | UTC 日期、UTC 时间、本地日期时间、远端 uptime、时间来源、校时状态。 |
| 导航 | `GPS_RAW_INT` / `GLOBAL_POSITION_INT` | fix、经纬度、高度、速度、航向、HDOP、卫星数、导航质量。 |
| 姿态 | `ATTITUDE` | roll、pitch、yaw、roll rate、pitch rate、yaw rate。 |
| 环境与电源 | `SCALED_PRESSURE` / `SYS_STATUS` / `BATTERY_STATUS` | 气压、温度、电压、电流、电量百分比、低电压标志；湿度后续扩展。 |
| 系统状态 | `HEARTBEAT` / `SYS_STATUS` | 系统 ready、基础模块状态、故障码预留、状态版本。 |
| 告警 | `STATUSTEXT` | 最高告警或文本状态；完整活动告警表后续扩展。 |
| 电机状态 | 后续扩展 | 四路 PWM/百分比、run_state、speed_level。当前只显示，不作为控制命令。 |
| 链路统计 | 本机 RX 统计 | 接收帧数、解析错误、过滤丢弃、重复消息、最后接收时间、目标 `sysid`。 |

本机数据使用原则：

- 远程模式主字段不使用本机 Navigation、Environment、System、Alarm、Motor 快照。
- 本机 RTC 可用于计算远端数据年龄和屏幕自身时间，但不能冒充远端设备时间。
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
- 远端时间、湿度、电机状态、完整模块状态和完整告警表补齐。
- ACK、重发、绑定握手、分时双工和远程控制权限。
- 模式切换、远端超时、非法设备等事件写入日志模块。
### 8.2 第二阶段：设备选择与字段补齐

1. 根据 `HEARTBEAT` 建远程设备表，显示 `sysid`、在线/超时、基础状态和链路质量。
2. 增加目标设备选择或固定目标配置，未匹配 `sysid` 的数据不进入远端快照。
3. 补齐远端时间载荷，可优先使用 MAVLink `SYSTEM_TIME` 或后续扩展消息。
4. 补齐湿度、电机状态、完整模块状态和完整告警表。
5. 配置低速链路发送周期，避免 9600 bps 下持续发送导致接收堆积或显示数据过期。
6. 增加远程数据字段级有效位，避免部分字段过期仍显示为有效。

### 8.3 第三阶段：可靠性与控制预留

1. 评估是否需要 ACK 和重发；如果显示只要求最新值，优先保持无 ACK 遥测流。
2. 绑定、模式切换或控制命令优先使用 MAVLink `COMMAND_LONG` / `COMMAND_ACK`。
3. 远端完整快照、电机状态和告警表可使用 `TUNNEL`、`V2_EXTENSION` 或自定义 MAVLink dialect。
4. 只有在 MAVLink 扩展不足或需要统一链路层可靠性时，才新增 LoRa Link 帧头、stop-and-wait、去重和重发。
5. 保留分时双工调度字段或调度模块入口，例如 `slot_id`、`window_ms`、`link_epoch`，但不在第一阶段实现。
6. 不在远程显示阶段执行电机控制命令。
7. 后续启用控制前，必须补充本地急停、命令白名单、权限超时和执行 ACK。

## 9. 验收要点

| 验收项 | 通过条件 |
|---|---|
| LOCAL 模式 | 显示本机数据，左上角显示 `LOCAL`，LoRa 只主动发送本机 MAVLink 遥测，不处理远端 MAVLink 数据。 |
| REMOTE 模式 | KEY0 切换后左上角显示 `REMOTE`，停止主动发送本机周期遥测，开始处理远端 MAVLink 数据。 |
| MAVLink RX | 能解析远端 `HEARTBEAT`、定位、姿态、电池、气压温度和状态文本等消息。 |
| 设备过滤 | 只将目标 `sysid` 的数据写入远程显示快照，其他设备只更新统计或设备表。 |
| 远程显示 | 屏幕主字段来自 `RemoteTelemetry`，不读取本机传感器快照冒充远端数据。 |
| 远程断链 | 停留远程模式并显示远端超时，不自动切回本地。 |
| 数据有效位 | 未收到或已过期字段显示无效，不静默使用本机字段替代。 |
| 带宽 | 9600 bps 下接收帧数、解析错误、过滤丢弃和数据过期统计可观察，周期可配置。 |
| 控制安全 | 当前阶段没有远程电机控制执行路径。 |

