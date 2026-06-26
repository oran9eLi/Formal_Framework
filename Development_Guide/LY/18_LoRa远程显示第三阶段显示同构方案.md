# LoRa 远程显示第三阶段：显示同构方案

本文记录第三阶段的目标、边界和实现拆分。第三阶段不做远程控制，不把 LCD 像素流传给对端；目标是让 REMOTE 模式下的显示变量值尽量与目标设备本机屏幕一致，同时保留显示端自身通信诊断。

## 1. 目标

切换到 REMOTE 模式后，显示端除本机通信状态灯、LoRa 收发数量和本机模式标识外，其余业务显示内容应来自目标设备。

“显示内容相同”指同一组 HMI 变量的值相同，不要求两台设备处在同一页面，也不要求同步屏幕刷新时序。后续页面布局调整时，数据源选择逻辑不得散落在布局绘制代码中。

## 2. 本机保留字段

REMOTE 模式下以下字段仍使用显示端本机数据：

| 字段 | 原因 |
|---|---|
| `DISPLAY_HMI_VAR_REMOTE_MODE` | 表示当前显示端处于 LOCAL 或 REMOTE。 |
| `DISPLAY_HMI_VAR_LORA_STATUS` | 表示显示端自身 LoRa 链路状态。 |
| `DISPLAY_HMI_VAR_LORA_TX_COUNT` | 表示显示端本机发送完成计数。 |
| `DISPLAY_HMI_VAR_LORA_RX_COUNT` | 表示显示端本机接收帧计数。 |
| `DISPLAY_HMI_VAR_LORA_LOSS_RATE` 及链路诊断 | 丢包率由显示端(接收端)按 MAVLink 帧头 `seq` 跳变本机估算——单向链路发送端无回传、自身算不了丢包；连同解析错误、过滤丢弃一律保持本机来源，不取远端值。实现见 `px4lite_mavlink_rx.c` 的 `MavlinkRx_UpdateLoss()`，单位 ‰(0~1000)。 |

时间字段不作为第三阶段同步目标。两端 RTC 均应保持准确，显示端可继续显示本机时间；远端时间载荷保留兼容，不作为“显示同构”验收重点。

## 3. 必须同步的远端内容

REMOTE 模式下以下内容应由目标设备提供：

| 内容 | 当前状态 | 第三阶段要求 |
|---|---|---|
| 导航/姿态 | 已有标准 MAVLink 恢复；ATTITUDE 已降至 2Hz 并截断角速度以省带宽(见第 7 节)。 | 保持远端来源，字段过期保留旧值并标记 stale。 |
| 环境/电源 | 温度、气压、电池已有；湿度用扩展字段。 | 保持远端来源，补齐缺失有效位口径。 |
| 电机 PWM | 已有 `MOTOR12` / `MOTOR34`。 | 保持只读显示，不产生控制命令。 |
| 模块状态灯 | 已有 `MODSTAT` 摘要。 | 扩展为完整显示所需状态；电机灯固定绿灯，不表示 ESC 存在检测。 |
| 告警页 | 当前只有最高告警和活动来源位图。 | 补齐告警表行数据，使告警页与目标设备一致。 |
| 消息日志 | 当前由显示端本地生成。 | 必须同步目标设备消息日志，供 REMOTE 显示和后续 PC LoRa 监控复用。 |

## 4. 数据源重构

第三阶段应先抽离显示数据源，避免继续扩大 `Display_PrepareSnapshot()` 内的 LOCAL/REMOTE 判断。

建议新增 Display ViewModel 层：

```text
Business/App API
  -> Display ViewModel Builder
    -> Display HMI cache
      -> Display Pages / Layout
```

职责划分：

| 模块 | 职责 |
|---|---|
| `display_view_model.h/c` | 根据 LOCAL/REMOTE 模式生成统一 HMI 变量值集合。 |
| `display.c` | 负责触摸、页面切换、刷新调度和把 ViewModel 写入 HMI cache。 |
| `display_pages.c` | 只负责绘制布局和字段，不判断数据来自本机还是远端。 |
| `app_data_api.c` | 提供本机快照和远端快照读取入口。 |
| `px4lite_remote_telemetry.c` | 保存远端只读快照、字段有效位、stale 位和目标设备状态。 |

ViewModel 构建规则：

1. LOCAL 模式使用本机 `App_CopyNavigation()`、`App_CopySystem()`、`App_CopyAlarm()`、`App_CopyEnvironment()`、`App_CopyMotor()`、`App_CopyDateTime()`。
2. REMOTE 模式默认使用 `App_CopyRemoteTelemetry()`。
3. REMOTE 模式按白名单覆盖本机链路字段。
4. 字段未收到时写无效或清零；字段已收到但 stale 时保留最后值。
5. `display_pages.c` 不得直接调用 Business API 或 RemoteTelemetry API。

## 5. 消息日志同步

消息日志不能只从 Display 内部环形缓冲读取，否则后续 PC 监控无法复用。第三阶段应把日志抽成结构化业务事件快照。

推荐结构：

```text
Framework/Business Log Event
  -> Local Display ViewModel
  -> MAVLink TX log extension
  -> RemoteTelemetry log snapshot
  -> Remote Display ViewModel
  -> PC LoRa monitor parser
```

日志条目至少包含：

| 字段 | 说明 |
|---|---|
| `sequence` | 目标设备日志序号，用于去重和增量同步。 |
| `time_hhmmss` 或 `timestamp_ms` | 事件发生时间；不要求同步 RTC，但日志时间应来自事件源设备。 |
| `message_id` | 显示消息枚举或协议消息编号。 |
| `severity` | 信息、警告、错误等级。 |
| `source_id` | 模块或设备来源。 |
| `fault_code` | 关联故障码，无故障时为 0。 |
| `active` | 对告警类消息表示触发或恢复。 |

当前 Display 日志条目本身是**枚举编码**(msg 枚举 id + 时间)，不是自由文本，因此每条只需约 8 字节，同步成本很低——不要为了"省带宽"退化成自由文本截断或逐行 `NAMED_VALUE_INT`。

发送策略：

- 用 `TUNNEL` / `V2_EXTENSION` 一帧承载多条增量日志(布局见第 6 节)，不使用 `NAMED_VALUE_INT` 逐行发送。
- 低速 LoRa 下只发增量(新增条目)，避免每周期重复完整日志表；空闲时用一条 `LOGSEQ` 心跳告知接收端当前序号。
- 接收端按 `sequence` 去重并追加。链路有损且无内容级重传(ARQ)，**允许丢帧造成的序号空洞**，断链后保留最后日志并标记 stale，不要求逐条不丢、不做补传。
- PC 监控应复用同一协议解析，不单独定义另一套日志格式。

## 6. 协议扩展与载荷选择

载荷选择直接决定带宽成败(见第 7 节预算)，第三阶段必须先定死，不再"三选一"：

- **完整告警表行、消息日志增量统一用 `V2_EXTENSION` 或 `TUNNEL` 承载打包载荷**，一帧装多行；字段布局以本节为唯一契约，TX/RX 两端不得各自猜测位定义。
- **禁止用 `NAMED_VALUE_INT` 逐行发送(如 `ALRMROW1` / `LOGROW1`)。** 该消息一帧只带 1 个 int32，外加 10 字节 name 与 12 字节 MAVLink 帧头，搬 4~7 字节有效数据要付约 80% 固定开销；14 行告警表若逐行发约 **840 B/s**，仅此一项就超过 9600bps 链路上限并导致丢帧。
- 模块状态数量固定，现有 `MODSTAT`(单 `NAMED_VALUE_INT`，每模块 4 bit 打包)已够，保持不变。

推荐打包布局(小端，作为初版契约，落地一次定稿后写回本节)：

告警表载荷 = 表头 + N 行：

| 字段 | 字节 | 说明 |
|---|---:|---|
| seq | 1 | 表版本号，变化即整表更新 |
| active_count | 1 | 当前活动行数 N |
| 行[0..N-1] | 7×N | 见下 |

- 告警表行(7 字节)：`source_id`(1) + `fault_code`(2) + `severity`(1) + `active`(1) + `age_s`(2)。
- 消息日志条目(8 字节)：`seq`(2) + `msg_id`(2) + `time_hhmmss`(3) + `severity`(1)。屏幕同构最少只需 `seq + msg_id + time`，`severity`/来源供 PC 监控与告警行复用。

## 7. 带宽预算

链路上限：UART 9600 8N1 = **960 B/s 硬顶**；E22 空中速率已从 2400 提到 9600bps，扣 LoRa 分包开销后可持续 MAVLink 净吞吐约 800 B/s。帧字节 = 12(MAVLink v2 帧头) + payload；REMOTE 模式下链路只跑目标→显示单方向。

当前发送负载(姿态优化后)≈ **483 B/s**，UART 利用率约 50%。姿态此前是单条最大头(200 B/s)，已做两项优化作为第三阶段的预算来源：

- ATTITUDE 频率 5Hz → 2Hz(`PX4LITE_MAVLINK_ATTITUDE_PERIOD_MS` 200→500)；
- 不再发送 `rollspeed`/`pitchspeed`/`yawspeed`(屏幕不显示角速度)，保持为 0 后由 MAVLink v2 尾部零截断把每帧从 40B 降到 28B，仍是标准 ATTITUDE 报文，地面站可正常解析；
- 合计 200 B/s → 56 B/s，省 144 B/s。本机屏幕读本地 topic，不受此值影响。

> 标准 ATTITUDE 的 roll/pitch/yaw 按定义是 float，**不要**为省 6 字节改成自定义整型报文——那会破坏与标准地面站/PC 监控的兼容。真正的浪费是 3 个无人显示的角速度 float，截掉即可。

新增数据占用(均按 `V2_EXTENSION`/`TUNNEL` 打包估算)：

| 数据 | 方案 | 占用 |
|---|---|---:|
| 完整告警表(≤14 行) | TUNNEL 全量 @2s | ~58 B/s |
| 完整告警表 | TUNNEL 仅 active 行 + on-change | 常态 ~0，5 行故障峰值 ~27 B/s |
| 消息日志(9 行，枚举编码) | 增量 + `LOGSEQ` 心跳 @2s | ~15~30 B/s |
| 反面：`NAMED_VALUE_INT` 逐行 | **禁止** | ~840 B/s，溢出 |

目标：常态总发送负载控制在 **~550 B/s(<60% UART)** 以内，给故障突发(告警与日志同时爆发)留出头量；最坏相关性下用 on-change 增量而非每周期全量，把峰值摊平。若后续仍紧，优先继续降姿态/位置频率，而不是砍日志增量(后者本就很小)。

## 8. 实施顺序

1. 电机状态灯口径修复与姿态带宽优化(降频 + 截断角速度)已完成，作为第三阶段干净起点。
2. 新增 Display ViewModel 层，先保持现有显示行为不变。
3. 把 LOCAL/REMOTE 数据源选择从 `Display_PrepareSnapshot()` 迁入 ViewModel。
4. 定义结构化消息日志快照，不再让日志只存在于 `display_pages.c` 内部。
5. 增加日志、完整告警表和完整模块状态的 MAVLink 扩展发送。
6. RemoteTelemetry 接收并保存新增字段。
7. REMOTE ViewModel 使用远端日志和告警表，叠加本机链路字段白名单。
8. 增加桌面单元测试覆盖 LOCAL/REMOTE ViewModel 字段来源。
9. Keil Rebuild All，目标 `0 Error(s), 0 Warning(s)`。

## 9. 验收标准

| 验收项 | 通过条件 |
|---|---|
| 数据源边界 | `display_pages.c` 不包含 LOCAL/REMOTE 数据源判断。 |
| 本机白名单 | REMOTE 下 LoRa 状态灯、TX/RX 计数和模式标识来自显示端本机。 |
| 远端主数据 | REMOTE 下导航、姿态、环境、电源、电机 PWM、模块状态、告警和消息日志来自目标设备。 |
| 消息日志 | 接收端按 `sequence` 增量同步日志，新条目顺序与目标一致；链路有损且无 ARQ，允许丢帧造成的序号空洞，断链保留最后日志并标记 stale，不要求逐条不丢、不做补传；PC 监控复用同一日志载荷。 |
| 告警页 | 告警表行不再只显示最高告警，能显示目标设备活动告警列表。 |
| 断链表现 | 断链后保留最后远端值并标记 stale，不自动切回本机主数据。 |
| 带宽预算 | 常态总发送负载 < ~550 B/s(<60% UART 960 B/s)；告警表/日志用 `TUNNEL`/`V2_EXTENSION` 打包，不使用 `NAMED_VALUE_INT` 逐行。 |
| 构建验证 | Keil Rebuild All 达到 `0 Error(s), 0 Warning(s)`。 |
