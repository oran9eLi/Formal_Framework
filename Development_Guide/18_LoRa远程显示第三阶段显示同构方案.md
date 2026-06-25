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
| 后续链路诊断字段 | 解析错误、过滤丢弃、丢包率等如果用于监控显示端链路，应保持本机来源。 |

时间字段不作为第三阶段同步目标。两端 RTC 均应保持准确，显示端可继续显示本机时间；远端时间载荷保留兼容，不作为“显示同构”验收重点。

## 3. 必须同步的远端内容

REMOTE 模式下以下内容应由目标设备提供：

| 内容 | 当前状态 | 第三阶段要求 |
|---|---|---|
| 导航/姿态 | 已有标准 MAVLink 恢复。 | 保持远端来源，字段过期保留旧值并标记 stale。 |
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

发送策略：

- 第一版可使用 `NAMED_VALUE_INT` 发送紧凑日志摘要，或使用 `TUNNEL` / `V2_EXTENSION` 承载多字段日志。
- 低速 LoRa 下优先发送增量日志，避免每周期重复完整日志表。
- 接收端按 `sequence` 去重，断链后保留最后日志，重新收到新日志后继续追加。
- PC 监控应复用同一协议解析，不单独定义另一套日志格式。

## 6. 协议扩展建议

第三阶段可继续沿用现有标准 MAVLink + `NAMED_VALUE_INT` 扩展，但完整告警表和消息日志字段较多，建议为两类数据预留更结构化的载荷：

| 数据 | 推荐载荷 | 说明 |
|---|---|---|
| 完整模块状态 | `NAMED_VALUE_INT` 分片或 `V2_EXTENSION` | 模块状态数量固定，可压缩分片。 |
| 告警表行 | `V2_EXTENSION` / `TUNNEL` | 每行包含 source、fault、severity、active、updated。 |
| 消息日志增量 | `V2_EXTENSION` / `TUNNEL` | 使用 sequence 去重，供屏幕和 PC 同时消费。 |

如果继续使用 `NAMED_VALUE_INT`，命名应保持固定且短，例如 `LOGSEQ`、`LOGROW1`、`ALRMROW1`。字段布局必须写入本文或协议文档，禁止在 TX/RX 两端各自猜测位定义。

## 7. 实施顺序

1. 提交当前电机状态灯口径修复，保持第三阶段工作起点干净。
2. 新增 Display ViewModel 层，先保持现有显示行为不变。
3. 把 LOCAL/REMOTE 数据源选择从 `Display_PrepareSnapshot()` 迁入 ViewModel。
4. 定义结构化消息日志快照，不再让日志只存在于 `display_pages.c` 内部。
5. 增加日志、完整告警表和完整模块状态的 MAVLink 扩展发送。
6. RemoteTelemetry 接收并保存新增字段。
7. REMOTE ViewModel 使用远端日志和告警表，叠加本机链路字段白名单。
8. 增加桌面单元测试覆盖 LOCAL/REMOTE ViewModel 字段来源。
9. Keil Rebuild All，目标 `0 Error(s), 0 Warning(s)`。

## 8. 验收标准

| 验收项 | 通过条件 |
|---|---|
| 数据源边界 | `display_pages.c` 不包含 LOCAL/REMOTE 数据源判断。 |
| 本机白名单 | REMOTE 下 LoRa 状态灯、TX/RX 计数和模式标识来自显示端本机。 |
| 远端主数据 | REMOTE 下导航、姿态、环境、电源、电机 PWM、模块状态、告警和消息日志来自目标设备。 |
| 消息日志 | 接收端消息日志内容和顺序与目标设备一致，PC 监控可解析同一日志载荷。 |
| 告警页 | 告警表行不再只显示最高告警，能显示目标设备活动告警列表。 |
| 断链表现 | 断链后保留最后远端值并标记 stale，不自动切回本机主数据。 |
| 构建验证 | Keil Rebuild All 达到 `0 Error(s), 0 Warning(s)`。 |
