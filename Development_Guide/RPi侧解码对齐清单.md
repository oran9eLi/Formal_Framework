# RPi 侧解码对齐清单（cns_rpi 改动指引）

版本：2026-07-07
方向：**方案 A —— RPi 端对齐固件现状**（固件是已验证的多方权威：TX↔LoRa 对端↔屏幕 RX 三方共用同一套 name/编码，不动固件）。
适用对象：`oran9eLi/cns_rpi`，改 `src/protocol/extension_decoder.cpp` 及 `src/state/state_store.*`。
数据权威来源：固件 `Framework/Src/px4lite_mavlink_tx.c`（本清单每条都标了源码行为，位布局逐字段核对过，可直接实现）。

---

## 0. 背景：为什么要改 RPi，不改固件

固件的 `NAMED_VALUE_INT` name 同时喂着 **LoRa 对端**和**屏幕自己的远端显示解码器**（`px4lite_mavlink_rx.c` 里就是 `HUMIDITY/MOTOR12/MOTOR34/ALRMHI/ALRMMSK/LOGSYNC` 这套），是已跑通、已双板联调验证的既定契约。RPi 是最后接入的第四方、只读、还在开发中（M4/M5 未接），且解码器就是个硬编码 switch —— 让 RPi 跟随固件，风险最小、改动最省。

RPi 现状（`src/main.cpp` + `extension_decoder.cpp`）：每条 CRC 通过的帧**只按 msgid 分发，不按 compid/sysid 过滤**（compid=193 的帧照收，sysid 仅用于拼 `DCDW-XXX`）。`NAMED_VALUE_INT` 按 **name 字符串**硬编码分支，不认识的 name **静默丢弃、不报错**。所以下面每条只要在 `DecodeNamedValueInt` 里加/改一个 `if (name == "...")` 分支即可。

---

## 1. 全量对照表（固件实际发的 vs RPi 现状）

| 固件 wire name | RPi 现状 | 动作 |
|---|---|---|
| `GNSS_SAT` | ✅ 已解 | 无需改 |
| `BAT2STAT` | ✅ 已解 | 无需改 |
| `MODSTAT0` / `MODSTAT1` | ✅ 已解 | 无需改 |
| `HUMIDITY` | ❌ RPi 键的是 `ENVHUM` | **改名**（见 §2.1） |
| `MOTOR12` / `MOTOR34` | ❌ RPi 等单条 `MOTORPWM` | **换分支**（见 §2.2） |
| `ALRMHI` / `ALRMMSK` | ❌ RPi 等 `TUNNEL 0x8001` | **新增两分支**（见 §2.3，注意告警完整性决策） |
| `LOGSYNC` | ❌ RPi 等 `TUNNEL 0x8002` | **新增分支**（见 §2.4，注意日志完整性决策） |
| `BAROALT` | ❌ 未解 | 可选新增（见 §2.5） |
| `GNSSUTC` | ❌ 未解 | 可选新增（见 §2.5） |
| `LORASTAT` | ❌ 未解（RPi 专属新消息） | **新增分支**（见 §2.6） |
| `LORATX` / `LORARX` | ❌ 未解（RPi 专属新消息） | **新增分支**（见 §2.6b） |
| `RIDSTAT` | ❌ 未解（RPi 专属新消息） | **新增分支**（见 §2.7） |
| `RPIIDENT`(STATUSTEXT) | ❌ 未解（RPi 专属新消息） | 可选（见 §3，建议改用 OPEN_DRONE_ID_*） |

> 位布局约定：`value` 是 `int32_t`，按 `uint32_t bits` 解读；`[a:b]` 指 bit a..b（含），bit0 = LSB。多字节整数无字节序问题（就是一个 32bit 整数按位取）。

---

## 2. 逐条改动（`extension_decoder.cpp` → `DecodeNamedValueInt`）

### 2.1 `HUMIDITY`（改名，布局与 ENVHUM 相同）

源码 `MavTx_SendEnvHumidity`：`value` = 相对湿度 ×10（0..1000，如 535=53.5%）；`time_boot_ms` = 采样时刻 ms。

把现有 `if (name == "ENVHUM")` 改为**同时接受两者**（保持向后兼容最稳）：

```cpp
if (name == "HUMIDITY" || name == "ENVHUM") {
  state::EnvHumidity hum{};
  hum.relative_humidity_x10 = static_cast<std::uint16_t>(bits);
  store.UpdateEnvHumidity(hum);
  return true;
}
```

### 2.2 `MOTOR12` / `MOTOR34`（换分支：两帧，各 2 路）

源码 `MavTx_SendMotorPair`。固件把 4 路电机拆成两帧发（不是 RPi 原来假设的单条 `MOTORPWM` 4 字节）：

| name | value 位布局 |
|---|---|
| `MOTOR12` | `[0:7]`电机1占空% `[8:15]`电机2占空% `[16]`run_state(0/1) `[24:31]`speed_level% |
| `MOTOR34` | `[0:7]`电机3占空% `[8:15]`电机4占空% `[16]`run_state `[24:31]`speed_level |

`time_boot_ms` = 电机采样时刻 ms。两帧的 `run_state`/`speed_level` 是同一份整机状态的冗余拷贝。

```cpp
if (name == "MOTOR12" || name == "MOTOR34") {
  state::MotorPwm pwm = store.SnapshotMotorPwm();      // 在已有值上就地更新，避免另一帧被覆盖
  const std::size_t base = (name == "MOTOR12") ? 0 : 2;
  pwm.duty_percent[base + 0] = static_cast<std::uint8_t>(bits & 0xFF);
  pwm.duty_percent[base + 1] = static_cast<std::uint8_t>((bits >> 8) & 0xFF);
  pwm.run_state    = ((bits >> 16) & 0x1) != 0;
  pwm.speed_level  = static_cast<std::uint8_t>((bits >> 24) & 0xFF);
  store.UpdateMotorPwm(pwm);
  return true;
}
```

### 2.3 `SERVO_OUTPUT_RAW`（新增四路 PWM 脉宽）

固件同时发送标准 MAVLink `SERVO_OUTPUT_RAW`（msgid 36，`port=0`）：

| 字段 | 含义 |
|---|---|
| `servo1_raw` | 电机 1 PWM 高电平脉宽，单位 us |
| `servo2_raw` | 电机 2 PWM 高电平脉宽，单位 us |
| `servo3_raw` | 电机 3 PWM 高电平脉宽，单位 us |
| `servo4_raw` | 电机 4 PWM 高电平脉宽，单位 us |
| `time_usec` | Control 电机快照采样时刻，单位 us |

当前有效范围为 1000~2000 us，发送周期为 1000 ms。`MOTOR12/MOTOR34` 继续保留，RPi 可用前者显示精确脉宽，用后者读取百分比、`run_state` 和 `speed_level`。

> 若不想在 `state_store` 加读回接口，可在 `MotorPwm` 里存 4 路，两帧各写自己那 2 路即可；关键是**别用单帧覆盖另一半**。原 `MOTORPWM` 分支可删除或保留兼容。

### 2.3 `ALRMHI` / `ALRMMSK`（告警摘要）—— ⚠️ 已作废（D1=全量表）

> **本节作废**：决策 D1 已定为"全量表"。固件改为发 **TUNNEL 0x8001 完整告警表**（见 `Change_History/2026-07-07_12`），RPi 用**现成的 `DecodeTunnel`/`DecodeAlarmTable` 即可，无需改**。下面的摘要解码不再需要，仅存档参考。

源码 `MavTx_SendAlarmStatus`，两帧交替发，`time_boot_ms` = 告警发布时刻 ms：

| name | value 位布局 | 含义 |
|---|---|---|
| `ALRMHI` | `[0:15]`fault_code `[16:23]`source_id `[24:27]`severity | 当前**最高**活动告警的摘要 |
| `ALRMMSK` | `[0:31]`活动模块位掩码，bit i = 模块 i 是否有活动告警 | 哪些模块正在告警 |

模块序号见 §4；severity 枚举见 §4。

```cpp
if (name == "ALRMHI") {
  state::AlarmSummary s = store.SnapshotAlarmSummary();
  s.highest_fault_code = static_cast<std::uint16_t>(bits & 0xFFFF);
  s.highest_source_id  = static_cast<std::uint8_t>((bits >> 16) & 0xFF);
  s.highest_severity   = static_cast<std::uint8_t>((bits >> 24) & 0xF);
  store.UpdateAlarmSummary(s);
  return true;
}
if (name == "ALRMMSK") {
  state::AlarmSummary s = store.SnapshotAlarmSummary();
  s.active_mask = bits;                 // bit i = 模块 i 有活动告警
  store.UpdateAlarmSummary(s);
  return true;
}
```

> ⚠️ **完整性决策（需与服务器/上位确认）**：这套是**摘要**（最高项 + 活动掩码），**不是** RPi 原契约里 TUNNEL 0x8001 的完整 14 行告警表。若 MQTT 只需"有没有告警/最严重是哪条/哪些模块在告警" → 摘要足够，按上面解即可。若必须上报**逐行完整告警表** → 这是固件侧要新增的数据出口（fj-lora 融合已删掉 TUNNEL），不在本清单范围，需另立需求。

### 2.4 `LOGSYNC`（日志增量）—— ⚠️ 已作废（D1=全量表）

> **本节作废**：决策 D1 已定为"全量表"。固件改为发 **TUNNEL 0x8002 批量日志**（一帧≤9 条，周期重播，见 `Change_History/2026-07-07_12`），RPi 用**现成的 `DecodeTunnel`/`DecodeMessageLog` 即可，无需改**。注意固件把 `time` 字段按"时/分/秒各 1 字节"填充（与 RPi `time_hhmmss[3]` 原样存储一致）。下面的单条增量解码不再需要，仅存档参考。

源码 `MavTx_SendMessageLog`：**每帧一条**日志，增量发送；固件每 20s 会把历史全量重发一遍，靠 `sequence` 去重（收到重复 sequence 丢弃即可）。`time_boot_ms` = 事件时间 `HHMMSS`（十进制，如 143025 = 14:30:25）。

`value` 位布局（**压缩单条**，注意 message_id 只带低 8 位）：

| 段 | 含义 |
|---|---|
| `[0:15]` | sequence（单调序号，从 1 起） |
| `[16:23]` | message_id 低 8 位 |
| `[24:27]` | severity |
| `[28]` | active（告警类触发/恢复标志） |
| `[29:31]` | source_id 低 3 位 |

```cpp
if (name == "LOGSYNC") {
  state::LogEntry e{};
  e.sequence   = static_cast<std::uint16_t>(bits & 0xFFFF);
  e.message_id = static_cast<std::uint8_t>((bits >> 16) & 0xFF);
  e.severity   = static_cast<std::uint8_t>((bits >> 24) & 0xF);
  e.active     = ((bits >> 28) & 0x1) != 0;
  e.source_id  = static_cast<std::uint8_t>((bits >> 29) & 0x7);
  e.time_hhmmss = static_cast<std::uint32_t>(value.time_boot_ms);  // 十进制 HHMMSS
  store.AppendLogEntryDedup(e);         // 按 sequence 去重后追加
  return true;
}
```

> ⚠️ **完整性决策**：RPi 原 TUNNEL 0x8002 一帧可带 9 条；固件现在一帧一条 + 20s 全量重播。RPi 需自建一个按 sequence 去重的日志环（替代原 `MessageLog` 批量填充）。若认为增量+重播够用即可；若必须批量帧，同 §2.3 属固件侧新增工作。

### 2.5 `BAROALT` / `GNSSUTC`（可选）

| name | value | time_boot_ms | 说明 |
|---|---|---|---|
| `BAROALT` | 融合高度 mm（**int32 有符号**） | now_ms | 不依赖 GPS 定位，GPS 未锁星时也有值；RPi 若已从 GLOBAL_POSITION_INT 取高度可不接 |
| `GNSSUTC` | UTC 日期，压缩 `yymmdd`（如 260707=2026-07-07；无效为 0） | 当日 UTC 秒数 | 日期在 value、当日秒数在 time_boot_ms；来源为统一时间服务，GNSS 已同步时来自 GNSS，未定位时可由 RTC 兜底 |

```cpp
if (name == "BAROALT") { store.UpdateBaroAltMm(value.value /*int32*/); return true; }
if (name == "GNSSUTC") { store.UpdateGnssUtc(static_cast<std::uint32_t>(value.value), value.time_boot_ms); return true; }
```

### 2.5b 温度/气压/电池 —— 已回退/改用官方 MAVLink 通道（2026-07-08_22）

> **本节的 BAROTEMP/BAROPRES 方案已作废**。依据《建议：部分自定义 NAMED_VALUE_INT 改走官方通道》，固件已把这些改回/收敛到官方 MAVLink 消息(见 Change_History/2026-07-08_22)：
>
> - **温度/气压** → 官方 `SCALED_PRESSURE`(msgID 29)：`temperature`(cdegC)、`press_abs`(hPa)。RPi 用现成 SCALED_PRESSURE 解码,**不再解 BAROTEMP/BAROPRES**。
> - **电池1电流** `BAT1CUR` → **删除**(与 `BATTERY_STATUS.current_battery` 重复,RPi 以官方字段为准)。
> - **电池2** `BAT2STAT`/`BAT2CUR` → 官方 `BATTERY_STATUS(id=1)`(多电池机制)。RPi 按 `battery_status.id` 分流:id=0 电池1、id=1 电池2,`BAT2STAT`/`BAT2CUR` 自定义解码与 `Battery2Status` 结构体可一并删除。
>
> 过渡:RPi M4 可先按真机当前格式接入,固件切官方通道后再删对应自定义分支、改读官方字段。

### 2.6 `LORASTAT`（RPi 专属新增：LoRa 链路状态）

源码 `MavTx_SendRpiLoraStatus`（只发 USART1，不上 LoRa）。`time_boot_ms` = now_ms。

| 段 | 含义 |
|---|---|
| `[0:15]` | 接收侧估算丢包率 ×10（0..1000，即 0.0%..100.0%） |
| `[16:23]` | LoRa 节点 ID |
| `[24]` | LoRa 模块在位（0/1） |
| `[25:27]` | LoRa 链路状态枚举 `Px4Lite_State_t`（见 §4，0..6） |

```cpp
if (name == "LORASTAT") {
  state::LoraStatus s{};
  s.loss_rate_x10 = static_cast<std::uint16_t>(bits & 0xFFFF);
  s.node_id       = static_cast<std::uint8_t>((bits >> 16) & 0xFF);
  s.present       = ((bits >> 24) & 0x1) != 0;
  s.link_state    = static_cast<std::uint8_t>((bits >> 25) & 0x7);
  store.UpdateLoraStatus(s);
  return true;
}
```

### 2.6b `LORATX` / `LORARX`（RPi 专属：LoRa 收发计数）

源码 `MavTx_SendRpiLoraTxCount` / `MavTx_SendRpiLoraRxCount`，均为 `NAMED_VALUE_INT`，只发 USART6，不上 LoRa，也不改变 `LORASTAT` 原有位布局。

| name | value | time_boot_ms | 说明 |
|---|---|---|---|
| `LORATX` | `tx_frame_count`，本机 LoRa 发送流程完成帧计数 | `last_tx_ms` | 发送完成以 UART DMA TC 回调统计为准，不代表对端收到 |
| `LORARX` | `rx_frame_count`，本机 LoRa 接收完整合法 MAVLink 帧计数 | `last_rx_ms` | 接收侧完整帧计数 |

```cpp
if (name == "LORATX") {
  store.UpdateLoraTxCount(static_cast<std::uint32_t>(value.value), value.time_boot_ms);
  return true;
}
if (name == "LORARX") {
  store.UpdateLoraRxCount(static_cast<std::uint32_t>(value.value), value.time_boot_ms);
  return true;
}
```

### 2.7 `RIDSTAT`（RPi 专属新增：RemoteID 广播状态）

源码 `MavTx_SendRpiRemoteIdStatus`。`time_boot_ms` = RemoteID **最近一次成功提交时间 ms**（RPi 可据此判断广播是否仍在推进：该值长时间不变 = 停发）。

| 段 | 含义 |
|---|---|
| `[0:15]` | 位置广播成功计数（低 16 位，看**增量**而非绝对值） |
| `[16:31]` | 编码/提交错误计数（低 16 位，同上看增量） |

```cpp
if (name == "RIDSTAT") {
  state::RemoteIdStatus s{};
  s.location_count = static_cast<std::uint16_t>(bits & 0xFFFF);
  s.error_count    = static_cast<std::uint16_t>((bits >> 16) & 0xFFFF);
  s.last_success_ms = value.time_boot_ms;
  store.UpdateRemoteIdStatus(s);
  return true;
}
```

---

## 3. `RPIIDENT`（STATUSTEXT）—— 可选，建议改走 OPEN_DRONE_ID_*

固件另发一条 `STATUSTEXT`（compid 193），文本形如 `ID=DCDW-160 NODE=160 HASH=1A2B3C4D`，携带节点 ID + 完整身份串 + 32 位身份哈希。但：

- RPi 当前 `telemetry_decoder`/`extension_decoder` **都不解 STATUSTEXT**，直接丢。
- **更好的路径**：RPi 的 M3c 已经能解 `OPEN_DRONE_ID_BASIC_ID/LOCATION/SYSTEM/OPERATOR_ID/SELF_ID`（含 `uas_id`→vendor_id、sysid→DCDW）。身份数据应让**固件把 RemoteID 那几条也镜像到 USART1**（固件侧小改），RPi 用现成的 M3c 解码即可，比解析这条自由文本干净得多。

因此 `RPIIDENT` 定位为**过渡/兜底**：如果短期内固件还没把 OPEN_DRONE_ID_* 接到 RPi 链路，可临时在 RPi 加一个 `MAVLINK_MSG_ID_STATUSTEXT` 分支，`sscanf`/字符串切分取 `NODE=`/`HASH=`；一旦 OPEN_DRONE_ID_* 到位就废弃它。

---

## 4. 枚举与模块编号（供上面各分支引用）

**`Px4Lite_State_t`（LORASTAT.link_state / MODSTAT 每 4bit）**：0 UNINIT / 1 STARTING / 2 ONLINE / 3 DEGRADED / 4 OFFLINE / 5 FAILED / 6 DISABLED。

**告警 severity**：0 INFO / 1 WARNING / 2 ERROR / 3 CRITICAL / 4 FATAL。

**模块序号（ALRMMSK 位序 / MODSTAT 打包序，共 14）**：
0 GNSS · 1 IMU · 2 BARO · 3 BATTERY · 4 LORA · 5 5G · 6 STORAGE · 7 REMOTE_ID · 8 DISPLAY · 9 CONTROL · 10 ALARM · 11 SYSTEM · 12 ESTIMATOR · 13 BUSINESS。

---

## 5. `state_store` 需要新增的结构（`src/state/state_store.hpp`）

```cpp
struct LoraStatus     { std::uint16_t loss_rate_x10; std::uint8_t node_id; bool present; std::uint8_t link_state; };
struct RemoteIdStatus { std::uint16_t location_count; std::uint16_t error_count; std::uint32_t last_success_ms; };
struct AlarmSummary   { std::uint16_t highest_fault_code; std::uint8_t highest_source_id; std::uint8_t highest_severity; std::uint32_t active_mask; };
```
配套 `Update*` / `Snapshot*` 接口，以及 §2.4 的日志去重环。若沿用原 `AlarmTable`/`MessageLog`（TUNNEL 版）就先并存、待完整性决策后再收敛。

---

## 6. 改动检查清单

- [ ] `HUMIDITY`：`ENVHUM` 分支加 `|| name=="HUMIDITY"`。
- [ ] `MOTOR12`/`MOTOR34`：替换 `MOTORPWM` 分支，两帧各写 2 路、勿互相覆盖。
- [ ] `ALRMHI`/`ALRMMSK`：新增摘要解码；确认服务器是否只要摘要（否则告警全量表另立需求）。
- [ ] `LOGSYNC`：新增单条增量解码 + 按 sequence 去重环；确认是否接受增量+重播模型。
- [ ] `LORASTAT`/`LORATX`/`LORARX`/`RIDSTAT`：新增分支 + 对应 state/update 接口。
- [ ] （可选）`BAROALT`/`GNSSUTC`。
- [ ] **身份**：不改 RPi 解码（M3c 已就绪），改由**固件**把 OPEN_DRONE_ID_* 镜像到 USART1；`RPIIDENT` STATUSTEXT 仅作过渡兜底。
- [ ] 回归：确认新增/改名分支不影响已验证的 `GNSS_SAT/BAT2STAT/MODSTAT0/1`。

---

## 7. 两处不属于"纯改 RPi"的例外（提醒）

1. **身份数据**：RPi M3c 解码已就绪，缺的是固件把 RemoteID（现走 UART4→ESP32）也发一份到 USART1。→ **固件侧小改**，非 RPi 改动。
2. **告警表 / 日志完整性**：固件现发摘要（ALRMHI/ALRMMSK）与单条增量（LOGSYNC），非完整 14 行表 / 批量日志。是否够用是**产品/服务器决策**；若需全量，是固件侧新增数据出口，非 RPi 改动。
