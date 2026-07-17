# 2026-07-08_16_温度气压改独立NAMED_VALUE帧_BAROTEMP_BAROPRES

## 背景与决策

原本**温度+气压合在一个标准帧 `SCALED_PRESSURE`** 里发(温度是气压帧的搭车字段),而湿度是独立的 `NAMED_VALUE_INT "HUMIDITY"`。三项环境量收发逻辑不统一。

**决策:以湿度为准**——把温度、气压也各自拆成独立的 `NAMED_VALUE_INT`,与 HUMIDITY 同构,**弃用 SCALED_PRESSURE**。wire name 定为 `BAROTEMP` / `BAROPRES`。

> 代价(已知并接受):环境数据从 1 帧变 3 帧,半双工 LoRa 上占空开销增加;放弃标准 MAVLink 消息(不再被 QGC 等地面站直接识别)。两块板同程序 + 屏幕 RX + RPi 一起改即自洽。

## 一、TX(`px4lite_mavlink_tx.c`)

- 删 `MavTx_SendScaledPressure`(及其 `s_stats.last_pressure_sequence` 去重),新增:
  - `MavTx_SendBaroTemperature` → `NAMED_VALUE_INT name="BAROTEMP"`,`value = 温度×100`(摄氏度厘度,int32,用 `MavTx_SaturateCdegFromFloat`)。
  - `MavTx_SendBaroPressure` → `NAMED_VALUE_INT name="BAROPRES"`,`value = 气压 Pa`(int32)。
  - 与 HUMIDITY 同构:**不去重、每周期发**、只做 fresh 检查。
- 目录:LoRa 主目录与 RPi 遥测目录各把原 `PRESSURE`(SCALED_PRESSURE) 一项**拆成 BAROTEMP+BAROPRES 两项**(NAMED_VALUE_INT;LoRa 侧 scope=EXTENSION,RPi 侧 scope=ALWAYS)。
- 计时/计数:新增 `s_next_baro_temp_ms` / `s_baro_temp_count`(BAROTEMP);BAROPRES 复用 `s_next_pressure_ms` / `s_stats.scaled_pressure_count`。Init 相应增设/复位、首发时间错开(temp +415ms、pres +400ms、hum +430ms)。
- 门控沿用 `PX4LITE_MAVLINK_ENABLE_SCALED_PRESSURE` / `PX4LITE_MAVLINK_PRESSURE_PERIOD_MS`(注释已标注现义)。内部统计名 `scaled_pressure_count` 保留(现计 BAROPRES 数),避免波及 debug 展示层。

## 二、RX(`px4lite_mavlink_rx.c`)

- 删 `MavRx_DecodePressure` 及分发 `case MAVLINK_MSG_ID_SCALED_PRESSURE`。
- 在 NAMED_VALUE_INT 解码里新增两分支(紧邻 HUMIDITY):
  - `BAROTEMP` → `remote->temperature_c = (int32_t)value / 100.0f`
  - `BAROPRES` → `remote->pressure_pa = (float)value`
  - 均置 `PX4LITE_REMOTE_VALID_ENVIRONMENT` + 更新时间戳;三项环境量各自维护、互不清零覆盖(沿用 HUMIDITY 的"不覆盖"约定)。

## 三、编码约定(供 RPi 侧对齐)

| name | 类型 | value 布局 | 还原 |
|---|---|---|---|
| `BAROTEMP` | NAMED_VALUE_INT | 温度 × 100(int32,有符号厘度) | `temperature_c = value / 100.0` |
| `BAROPRES` | NAMED_VALUE_INT | 气压 Pa(int32) | `pressure_pa = value`(Pa) |
| `HUMIDITY` | NAMED_VALUE_INT | 湿度 × 10(0..1000) | `humidity_pct = value / 10.0`(不变) |

`time_boot_ms` = 采样时刻 ms。

## 影响面

- **屏幕显示**:远端环境数据经 RX 写入远端快照(temperature_c/pressure_pa),显示层读快照,无需改;本地环境显示读本地 baro topic,不受影响。
- **LoRa 对端/屏幕 RX**:两块板同程序,一起重编译即自洽。
- **RPi(cns_rpi)**:需在 `extension_decoder.cpp` 加 `BAROTEMP`/`BAROPRES` 两个 name 分支(替代原先对 SCALED_PRESSURE 的标准帧解码)。见 `RPi侧解码对齐清单.md`。

## 改动文件

- `Framework/Src/px4lite_mavlink_tx.c`:删 SCALED_PRESSURE 编码器,加 BAROTEMP/BAROPRES 两编码器;两目录各拆两项;statics/init 增设。
- `Framework/Src/px4lite_mavlink_rx.c`:删 SCALED_PRESSURE 解码器+case;加 BAROTEMP/BAROPRES name 分支。
- `Framework/Inc/px4lite_config.h`:ENABLE_SCALED_PRESSURE 注释标注现义。

## 构建与验证

- 无新增文件,需重新编译。
- **待验证**:① Keil 编译;② 双端联调:确认对端收到 `BAROTEMP`/`BAROPRES` 后温度、气压显示正确(温度含负值场景验 int32 符号);湿度不受影响;③ RPi 侧补 `BAROTEMP`/`BAROPRES` 解码后端到端验证;④ 半双工 LoRa 占空:环境帧由 1→3,确认调度预算仍在 40% 内。
