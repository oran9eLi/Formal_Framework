# 固件侧 vs RPi 侧改动分工

版本：2026-07-07
用途：明确"树莓派链路对接"里哪些改动落在**固件**（`Formal_Framework`），哪些落在 **RPi**（`cns_rpi`）。
总原则：**方案 A —— 固件是已验证的多方权威（TX↔LoRa 对端↔屏幕 RX 共用一套 name/编码），RPi 作为新接入的只读方去对齐固件**。仅两处例外反向落在固件（见 §3）。
配套详表：RPi 侧每条的位布局与解码片段见 `Development_Guide/RPi侧解码对齐清单.md`，本文只讲分工，不重复实现。

---

## 1. 一句话结论

- **绝大多数改动在 RPi 侧**（加/改 `extension_decoder.cpp` 的 name 分支去对齐固件现状）。
- **固件侧只有 1 项必改**（身份数据镜像到 USART1）+ **1 项待决**（告警/日志是否要全量）。
- 我上一轮已完成的固件改动（`LORASTAT`/`RIDSTAT`/`RPIIDENT` 只发 USART1）**保持不变**，RPi 补解码即可。

---

## 2. 主表：每项数据 → 谁改

| # | 数据项 | 固件现状（发什么） | RPi 现状（解什么） | **固件改？** | **RPi 改？** |
|---|---|---|---|:---:|:---:|
| 1 | 标准遥测(HEARTBEAT/GPS_RAW/ATTITUDE/POSITION/SYS_STATUS/BATTERY/PRESSURE) | 官方标准帧 | ✅ 已解 | — | — |
| 2 | `GNSS_SAT` | ✅ | ✅ 已解 | — | — |
| 3 | `BAT2STAT` | ✅ | ✅ 已解 | — | — |
| 4 | `MODSTAT0`/`MODSTAT1` | ✅ | ✅ 已解 | — | — |
| 5 | 湿度 | 发 `HUMIDITY` | 键的是 `ENVHUM` | — | ✅ 加别名 |
| 6 | 电机 | 发 `MOTOR12`+`MOTOR34`(各2路) | 等单条 `MOTORPWM` | — | ✅ 换分支 |
| 7 | 告警 | 发 `ALRMHI`+`ALRMMSK`(摘要) | 等 `TUNNEL 0x8001`(全表) | ⚠️ 见 D1 | ✅ 加摘要解码 |
| 8 | 日志 | 发 `LOGSYNC`(单条增量) | 等 `TUNNEL 0x8002`(批量) | ⚠️ 见 D1 | ✅ 加增量去重 |
| 9 | `BAROALT`/`GNSSUTC` | ✅ 发 | 未解(可选) | — | ◻ 可选 |
| 10 | **`LORASTAT`**(LoRa状态) | ✅ 已发(我实现) | 未解 | — | ✅ 新增分支 |
| 11 | **`RIDSTAT`**(RemoteID状态) | ✅ 已发(我实现) | 未解 | — | ✅ 新增分支 |
| 12 | **身份**(OPEN_DRONE_ID_*) | 只发 UART4→ESP32，**没发 RPi** | ✅ M3c 已就绪，缺输入 | ✅ **镜像到 USART1** | — |
| 13 | `RPIIDENT`(STATUSTEXT 身份文本) | ✅ 已发(我实现，过渡用) | 未解 | ◻ 见 D2 后可废弃 | ◻ 过渡兜底 |

图例：✅=需要做 / —=无需改 / ◻=可选 / ⚠️=取决于决策。

---

## 3. 固件侧改动清单（`Formal_Framework`）

**必做（1 项）**

- **[F1] 身份数据镜像到 USART1**：目前 `OPEN_DRONE_ID_BASIC_ID/LOCATION/SYSTEM/OPERATOR_ID/SELF_ID` 只由 `px4lite_remoteid_tx.c` 发往 UART4→ESP32，**不到 RPi**。需新增一条 RPi 出口，把这几条也用 compid=193 发到 USART1（做法同我加的 `LORASTAT`：在 `px4lite_mavlink_tx.c` 目录里加 RPi 专属项，或让 remoteid 调度另发一份）。
  - 理由：RPi 的 M3c 身份解码**已写好并验证**，喂 `OPEN_DRONE_ID_*` 即可用，比解析自由文本干净。
  - 完成后：`RPIIDENT` STATUSTEXT 可废弃（见 D2）。

**待决（1 项，取决于 D1）**

- **[F2] 告警/日志全量出口**（**仅当 D1 决定"要全量"时才做**）：固件现在只发摘要（`ALRMHI/ALRMMSK`）和单条增量（`LOGSYNC`）。若服务器需要完整 14 行告警表 / 批量日志，需新增全量数据出口（重新引入 TUNNEL 或用更宽的 NAMED_VALUE，作为 RPi 专属帧）。若 D1 决定"摘要够用" → **固件不改**。

**保持不变**

- 我上一轮实现的 `LORASTAT`/`RIDSTAT`/`RPIIDENT`（只发 USART1、不上 LoRa、独立序号）已就位，无需再动。
- 现有所有 LoRa/屏幕共用的 name（`HUMIDITY`/`MOTOR12/34`/`ALRMHI/ALRMMSK`/`LOGSYNC` 等）**不许改**——它们是已验证的三方契约，改了会波及 LoRa 对端和屏幕 RX。

---

## 4. RPi 侧改动清单（`cns_rpi`，全部在 `src/protocol/extension_decoder.cpp` + `src/state/state_store.*`）

**必做**

- **[R1]** `HUMIDITY`：`ENVHUM` 分支加 `|| name=="HUMIDITY"`。
- **[R2]** `MOTOR12`/`MOTOR34`：替换 `MOTORPWM` 分支，两帧各写 2 路、勿互相覆盖。
- **[R3]** `ALRMHI`/`ALRMMSK`：新增告警摘要解码（替代 TUNNEL 0x8001 分支）。
- **[R4]** `LOGSYNC`：新增单条增量解码 + 按 `sequence` 去重环（替代 TUNNEL 0x8002 分支）。
- **[R5]** `LORASTAT`/`RIDSTAT`：新增两个 name 分支 + 两个 `state_store` 结构。
- **[R6]** 回归确认 `GNSS_SAT/BAT2STAT/MODSTAT0/1` 不受影响。

**可选**

- **[R7]** `BAROALT`/`GNSSUTC`（RPi 若已从标准帧取高度/时间可不做）。
- **[R8]** STATUSTEXT(`RPIIDENT`) 过渡解码：仅在 F1 未落地前临时用，F1 完成即删。

**无需改**

- `OPEN_DRONE_ID_*` 解码（M3c 已就绪）——只等固件 F1 把数据送上来。

位布局、枚举、模块编号、可直接粘的 C++ 片段：见 `RPi侧解码对齐清单.md` §2–§5。

---

## 5. 两个决策点 —— 已拍板（2026-07-07）

| 决策 | 结论 | 对分工的影响 |
|---|---|---|
| **D1 告警/日志完整性** | **全量表** | 固件做 **F2**：发完整告警表/批量日志（**TUNNEL 0x8001/0x8002**）。RPi **无需改**——现成 `DecodeTunnel` 已解码。原 R3/R4（解 ALRMHI/LOGSYNC 摘要）**取消**。 |
| **D2 身份传递方式** | **走 OPEN_DRONE_ID_*** | 固件做 **F1**：把 RemoteID 身份帧镜像到 USART1。RPi **无需改**——现成 M3c 已解码。`RPIIDENT` STATUSTEXT 已从固件删除，原 R8 **取消**。 |

**实现状态**：F1 + F2 已在固件落地（见 `Change_History/2026-07-07_12`）。因 D1 选 TUNNEL、D2 选 OpenDroneID，两块都复用 RPi 已验证解码，RPi 侧工作量比原估计**显著缩小**（见下）。

## 5.1 决策落定后的最终分工（覆盖 §2–§4）

**固件侧（已实现）**：F1 身份镜像、F2 TUNNEL 告警表+日志、删 RPIIDENT。其余不动。

**RPi 侧（待做，仅 4 项）**：
- **R1** `HUMIDITY`（别名 ENVHUM）
- **R2** `MOTOR12`/`MOTOR34`（替代 MOTORPWM）
- **R5** `LORASTAT`/`RIDSTAT` 新增 + 两个 state 结构
- **R6** 回归

**RPi 侧无需改**：告警/日志（现成 `DecodeTunnel`）、身份（现成 M3c）。原 **R3/R4/R8 取消**。可选 R7（BAROALT/GNSSUTC）不变。

---

## 6. 建议推进顺序

1. **拍 D1/D2**（各一句话即可）。
2. RPi 侧并行做 **R1–R6**（不依赖固件，随时可开工、可用固件现有输出联调）。
3. 固件侧做 **F1**（身份镜像）；D1 若选全量再排 **F2**。
4. F1 落地后，RPi 删 R8、启用 M3c 身份解码联调。
5. 两边各自回归 + 一次双端联调（对齐 `sysid` = `PX4LITE_UNIT_ID`，确认 compid=193 帧被正常收下）。

---

## 7. 汇总一句话

**RPi 侧改一批（R1–R6 必做，R7/R8 可选），固件侧只改一项（F1 身份镜像）外加一项待决（F2 视 D1）。** 其余固件不动——因为固件那套 name 是 LoRa+屏幕已验证的共享契约。
