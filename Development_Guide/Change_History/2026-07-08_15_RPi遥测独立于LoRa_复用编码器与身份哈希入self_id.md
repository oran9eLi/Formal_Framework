# 2026-07-08_15_RPi遥测独立于LoRa_复用编码器与身份哈希入self_id

## 背景

承接 `2026-07-08_14`。第 14 篇把 RPi 专属帧(LORASTAT/RIDSTAT/告警表/日志表)从 LoRa 门控解耦,但**标准遥测**(GNSS/姿态/位置/电池/电机/系统健康等)当时仍是"搭 LoRa 便车的镜像"——只在 LoRa 空闲窗口产生,`Lora_E22_Service` 在 E22 未初始化时返回 `IO_ERROR` 导致 `Px4Lite_MavlinkTxRun` 整个不跑,于是 **E22 拔掉/坏了时 RPi 的标准遥测全断**,只剩专属帧 + 身份。

本篇按"一套数据、三条链路、互不混发"的目标,把 RPi 标准遥测也做成**独立出口**:同一批编码器被 LoRa 与 RPi 两趟复用,各走各的目标、各用各的调度,LoRa 忙/掉线时 RPi 仍满调度收全量遥测。另把身份哈希摘要写进 RemoteID 的 `self_id.description`。

## 一、发送目标抽象(target)

新增 `s_send_target`(`MAV_TX_TARGET_LORA` / `MAV_TX_TARGET_RPI`)。编码器内部统一调 `MavTx_SendPrepared()`,由它按当前目标路由:

- 删除旧的"blanket 镜像"(`MavTx_SendRpiCopy` / `MavTx_SendPreparedEx(mirror)`)。
- `MavTx_SendToLora()`:只序列化并发 USART3→E22。
- `MavTx_SendPrepared()`:`target==RPI` → `MavTx_SendRpiExclusive()`(只写 USART6);否则 → `MavTx_SendToLora()`。
- `MavTx_SendRpiExclusive()` 现统一改写 `compid=193`(RPi 专属帧本以 193 编码,幂等;被复用的遥测编码器以 191 编码,这里重贴成 193),并回退 COMM_0 序号——RPi 编码不消耗 LoRa 通道序号,LoRa 对端看到的 seq 保持连续。
- 告警/日志(ALRMHI/ALRMMSK/LOGSYNC)直接调 `MavTx_SendToLora()`,永不进 RPi(RPi 从 TUNNEL 全量表取)。

`Px4Lite_MavlinkTxMirrorToRpi`(公开镜像函数)保留,仍用于 RemoteID 身份帧镜像到 RPi。

## 二、有状态编码器按目标分离

同一编码器被两趟复用,凡"按发送成功推进"的内部状态必须按目标拆成两份,否则一条链路发过后另一条会被误判为已发而漏帧。改为 `[MAV_TX_TARGET_COUNT]` 数组、以 `s_send_target` 为下标:

- `s_module_state_part[2]`(MODSTAT0/1 分片游标)
- `s_motor_pair_part[2]`(MOTOR12/34 分片游标)
- `s_last_battery1_current_sequence[2]` / `s_last_battery2_sequence[2]` / `s_last_battery2_current_sequence[2]`(BAT1CUR/BAT2STAT/BAT2CUR 去重序号)

> `s_alarm_status_part`、日志游标不拆——ALARM/LOG 只发 LoRa。电机 urgent 路径(`s_motor_urgent_*`)也只在 LoRa 趟,不拆。

## 三、RPi 独立遥测目录 + 调度

- 新增 `s_mav_tx_rpi_telem_catalog[]`:复用 16 个 LoRa 编码器(HEARTBEAT/GPS_RAW/GNSS_DETAIL/ATTITUDE/POSITION/BAROALT/SYS_STATUS/MODULE_STATE/BATTERY/BAT1CUR/BAT2STAT/BAT2CUR/GNSSUTC/PRESSURE/ENV_HUM/MOTOR),`scope=ALWAYS`(RPi 出口不受 LoRa 的 NAMED_VALUE 扩展开关约束),周期沿用 LoRa 同值。
- 独立调度:并行数组 `s_rpi_telem_next_ms[]` / `s_rpi_telem_success[]` + 游标 `s_rpi_telem_index`(catalog 项的 next_ms/success 指针置 0,不复用)。各项首发时间错开 30ms。
- `Px4Lite_MavlinkTxRunRpi` 扩为两趟:先跑 RPi 遥测目录(`target=RPI`,各最多一帧),再跑 RPi 专属目录(LORASTAT/RIDSTAT/告警/日志)。整函数不看 LoRa 空闲/服务状态。
- `Px4Lite_MavlinkTxRun` 开头置 `target=LORA`;`Px4Lite_CommWorkRun` 已在 LoRa 门控外无条件调 `TxRunRpi`(第 14 篇)。

**效果**:E22 拔掉/坏了时,`Px4Lite_MavlinkTxRun` 不跑,但 `TxRunRpi` 照跑 → RPi 仍满调度收到 GNSS/姿态/位置/电池/电机/系统健康/告警/日志/身份全量。LoRa 与 RPi 各自独立调度、互不抢占。

## 四、身份哈希摘要写入 RemoteID self_id.description(D2 完善)

`px4lite_remoteid_tx.c` 的 `Px4Lite_RemoteIdTxInit`:`self_id.description` 从纯可读全 ID 改为 `"DCDW-160 H:1A2B3C4D"`——可读身份 + 32 位 UID 哈希摘要(`Px4Lite_IdentityGetUidHash`)。手工按 description 字段长度封顶格式化,不引入 `<stdio.h>`。RPi 用现成 M3c `OPEN_DRONE_ID_SELF_ID` 解码即可取身份指纹,`Px4Lite_IdentityGetUidHash` 至此有了正式使用者。

## 改动文件

- `Framework/Src/px4lite_mavlink_tx.c`:target 常量与 `s_send_target`;5 组有状态变量改 `[2]` 并按目标索引;`SendRpiExclusive` 设 compid=193;删 `SendRpiCopy`/`SendPreparedEx`,新增 `SendToLora` + target 路由的 `SendPrepared`;ALARM/LOG 直发 LoRa;新增 RPi 遥测目录 + 并行调度数组;`TxRunRpi` 两趟;`TxRun` 置 target;Init 复位新状态。
- `Framework/Src/px4lite_remoteid_tx.c`:`self_id.description` 携带哈希摘要。

## 构建与验证

- 无新增文件,Keil 无需改 `.uvprojx`;需重新编译。全部 RPi 遥测路径由 `PX4LITE_ENABLE_RPI_MAVLINK` 守卫。
- **待验证**:① Keil 编译;② CommTask 栈——RPi 遥测趟复用同一批 `mavlink_*_t packet` 局部(与 LoRa 趟不同时,各趟返回后释放),但 `TxRunRpi` 一次最多 2 帧(遥测 1 + 专属 1),确认栈裕量;③ 双端联调:**拔掉 E22**,确认 RPi 仍持续收 GPS/姿态/电池/电机/MODSTAT 等标准遥测(compid=193);④ LoRa 在位时确认 LoRa 对端 seq 连续(RPi 编码回退 COMM_0 生效)、两链路互不抢占;⑤ RPi 解 `OPEN_DRONE_ID_SELF_ID.description` 拿到 `H:` 哈希。

## 落地顺序对照(用户蓝图)

- 第 3 步 拆分 component_id —— ✅(191/192/193)
- 第 4 步 哈希入 RemoteID —— ✅(本篇 self_id.description)
- 第 5 步 RPi MAVLink TX —— ✅(USART6,集成复用而非独立文件;标准遥测已独立)
- 第 6 步 USART1 剥离 DebugConsole —— ✅ 作废(改用 USART6)
- 第 1/2 步 fjlora 并入 + 屏幕 LoRa 显示 —— 🟡 见 fj-lora-fusion,待编译联调
- 第 7 步 RPi 解析端字段表 —— 🟡 见 RPi侧解码对齐清单.md(R1/R2/R5)
