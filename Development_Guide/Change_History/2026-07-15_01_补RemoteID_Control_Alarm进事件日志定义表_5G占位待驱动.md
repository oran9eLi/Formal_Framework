# 2026-07-15 01 补 RemoteID/Control/Alarm 进事件日志定义表，5G 占位待驱动

## 背景

审查日志模块时发现：SD 事件日志（`YYMMDD_E.CSV`）的模块状态变化记录，依赖 `Storage/Src/storage_module_event.c` 的定义表 `s_module_defs`。落盘逻辑 `Storage_CheckModuleStatusEvents`（`Framework/Src/px4lite_storage.c`）本已遍历全部 14 个模块，但每个模块要先在 `StorageModuleEvent_FindDef` 里查到定义才会写；查不到即 `return IDLE` 静默跳过。

原定义表只登记了 7 个模块：GNSS / IMU / BARO / BATTERY / LORA / STORAGE / DISPLAY。以下**启用中的真实模块**因未登记，其状态变化从未写入事件日志：

- `PX4LITE_MODULE_REMOTE_ID`（`ENABLE_REMOTE_ID=1`，有真实在线/离线判定，屏显与 MAVLink 遥测均有该状态）
- `PX4LITE_MODULE_CONTROL`（`ENABLE_CONTROL=1`）
- `PX4LITE_MODULE_ALARM`（`ENABLE_ALARM=1`）

`PX4LITE_MODULE_5G` 同样未登记，但其 `ENABLE_5G=0` 为占位、无真实在线判定（见 [2026-07-13_05_5G通信显示与日志占位](2026-07-13_05_5G通信显示与日志占位.md)），本轮不补。

## 本次改动

- `Storage/Src/storage_module_event.c`：在 `s_module_defs` 中追加三条定义，按模块枚举顺序插入：
  - `REMOTE_ID` → 来源名 `REMOTEID`，消息 `remoteid_degraded/offline/failed/recovered`
  - `CONTROL` → 来源名 `CONTROL`，消息 `control_degraded/offline/failed/recovered`
  - `ALARM` → 来源名 `ALARM`，消息 `alarm_degraded/offline/failed/recovered`
- 来源名与 `Framework/Src/px4lite_mavlink_tx.c` 的 `MavTx_ModuleName` 一致（`REMOTEID`/`CONTROL`/`ALARM`），保证 SD 事件日志、MAVLink 遥测、屏显三处口径统一。
- 5G 位置保留一行注释占位，说明未登记原因与后续补入条件。

## 效果

- RemoteID / Control / Alarm 的状态跳变（DEGRADED / OFFLINE / FAILED ↔ 恢复）现会写入 `YYMMDD_E.CSV`，事件类型 `ALARM_ACTIVE` / `STATUS_RECOVERED`。
- 保持原有边沿去重（`StorageModuleEvent_Update` 中 `state->active` 比较）：持续离线只记一条，不刷屏；一直正常的模块不产生事件。
- `s_states[]` 按 `PX4LITE_MODULE_COUNT` 定长，`module_id < COUNT` 有边界保护，新增条目不越界。

## 边界说明

- 本次只补**事件日志（E.CSV）**。周期数据日志（`YYMMDD_D.CSV` / `storage_csv.h` 的 `Storage_CsvData_t`）仍无各模块在线状态列（通信类仅有 LoRa 帧计数），如需数据行也带模块状态另行改动。
- 5G 不在本次范围：待 5G 驱动/服务接入、有真实在线判定后，再把它补进 `s_module_defs`（及按需补数据行字段）。

## 验证

- 已完成源码修改与来源名一致性核对（对照 `MavTx_ModuleName` 与模块枚举 `px4lite_types.h`）。
- 本机无 Keil `UV4.exe` 命令入口，本轮未执行 Rebuild All，待在 Keil 中编译确认。
