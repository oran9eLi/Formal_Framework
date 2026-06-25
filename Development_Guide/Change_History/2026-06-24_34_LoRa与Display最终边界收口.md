# 2026-06-24 34 LoRa 与 Display 最终边界收口

## 背景

后续开发重点将进入 LoRa 收发和外设扩展。为避免后续反复返工，本次将通信 Driver 目录、Display 应用视图和周期任务栈约束一次性收口。

## 变更内容

- 将 LoRa E22 从 `Sensor/Inc` / `Sensor/Src` 迁移到 `Driver/Comm/Inc` / `Driver/Comm/Src`。
- 更新 Keil include path 和 source list，工程组名由 `GNSS Driver` 收敛为 `Device Driver`。
- 新增 `App_DisplaySnapshot_t`、`App_ViewState_t` 和 `App_CopyDisplaySnapshot()`，Display 只消费 App 层显示视图，不再直接读取 Framework 模块枚举、故障枚举或 topic 有效位。
- 新增 `App_CommandMotorThrottlePercent()`，Display 设置电机油门时不再直接判断 `PX4LITE_OK`。
- `Display/Src/display.c` 改为使用文件级静态 `s_display_snapshot` 和 `s_display_selfcheck_faults`，避免在 `biz_display` 栈上放大聚合视图或告警数组。
- `Framework/Src/px4lite_alarm.c` 将告警更新中的 old/new snapshot 改为文件级静态 scratch，避免 Health task 栈上放两份完整告警表。
- 新增 `Px4Lite_CopyAlarmSummary()` 和 `Px4Lite_CopyAlarmRecord()`，App、Debug 和 MAVLink STATUSTEXT 使用轻量摘要或单条记录复制，避免周期任务复制完整告警表。
- `Framework/Src/px4lite_mavlink_tx.c` 的 STATUSTEXT 外发改为读取告警摘要，不在 `comm` 栈上放完整告警表。
- `Debug/Src/debug_service.c` 告警监控改为 `App_CopyAlarmSummary()`，不在 Debug task 栈上放完整告警表。
- 更新 `01_统一开发手册.md`、`02_架构边界与数据流.md`、`03_模块接入手册.md` 和 `05_完成度与后续清单.md`，明确通信 Driver 目录和周期任务栈约束。

## 栈边界

- `biz_display`：不在栈上创建 `App_DisplaySnapshot_t`、完整系统快照或告警表；使用静态显示视图。
- `comm`：不在栈上创建完整告警表或全量外设大包；LoRa/MAVLink 外发按 Telemetry Catalog 分项调度，告警文本只读 summary。
- `health`：告警 old/new snapshot 使用静态 scratch，不占用 Health task 栈。
- `debug`：告警监控只读 summary，不占用 Debug task 栈。

## 验证

- 已执行 Keil Rebuild All：`MDK-ARM/formal_framework.uvprojx` / `Target 1`。
- 构建工具：ARMCC 5.06 update 7 build 960。
- 构建结果：`formal_framework.axf - 0 Error(s), 0 Warning(s)`。
- 程序大小：`Code=110056 RO-data=380324 RW-data=700 ZI-data=54940`。
- 静态扫描确认：
  - 旧 LoRa 路径 `Sensor/Inc/lora_e22.h`、`Sensor/Src/lora_e22.c` 无工程或源码残留引用。
  - `Display/` 不再直接引用 `PX4LITE_` / `Px4Lite_`、`App_GetModuleStatus()` 或完整 App 快照接口。
  - 完整系统快照、完整告警表和 Display 聚合视图只保留文件级静态 scratch / snapshot，未发现函数局部大对象。
