# 2026-07-02 LoRa GetState 状态语义修复

## 背景

排查 LoRa 连接状态时发现 `Lora_E22_GetState()` 丢弃 `offline_timeout_ms`，并且在本机 E22 硬件可用、RX DMA 可用的正常路径中仍返回 `LORA_STATE_OFFLINE`，导致 Framework 无法区分“本机 LoRa 模块可用但暂无对端帧”和“本机 LoRa 硬件不可用”。

## 改动

- `Driver/Comm/Src/lora_e22.c` 恢复 `offline_timeout_ms` 参与判定。
- `s_initialized != 0` 且 `Lora_E22_LocalUnavailable() == 0` 时：
  - 若从未收到完整 MAVLink 帧，返回 `LORA_STATE_ONLINE`，表示本机硬件链路可用；
  - 若曾收到帧且 `now_ms - s_last_rx_ms > offline_timeout_ms`，返回 `LORA_STATE_OFFLINE`；
  - 否则返回 `LORA_STATE_ONLINE`。
- `Framework/Src/px4lite_modules.c` 增加 `state == PX4LITE_STATE_ONLINE && peer_recent == 0` 分支，显示为 `PX4LITE_STATE_DEGRADED`，保持教学需求：本机 LoRa 在位但没收到对端帧为黄灯，收到对端帧为绿灯。
- `Driver/Comm/Inc/lora_e22.h` 同步修正状态枚举和 `offline_timeout_ms` 注释。

## 栈和实时性

- 未新增 FreeRTOS task。
- 未扩大任何任务栈。
- 只调整状态判定分支和注释，不增加 comm 栈上的大对象。

## 验证

- 2026-07-02 已执行 Keil Rebuild All。
- 工程：`MDK-ARM/formal_framework.uvprojx`
- 日志：`MDK-ARM/Objects/formal_framework.build_log.htm`
- Program Size: `Code=270168 RO-data=91776 RW-data=1388 ZI-data=100852`
- 结果：`0 Error(s), 0 Warning(s)`
