# 2026-06-26 Fault 与 Storage 健壮性收口

## 1. 背景

本次处理 4 个低危/提示项：Fault Handler 无现场、远端 App API 注释检查、Storage 状态复制无临界区、FreeRTOS stack overflow 缺少任务名现场。

## 2. 采纳结果

1. Cortex-M `HardFault/MemManage/BusFault/UsageFault` 改为汇编入口，按 EXC_RETURN 选择 MSP/PSP，并保存 fault snapshot。
2. `g_fault_crash_snapshot` 记录 fault 类型、EXC_RETURN、MSP、PSP、active SP、stacked R0-R3/R12/LR/PC/xPSR、CFSR、HFSR、DFSR、AFSR、BFAR、MMFAR、ICSR 和 SHCSR。
3. `Storage_SD_CopyStatus()` 改为临界区复制；Storage 状态短写入封装为短临界区，FatFs/SPI 阻塞 I/O 保持在临界区外。
4. FreeRTOS stack overflow hook 保存 task handle、task name 指针和固定长度 task name 副本，不在异常现场打印。
5. `App_SetRemoteViewEnabled()` 等远端 App API 在当前代码中已具备多行 Doxygen 注释，本次确认无需重复修改。

## 3. 边界要求

1. Fault Handler 内禁止打印、SD 写入、总线访问、协议发送、页面渲染和恢复动作。
2. Storage 状态临界区只允许复制或更新短状态字段，不得包裹 FatFs、SPI、文件 close/open/write/sync。
3. FreeRTOS fatal hook 只保存现场并进入 `Error_Handler()`，不得依赖 DebugConsole。

## 4. 验证

Keil Rebuild All 已通过：

```text
Program Size: Code=243052 RO-data=80412 RW-data=1064 ZI-data=113736
".\Objects\formal_framework.axf" - 0 Error(s), 0 Warning(s).
```
