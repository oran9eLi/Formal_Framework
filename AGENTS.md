# AGENTS.md — Formal Framework 11

## Build

- **Keil MDK-ARM v5 + ARMCC 5.06** (`--c99`). Open `MDK-ARM/formal_framework.uvprojx`.
- No CLI / CMake / Makefile. No CI. Output: `MDK-ARM/Objects/formal_framework.axf` / `.hex`.
- Release requires `0 Error / 0 Warning` Rebuild All.

## Architecture

Strict one-way dependency:

```
Core → Business → Framework → Platform Adapter → Driver → BSP → HAL
```

`main.c` is the composition root. `px4lite_platform_f407.c` is the **only** framework file allowed to `#include` BSP headers.

## DMA rule

DMA buffers **must** live in 128 KB SRAM at `0x20000000`. The 64 KB CCM at `0x10000000` is **not** DMA-capable.

## Key config files

| File | Purpose |
|------|---------|
| `Bsp/Inc/bsp_config.h` | Pins, buses, DMA streams, peripheral enable switches |
| `Framework/Inc/px4lite_config.h` | Module enables, task periods, stack sizes, priorities, timeouts, MAVLink message enables/periods |
| `Debug/Inc/debug_config.h` | Independent debug monitor switches. **Derived macros at the bottom are auto-computed — never edit manually.** |
| `Business/Inc/business_template_config.h` | Business task periods, stacks, priorities, display config |
| `Storage/Inc/storage_config.h` | SD service periods, queue depth, SPI timeout, CSV file names (task stack/priority live in `px4lite_config.h`) |

## Layer boundaries

| Layer | May do | Must not |
|-------|--------|----------|
| Business | Read via `app_data_api.h` only | Include BSP/Sensor, read DMA buffers |
| Framework | Manage topics, state, lifecycles | Include Business headers |
| Platform Adapter | Convert Driver types → Framework types (sole owner) | — |
| Sensor Driver | Chip protocol, parse, device facts | `#include stm32f4xx_hal.h`, call `HAL_*`, declare OFFLINE/FAILED state |
| BSP | Pins, buses, DMA, raw tx/rx | Parse data, business logic |

## Tasks

| Task | Period | Priority | Ownership |
|------|--------|----------|-----------|
| `sensor` | 10ms | idle+4 | Sole hardware collector for GNSS, IMU, Baro, Battery |
| `estimator` | 20ms | idle+3 | GNSS→Navigation domain + IMU FIFO→Madgwick attitude |
| `health` | 100ms | idle+2 | Timeout→OFFLINE, health snapshot, gated watchdog feed |
| `comm` | 10ms | idle+2 | LoRa RX, MAVLink TX scheduling (8 message types) |
| `biz_system` | 1000ms | idle+2 | Startup log, registry polling |
| `biz_acq` | 200ms | idle+2 | Nav snapshot → EventBus fan-out |
| `biz_display` | 10ms | idle+1 | Touch-first, budgeted LCD refresh (2ms step budget) |
| `debug` | 100ms | idle+1 | Period diagnostics, **not in watchdog set** |
| `storage` | 50ms | idle | Registry-registered (init/recover); SD CSV logging via FatFs/SPI3; reads Framework topics; blocking SD I/O so it sits below `biz_display`; **not in watchdog set** |

**One hardware resource = one task owner.** Do not create one task per sensor; all normal sensors go through `sensor`. The `storage` module is registered in the framework registry like any sensor (descriptor with `init`/`recover`); its `recover` only sets a remount-request flag.

## Core rules

- **Time**: every `_ms` field must be a real millisecond value from `PlatformGetMs()` or `BSP_Time_GetTickMs()`. `_ticks` = RTOS ticks. Never mix them. `HAL_GetTick()` must not appear in Sensor/Driver code.
- **Period tasks**: `vTaskDelayUntil()`, not `vTaskDelay()`.
- **No malloc after startup**. All data structures are static. `heap_4`, peak usage must stay <75%.
- **ISR**: only update counters / write positions / send task notifications. No `printf`, no parsing, no SD write, no page rendering.
- **`__enable_irq()`**: always save `__get_PRIMASK()` first, restore only if it was 0. Never unconditional.
- **Send semantics**: `Lora_E22_Send()` = async copy — `OK` means frame was staged, not that it left the air. TX completion = `HAL_UART_TxCpltCallback` (DMA TC interrupt).
- **Never send raw C struct memory** over comms. MAVLink uses field-by-field encoding.
- **State single-writer** (spec 13.5): Drivers only record hardware facts and promote ONLINE/DEGRADED. OFFLINE/FAILED is owned **exclusively** by the Health task.

## Snapshot publish pattern

```c
// Writer (in sensor task):
taskENTER_CRITICAL();
g_topic = *source;
g_topic_ready = 1U;
taskEXIT_CRITICAL();

// Reader (in estimator / comm / business):
taskENTER_CRITICAL();
if (g_topic_ready) { *dest = g_topic; }
taskEXIT_CRITICAL();
```

No parsing, computation, or printf inside critical sections.

## Recovery mechanism

- `Px4Lite_RecoveryMonitorRun()` iterates all registered modules with a `recover` callback.
- Rate-limited: 2s backoff, unlimited retries (hot-plug tolerant).
- `recover` callbacks **only set a request flag** — no bus I/O on the Health task. The owning service task performs the actual re-init.
- IMU has **driver-level auto-reinit**: 5 consecutive read failures → auto-schedules `Init()` with adaptive backoff (100ms fast retry ×5, then 500ms).
- IMU stable-validation gate: after reinit, the Platform Adapter requires ≥3 consecutive valid frames before publishing to Framework.

## Debug

- Independent per-module switches, no master toggle. Derived macros at bottom of `debug_config.h` are computed from the per-module switches — **do not edit them**.
- `DebugTask` runs at idle+1 (lowest business priority), excluded from the watchdog heartbeat set.
- Do **not** enable raw NMEA printing during endurance tests — UART formatting changes task timing.
- Closing debug switches must not alter the production data path.

## Business layer

Application consumers **must** read data through `app_data_api.h`:

```c
App_CopyNavigation(&nav, now_ms);
App_CopySystem(&sys, now_ms);
App_CopyEnvironment(&env, now_ms);
App_CopyAlarm(&alarm, now_ms);
App_GetModuleStatus(id, &status);
App_GetCommStats(&comm);
```

Never read Framework topics, BSP DMA buffers, or driver private variables directly.

## Module status labels

`Development_Guide/07_移植进度与功能清单.md` is the **single source of truth**. Labels: `完成` > `基础完成` > `骨架完成` > `占位` > `预留`. Do not treat a module as working just because its files, tasks, or function declarations exist.

## Key doc references

| Doc | Content |
|-----|---------|
| `Development_Guide/01_强制开发规范.md` | Code-review gates |
| `Development_Guide/07_移植进度与功能清单.md` | Module completion status |
| `Development_Guide/10_Debug模块化使用指南.md` | Debug switch system |
| `Development_Guide/12_团队模块化开发手册.md` | New module integration process |
| `Development_Guide/16_模块接入标准与评审清单.md` | GNSS/LoRa templates + 16-item checklist |
