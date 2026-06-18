# STM32F407 Formal Sensor Framework

This directory is an independent build. It does not modify or include the
original application source files.

Business-layer code is grouped under `Business/Inc` and `Business/Src`.

## Included baseline

- STM32F407ZG, HAL and FreeRTOS
- USART1 debug console
- ATGM336H on USART2
- DMA1 Stream5 Channel4 circular reception
- USART2 IDLE interrupt chain
- NMEA GGA/RMC/GSA/GSV parsing
- GPS and BeiDou visible/used satellite counts
- latest-value topics with protected copy access
- GNSS to navigation snapshot conversion
- module state, timeout and health snapshot
- non-blocking business event bus with per-subscriber delivery statistics
- business system and snapshot-distribution tasks
- application read-only API for coherent navigation and system snapshots
- enabled Display service task with a NOT_READY placeholder adapter
- FreeRTOS stack-overflow and malloc-failure hooks
- module registry and lifecycle sequencing
- fixed-period work items with deadline and execution-time statistics
- optional USART1 task stack high-water and heap reporting

## Runtime flow

```text
USART2 DMA/IDLE
      |
      v
BSP GNSS byte buffer
      |
      v
GNSS driver + NMEA parser
      |
      v
platform adapter
      |
      v
sensor task -> GNSS topic -> estimator task -> navigation topic
      |                              |                 |
      +---------- health task <------+                 v
                                                business snapshot task
                                                        |
                                                        v
                                                   event bus

system task --------------------------------------> status/log event
display task <------------------------------------ navigation/health copy
```

Six tasks are created:

| Task | Period | Responsibility |
|---|---:|---|
| `sensor` | 10 ms | service drivers and publish measurements |
| `estimator` | 20 ms | convert measurements to domain snapshots |
| `health` | 100 ms | timeout/state checks and 1 Hz debug report |
| `biz_system` | 1000 ms | one-shot startup log and business polling |
| `biz_acq` | 200 ms | copy published navigation and fan out events |
| `biz_display` | 10 ms | touch-first, budgeted Display service |

`biz_acq` never reads BSP or sensor drivers. Hardware acquisition remains
single-owner inside `sensor`. Business consumers use `app_data_api.h` rather
than reading Framework topics directly.

Display is enabled with `BUSINESS_ENABLE_DISPLAY=1U`. Until the real adapter
is implemented, `business_display_placeholder.c` reports `NOT_READY`, and the
Display module state is `OFFLINE`.

## GNSS states

| State | Meaning |
|---|---|
| `STARTING` | initialized and waiting for NMEA |
| `ONLINE` | NMEA is arriving and the position fix is valid |
| `DEGRADED` | NMEA is arriving but there is no valid fix |
| `OFFLINE` | no new valid NMEA for 2 seconds |
| `FAILED` | initialization failed |

The first 5 seconds are a startup grace period before an offline state is
reported.

## Build

Open:

`MDK-ARM/formal_framework.uvprojx`

The project uses ARMCC 5.06. HAL, CMSIS and FreeRTOS sources are included
under `Third_Party`, so this directory no longer references the parent
project's source tree. A successful build creates:

- `MDK-ARM/Objects/formal_framework.axf`
- `MDK-ARM/Objects/formal_framework.hex`

The startup debug output includes the remaining FreeRTOS heap after queues and
all six tasks are created. Stack overflow and allocation failure still enter
the existing FreeRTOS hooks.

## Debug switch

`Debug/Inc/debug_config.h` contains independent debug switches.

- `DEBUG_ENABLE`: enables the USART1 debug console
- `DEBUG_PERIODIC_ENABLE`: enables the one-second GNSS summary
- `DEBUG_TASK_STACK_ENABLE`: enables the five-second task stack and heap report
- undefined: debug calls compile out; the sensor pipeline keeps running

The Keil target is `STM32F407ZGTx`, matching STM32F407ZGT6. DMA buffers must
remain in the 128 KB SRAM region at `0x20000000`; the 64 KB CCM region at
`0x10000000` is not DMA-accessible.

Do not enable raw NMEA printing for endurance tests because continuous UART
formatting changes task timing.

## Adding another sensor

1. Put register/bus handling in `Bsp`.
2. Put chip parsing, calibration and device state in `Sensor`.
3. Add a HAL-free adapter in `Framework/Src/px4lite_platform_f407.c`.
4. Add a typed measurement in `px4lite_types.h`.
5. Add publish/copy access in `px4lite_topics.*`.
6. Schedule the driver only from `sensor`; do not create one task per sensor.
7. Add freshness and offline limits in `px4lite_config.h`.
8. Let domain/communication tasks copy snapshots; they must not consume BSP
   DMA buffers directly.

Large or high-rate data should use a FIFO or memory pool. Low-rate state data
should use the latest-value publish/copy pattern already used by GNSS.
