# Formal Framework 11

低空教学实验室 STM32F407 固件框架工程。

本仓库基于 **STM32F407 + HAL + FreeRTOS + Keil MDK-ARM v5**，用于沉淀低空教学实验室产品中的传感器采集、状态管理、业务快照、显示、通信、调试和存储框架。工程目标不是临时 Demo，而是可持续扩展、可教学讲解、可团队分工维护的嵌入式基础框架。

当前工程以 `MDK-ARM/formal_framework.uvprojx` 为唯一构建入口，HAL、CMSIS、FreeRTOS、FatFs、MAVLink 等依赖已经收拢在本仓库内。

## 当前定位

- MCU 平台：STM32F407ZGT6 / STM32F407ZGTx
- 开发栈：HAL + FreeRTOS + ARMCC 5.06
- 工程入口：Keil MDK-ARM v5
- 任务模型：固定周期任务 + 框架注册表 + 强类型 topic / snapshot
- 主要边界：硬件采集、设备驱动、框架调度、业务读取、显示刷新、LoRa 发送、日志显示、SD 存储分层隔离
- 完成度口径：以 `Development_Guide/07_移植进度与功能清单.md` 为准

## 目录结构

| 目录 | 说明 |
|---|---|
| `Core/` | `main.c`、HAL MSP、IRQ、FreeRTOS hook、系统启动代码 |
| `Bsp/` | 板级外设、GPIO、总线、DMA、中断、原始收发接口 |
| `Sensor/` | 设备协议、解析、校准、设备事实和驱动状态 |
| `Framework/` | 模块注册、topic、状态、恢复、调度、MAVLink 发送和平台适配 |
| `Business/` | 应用层任务、事件总线、业务快照读取和显示业务入口 |
| `Display/` | ATK-MD0700 / SSD1963 / GT911 / 页面渲染相关代码 |
| `Debug/` | 独立调试开关、任务监控、周期诊断、串口调试输出 |
| `Storage/` | SD 卡、FatFs、CSV 记录队列和存储服务 |
| `Third_Party/` | HAL、CMSIS、FreeRTOS、FatFs、MAVLink 等第三方代码 |
| `Development_Guide/` | 架构规范、模块接入、完成度清单、变更记录和模板 |
| `MDK-ARM/` | Keil 工程文件、调试配置和本地构建输出目录 |
| `Release/` | 已验证发布产物和发布说明 |

## 构建方式

使用 Keil MDK-ARM v5 打开：

```text
MDK-ARM/formal_framework.uvprojx
```

编译目标：

```text
Target 1
```

发布要求：

```text
0 Error(s), 0 Warning(s)
```

成功构建后，Keil 会在以下目录生成中间产物和固件：

```text
MDK-ARM/Objects/
MDK-ARM/Listings/
```

这些构建产物不进入 Git。需要对外发布时，按 `Release/README.md` 的规则更新 `Release/Latest/`，并保证 `hex`、`axf`、`map`、构建日志和 `MANIFEST.md` 来自同一次全量构建。

## 架构规则

工程采用单向依赖：

```text
Core -> Business -> Framework -> Platform Adapter -> Driver -> BSP -> HAL
```

关键约束：

- `main.c` 是组合根。
- `Framework/Src/px4lite_platform_f407.c` 是 Framework 层唯一允许包含 BSP 头文件的位置。
- Business 层只能通过 `app_data_api.h` 读取数据。
- Business 不得直接包含 BSP / Sensor 头文件，不得读取 DMA buffer 或驱动私有变量。
- Sensor Driver 只负责设备协议、解析、标定和硬件事实，不负责业务状态。
- OFFLINE / FAILED 状态由 Health task 统一判定，驱动只上报硬件事实和 ONLINE / DEGRADED。
- 通信层不得直接发送 C struct 内存，MAVLink 必须逐字段编码。
- ISR 只做计数、位置更新或任务通知，不做解析、打印、SD 写入或页面渲染。

## 任务模型

| Task | 周期 | 职责 |
|---|---:|---|
| `sensor` | 10 ms | GNSS、IMU、Baro、Battery 等普通传感器的唯一硬件采集者 |
| `estimator` | 20 ms | GNSS 到 Navigation domain，IMU FIFO 到姿态解算 |
| `health` | 100 ms | 超时检测、OFFLINE 判定、健康快照、看门狗喂狗门控 |
| `comm` | 10 ms | LoRa RX 和 MAVLink TX 调度 |
| `biz_system` | 1000 ms | 启动日志、注册表轮询、系统业务状态 |
| `biz_acq` | 200 ms | 导航快照复制和 EventBus 分发 |
| `biz_display` | 10 ms | 触摸优先、预算化 LCD 刷新 |
| `debug` | 100 ms | 周期诊断和任务监控，不纳入看门狗集合 |
| `storage` | 50 ms | SD CSV 日志记录，注册表登记，低优先级阻塞 I/O |

原则：**一个硬件资源只有一个任务所有者**。不要为每个传感器单独创建任务，普通传感器统一由 `sensor` 任务采集。

## 数据读取方式

应用消费者必须使用 `Business/Inc/app_data_api.h`：

```c
App_CopyNavigation(&nav, now_ms);
App_CopySystem(&sys, now_ms);
App_CopyEnvironment(&env, now_ms);
App_CopyAlarm(&alarm, now_ms);
App_GetModuleStatus(id, &status);
App_GetCommStats(&comm);
```

低频状态数据采用 latest-value snapshot 模式。高频或不能丢样的数据使用 FIFO 或固定内存池。临界区只允许复制快照和设置 ready 标志，不允许解析、计算或打印。

## 当前能力概览

当前真实完成度以 `Development_Guide/07_移植进度与功能清单.md` 为唯一来源。不要仅凭文件、任务或函数声明存在就判断模块已经完成。

| 模块 | 当前状态口径 |
|---|---|
| Core / HAL / FreeRTOS 启动 | 已形成独立工程基础 |
| Sensor 读取 | GNSS、IMU、气压计、电源等传感器读取链路已接入统一采集任务 |
| Framework topic / registry / health | 已形成模块生命周期、latest-value topic / FIFO、基础健康状态和恢复框架 |
| Business API / EventBus | 应用只读快照边界已建立，业务层通过统一 API 获取导航、环境、电源、状态和告警数据 |
| Display | 传感器数据显示、状态显示和业务刷新链路已完成 |
| LoRa / MAVLink | LoRa 遥测发送链路已完成，按低速链路预算发送 MAVLink 遥测消息 |
| Debug / Log 显示 | 调试日志、状态日志和运行信息显示链路已完成 |
| Storage | SD CSV 日志存储链路已完成，数据记录和错误记录通过存储服务落盘 |
| Alarm / Command | 保留接口和扩展位置，不能视为完整业务闭环 |

## 关键配置文件

| 文件 | 作用 |
|---|---|
| `Bsp/Inc/bsp_config.h` | 引脚、总线、DMA stream、外设开关 |
| `Framework/Inc/px4lite_config.h` | 模块使能、任务周期、栈、优先级、超时、MAVLink 发送周期 |
| `Debug/Inc/debug_config.h` | 独立调试开关，底部派生宏自动计算，不要手动改 |
| `Business/Inc/business_template_config.h` | 业务任务周期、栈、优先级、显示配置 |
| `Storage/Inc/storage_config.h` | SD 服务周期、队列深度、SPI 超时、CSV 文件名 |

## 开发入口

常用文档：

| 文档 | 用途 |
|---|---|
| `Development_Guide/01_强制开发规范.md` | 代码评审和合入门禁 |
| `Development_Guide/06_应用层数据接口.md` | Business 层读取数据的正式接口 |
| `Development_Guide/07_移植进度与功能清单.md` | 当前完成度、占位项、后续阶段和验收标准 |
| `Development_Guide/10_Debug模块化使用指南.md` | Debug 开关和诊断模块使用方式 |
| `Development_Guide/12_团队模块化开发手册.md` | 新成员和协作开发流程 |
| `Development_Guide/16_模块接入标准与评审清单.md` | 新模块接入模板和 16 项检查清单 |

新增模块时，应按以下顺序设计和落地：

1. 在 `Bsp` 中处理引脚、时钟、DMA、中断和原始收发。
2. 在 `Sensor` 中处理芯片协议、解析、校准和设备事实。
3. 在 `Framework/Src/px4lite_platform_f407.c` 中做 Driver 类型到 Framework 类型的转换。
4. 在 `Framework` 中建立强类型 topic、状态、超时和恢复入口。
5. 在 `Business` 中只通过 `app_data_api.h` 或 EventBus 消费快照。
6. 在 `Development_Guide/07_移植进度与功能清单.md` 和 `Change_History/` 中记录完成度、验证结果和未完成项。

## 版本库约定

本仓库跟踪：

- 源码
- Keil 工程文件
- 配置文件
- 规范文档
- 发布清单和发布说明

本仓库不跟踪：

- `MDK-ARM/Objects/`
- `MDK-ARM/Listings/`
- Keil build log
- `.uvguix.*` 本地界面状态
- `.claude/` 本地助手状态
- `Release/Latest/` 下的 `.axf`、`.hex`、`.map`、`.log` 二进制或构建产物

初始化仓库后，首个提交应包含源码、工程文件、配置、文档和 `.gitignore`。构建产物只在发布流程中按 `Release/README.md` 归档。

## 重要硬件和实时性规则

- DMA buffer 必须放在 `0x20000000` 起始的 128 KB SRAM 中。
- `0x10000000` 起始的 64 KB CCM 不支持 DMA。
- 周期任务使用 `vTaskDelayUntil()`，不要用 `vTaskDelay()` 代替。
- `_ms` 字段必须是真实毫秒值，来自 `PlatformGetMs()` 或 `BSP_Time_GetTickMs()`。
- `_ticks` 字段才表示 RTOS ticks，禁止混用。
- Sensor / Driver 层不得调用 `HAL_GetTick()`。
- 启动后不再动态分配内存，数据结构优先静态分配。
- `heap_4` 峰值使用率应保持在 75% 以下。
- 调试输出开关关闭后，不得改变生产数据路径。
