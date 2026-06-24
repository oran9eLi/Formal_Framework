# AGENTS.md - Formal Framework

本文件是本仓库内所有自动化 agent、代码助手和协作者必须遵守的开发规范。处理本工程时，应优先遵守本文件，其次参考 `README.md` 和 `Development_Guide/` 中的长期规范。

## Agent 必须遵守的工作方式

- 默认项目上下文为 STM32F407 + HAL + FreeRTOS + Keil MDK-ARM。
- 交付代码必须达到工程级质量，不能写成临时 Demo 风格。
- 修改前先阅读真实代码、工程配置和相关文档，不得只凭文件名或猜测修改。
- 不得破坏既有分层边界，不得为了省事绕过 `app_data_api.h`、平台适配层或 BSP 边界。
- 外设相关开发必须保持初始化链路完整，包括 HAL config、Keil 工程 source list、`MX_xxx_Init` 风格入口、MSP GPIO/clock/DMA/NVIC 配置、interrupt handlers，以及必要的 FreeRTOS 集成。
- 新增或调整模块时，应预留或实现 startup self-check、error-handling、status-reporting、alarm 和 recovery 入口。
- protocol parsing、data conversion、display、logging 必须分层隔离，不得把解析逻辑散落在业务代码中。
- 小规模嵌入式数据存储优先采用 filesystem-based storage，除非有明确理由才引入数据库。
- 除非有明确技术或产品理由，不得引入 QT 或额外高成本硬件等偏重、偏贵的 UI/runtime 方案。
- 修改 STM32 固件源码、Keil 工程或外设配置后，必须运行 Keil build verification，并报告实际结果；目标为 `0 Error(s), 0 Warning(s)`。
- 文档或仓库规则修改不需要强制跑 Keil build，但必须说明未跑 build 的原因。
- `Development_Guide/` 是统一开发手册；代码框架、任务模型、接口边界、配置入口或外设接入方式发生变化时，必须同步修改当前编号手册。

## 构建规则

- 使用 Keil MDK-ARM v5 + ARMCC 5.06，C 标准为 `--c99`。
- 唯一工程入口为 `MDK-ARM/formal_framework.uvprojx`。
- 本仓库没有 CLI / CMake / Makefile / CI 构建入口。
- 输出目录为 `MDK-ARM/Objects/` 和 `MDK-ARM/Listings/`。
- 发布构建必须 Rebuild All，并达到 `0 Error(s), 0 Warning(s)`。
- 构建产物不进入 Git；正式发布产物按 `Release/README.md` 归档。

## 架构边界

工程采用严格单向依赖：

```text
Core -> Business -> Framework -> Platform Adapter -> Driver -> BSP -> HAL
```

- `main.c` 是组合根。
- `Framework/Src/px4lite_platform_f407.c` 是 Framework 层唯一允许 `#include` BSP 头文件的位置。
- Business 层只能通过 `Business/Inc/app_data_api.h` 读取数据。
- Business 不得包含 BSP / Sensor 头文件，不得读取 DMA buffer 或驱动私有变量。
- Sensor Driver 只负责芯片协议、解析、校准、设备事实和驱动内部状态。
- BSP 只负责引脚、总线、DMA、中断和原始收发，不得做协议解析或业务逻辑。

## DMA 规则

- DMA buffer 必须放在 `0x20000000` 起始的 128 KB SRAM 中。
- `0x10000000` 起始的 64 KB CCM 不支持 DMA，不能放 DMA buffer。

## 关键配置文件

| 文件 | 作用 |
|---|---|
| `Bsp/Inc/bsp_config.h` | 引脚、总线、DMA stream、外设使能开关 |
| `Framework/Inc/px4lite_config.h` | 模块使能、任务周期、栈大小、优先级、超时、MAVLink 消息使能和周期 |
| `Debug/Inc/debug_config.h` | 独立调试开关；底部派生宏自动计算，不得手动编辑 |
| `Business/Inc/business_template_config.h` | 业务任务周期、栈、优先级、显示配置 |
| `Storage/Inc/storage_config.h` | SD 服务周期、队列深度、SPI 超时、CSV 文件名 |

## 分层职责

| 层级 | 可以做 | 禁止做 |
|---|---|---|
| Business | 通过 `app_data_api.h` 读取快照，处理业务流程和显示业务入口 | 包含 BSP/Sensor 头文件，读取 DMA buffer 或驱动私有变量 |
| Framework | 管理 topic、registry、状态、生命周期、恢复和调度 | 包含 Business 头文件 |
| Platform Adapter | 负责 Driver 类型到 Framework 类型的转换 | 把 BSP 细节扩散到 Framework 其他文件 |
| Sensor Driver | 芯片协议、解析、校准、设备事实、ONLINE/DEGRADED 事实记录 | 调用 `HAL_*`，声明 OFFLINE/FAILED 最终状态 |
| BSP | 引脚、时钟、总线、DMA、中断、原始 tx/rx | 协议解析、业务逻辑、状态策略 |

## 任务模型

| Task | 周期 | 优先级 | 归属和职责 |
|---|---:|---|---|
| `sensor` | 10 ms | idle+4 | GNSS、IMU、Baro、Battery 等普通传感器的唯一硬件采集者 |
| `estimator` | 20 ms | idle+3 | GNSS 到 Navigation domain，IMU FIFO 到 Madgwick 姿态解算 |
| `health` | 100 ms | idle+2 | 超时检测、OFFLINE 判定、健康快照、看门狗喂狗门控 |
| `comm` | 10 ms | idle+2 | LoRa RX，MAVLink TX 调度 |
| `biz_system` | 1000 ms | idle+2 | 启动日志、注册表轮询、系统业务状态 |
| `biz_acq` | 200 ms | idle+2 | Navigation 快照复制和 EventBus 分发 |
| `biz_display` | 10 ms | idle+1 | 触摸优先，预算化 LCD 刷新，单步预算 2 ms |
| `debug` | 100 ms | idle+1 | 周期诊断和任务监控，不纳入 watchdog 集合 |
| `storage` | 50 ms | idle | 注册式 SD CSV 日志服务，读取 Framework topic，低优先级阻塞 I/O，不纳入 watchdog 集合 |

一个硬件资源只能有一个任务所有者。不得为每个普通传感器单独创建任务，普通传感器统一由 `sensor` 任务采集。

## 核心实时性规则

- 所有 `_ms` 字段必须是真实毫秒值，来源为 `PlatformGetMs()` 或 `BSP_Time_GetTickMs()`。
- `_ticks` 字段才表示 RTOS tick，禁止和毫秒值混用。
- Sensor / Driver 层不得调用 `HAL_GetTick()`。
- 周期任务必须使用 `vTaskDelayUntil()`，不得用 `vTaskDelay()` 代替。
- 启动后不得再动态分配内存，数据结构优先静态分配。
- `heap_4` 峰值使用率应保持在 75% 以下。
- ISR 只允许更新计数、写位置或发送任务通知；不得 `printf`、解析协议、写 SD、渲染页面。
- 使用 `__enable_irq()` 前必须保存 `__get_PRIMASK()`，只有原本允许中断时才恢复，禁止无条件开中断。
- `Lora_E22_Send()` 是异步 copy 语义，返回 `OK` 只表示帧已入发送流程，不代表已经发到空中。
- LoRa TX 真正完成以 `HAL_UART_TxCpltCallback` 的 DMA TC 中断为准。
- 通信层禁止直接发送 C struct 内存，MAVLink 必须逐字段编码。
- 状态单写者规则：驱动只记录硬件事实并提升 ONLINE/DEGRADED，OFFLINE/FAILED 由 Health task 独占判定。

## Snapshot 发布模式

```c
/* Writer: sensor task */
taskENTER_CRITICAL();
g_topic = *source;
g_topic_ready = 1U;
taskEXIT_CRITICAL();

/* Reader: estimator / comm / business */
taskENTER_CRITICAL();
if (g_topic_ready)
{
    *dest = g_topic;
}
taskEXIT_CRITICAL();
```

临界区内只允许复制快照和设置标志，不得解析、计算、打印或执行阻塞 I/O。

## 恢复机制

- `Px4Lite_RecoveryMonitorRun()` 遍历所有带 `recover` 回调的注册模块。
- 恢复限速为 2 s backoff，允许无限重试，支持热插拔。
- `recover` 回调只能设置请求标志，不得在 Health task 中执行总线 I/O。
- 实际重新初始化必须由资源所属 service task 执行。
- IMU 保留 driver-level auto-reinit：连续 5 次读取失败后调度 `Init()`，前 5 次 100 ms 快速重试，之后 500 ms backoff。
- IMU 重初始化后，Platform Adapter 至少等待 3 帧连续有效数据后才允许发布到 Framework。

## Debug 规则

- Debug 使用独立的 per-module switch，不设置总开关。
- `debug_config.h` 底部派生宏由 per-module switch 自动计算，不得手动编辑。
- `DebugTask` 运行在 idle+1，是最低业务优先级，不纳入 watchdog heartbeat。
- 长稳测试时不得开启 raw NMEA 打印，避免串口格式化改变任务时序。
- 关闭 Debug 开关不得改变生产数据路径。

## Business 数据读取规则

应用消费者必须通过 `app_data_api.h` 读取数据：

```c
App_CopyNavigation(&nav, now_ms);
App_CopySystem(&sys, now_ms);
App_CopyEnvironment(&env, now_ms);
App_CopyAlarm(&alarm, now_ms);
App_GetModuleStatus(id, &status);
App_GetCommStats(&comm);
```

不得直接读取 Framework topic、BSP DMA buffer 或 driver private variable。

## 注释规范

- 新增或重构源码必须使用 Doxygen 风格中文注释。
- 每个 `.c` / `.h` 文件开头应有文件头注释，说明文件职责、所属层级、主要依赖和禁止事项。
- 每个公开函数和重要内部函数的定义处应包含函数注释。
- 每个公开数据结构、配置结构、消息结构和跨层传递结构应包含结构体注释。
- 结构体成员使用中文注释说明含义；物理量、时间、长度、计数和状态值必须写清单位或取值范围。
- 结构体成员推荐使用 Doxygen 后置注释格式：`/**< 字段说明。 */`。
- 枚举类型应说明用途；枚举值较多或含义不直观时，应逐项注释。
- 注释应说明模块职责、数据含义、调用边界、实时性约束和设计原因，不得只重复代码。
- 修改函数行为、参数含义、结构体字段或模块边界时，必须同步更新对应注释。
- 禁止把过期设计、未实现能力或猜测性描述写成已完成事实。

## 模块完成度口径

- `Development_Guide/05_完成度与后续清单.md` 是模块完成度的唯一事实来源。
- 状态标签优先级为：`完成` > `基础完成` > `骨架完成` > `占位` > `预留`。
- 不得因为文件、任务、函数声明或空实现存在，就把模块视为完成。
- 修改模块完成度时，必须同步记录验证结果、未完成项和后续入口。

## 提交和文档规则

- 提交信息统一使用 `<type>: <简短中文说明>`。
- `type` 使用小写英文，冒号后保留一个空格。
- 常用类型：`feat`、`fix`、`docs`、`refactor`、`chore`、`build`、`test`。
- 示例：`docs: 补充注释规范`。
- 架构、协议、日志、存储、显示、任务边界或外设接入方式发生变化时，必须同步更新 `Development_Guide/` 当前编号手册和 `Change_History/`。
- 文档应描述已验证事实，不得把计划项写成已完成能力。

## 关键文档入口

| 文档 | 内容 |
|---|---|
| `Development_Guide/01_统一开发手册.md` | 项目基线、规则优先级、分层原则和文档同步要求 |
| `Development_Guide/02_架构边界与数据流.md` | 架构边界、数据读取、状态和恢复边界 |
| `Development_Guide/03_模块接入手册.md` | 新外设分类、初始化链路、接入样板和评审清单 |
| `Development_Guide/04_调试验证与发布门禁.md` | Debug、构建验证、必测项目和发布门禁 |
| `Development_Guide/05_完成度与后续清单.md` | 模块完成度事实来源和后续清单 |
