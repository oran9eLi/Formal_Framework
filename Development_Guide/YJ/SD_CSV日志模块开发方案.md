# SD CSV 日志模块开发方案

## 1. 目标

在当前 Formal_Framework1 工程中新增基于 SPI SD 卡的 CSV 日志模块，实现正常数据和错误信息分离存储。

目标结果：

```text
/SENSOR_DATA.CSV
/SYSTEM_ERROR.CSV
```

其中：

- `SENSOR_DATA.CSV`：记录 GNSS、IMU、BME280、ADC 电源等正常采集数据。
- `SYSTEM_ERROR.CSV`：记录传感器异常、存储异常、队列溢出、低电压、模块状态变化等错误事件。

设计重点不是只把文件写出来，而是保证：

1. SD 卡未插入、接触不良、写入失败时，主程序不能卡死。
2. SD 写入不能影响 Sensor、Estimator、Health 等实时任务。
3. 新增任务不能造成栈溢出。
4. CSV 文件拔卡后可以在电脑上直接读取。
5. 数据来源必须来自当前 Formal_Framework1 强类型数据链路，不能使用弱类型 `device_id + channel + value` 通道。

## 2. 硬件连接

SD 卡模块使用 SPI1。

| SD 模块信号 | STM32F407 引脚 | 复用功能 |
|---|---|---|
| SCK | PB3 | SPI1_SCK |
| MISO | PB4 | SPI1_MISO |
| MOSI | PB5 | SPI1_MOSI |
| CS | PA15 | GPIO 输出 |

注意事项：

- PB3、PB4、PA15 与 JTAG 功能复用，调试器必须使用 SWD 模式。
- SWD 使用 PA13、PA14，不影响上述 SPI1 引脚。
- PA15 必须作为普通 GPIO 片选脚，由 SD 驱动显式拉低和拉高。
- SPI 初始阶段使用低速，初始化完成后再提高时钟。

## 3. 当前工程基础

当前工程已有条件：

- `HAL_SPI_MODULE_ENABLED` 已打开。
- `PX4LITE_MODULE_STORAGE` 已在模块枚举中预留。
- `PX4LITE_ENABLE_STORAGE` 当前为 `0U`。
- `DEBUG_STORAGE_MONITOR_ENABLE` 当前为 `0U`。
- 已有 Debug 栈监控机制，可注册新增 Storage 任务。
- 已有 GNSS、IMU、Baro、Battery 强类型数据链路。

当前缺失内容：

- `bsp_spi` SPI BSP 封装。
- FatFS 文件系统源码和配置。
- SD SPI `diskio` 适配层。
- Storage 独立任务。
- CSV 数据记录和错误记录队列。
- Storage Debug 状态打印。

## 4. 总体架构

采用独立低优先级 Storage 任务，不允许其他任务直接访问 FatFS。

```text
Sensor Task
  -> GNSS / IMU / BARO / BATTERY Topic
  -> Estimator Task
  -> Navigation Topic
  -> App Data API
  -> Storage CSV Producer
  -> Storage Record Queue
  -> Storage Task
  -> storage_csv
  -> FatFS
  -> diskio_sd_spi
  -> bsp_spi
  -> HAL SPI1
  -> SD Card
```

关键约束：

- Sensor、Estimator、Health、Debug 任务都不直接调用 `f_write()`。
- 所有 SD 文件操作只允许在 Storage 任务中执行。
- Storage 任务低优先级运行，不能抢占传感器采集实时性。
- 数据生产者只投递固定长度记录，队列满时丢弃并计数，不阻塞。

## 5. 模块分层

### 5.1 BSP 层

新增文件：

```text
Bsp/Inc/bsp_spi.h
Bsp/Src/bsp_spi.c
```

职责：

- 初始化 SPI1。
- 配置 PB3/PB4/PB5 为 SPI1 AF5。
- 配置 PA15 为 GPIO 输出，默认拉高。
- 提供带超时的 SPI 收发接口。
- 提供 CS 控制接口。

建议接口：

```c
BSP_Status_t BSP_SPI1_Init(void);
BSP_Status_t BSP_SPI1_SetSpeedLow(void);
BSP_Status_t BSP_SPI1_SetSpeedHigh(void);
BSP_Status_t BSP_SPI1_TransmitReceive(
    const uint8_t *tx,
    uint8_t *rx,
    uint16_t length,
    uint32_t timeout_ms);
void BSP_SPI1_SD_Select(void);
void BSP_SPI1_SD_Deselect(void);
```

约束：

- 不得在 BSP 层解析 FAT、CSV 或业务数据。
- 不得无限等待 SPI 状态。
- 所有收发必须有超时。

### 5.2 FatFS / diskio 层

新增或移植：

```text
Third_Party/FatFs/ff.c
Third_Party/FatFs/ff.h
Third_Party/FatFs/ffconf.h
Third_Party/FatFs/diskio.h
Storage/Src/diskio_sd_spi.c
Storage/Inc/diskio_sd_spi.h
```

职责：

- 实现 FatFS 需要的 `disk_initialize()`、`disk_status()`、`disk_read()`、`disk_write()`、`disk_ioctl()`。
- 将 FatFS 扇区读写映射到 SD SPI 协议。
- 限制 SD 初始化和读写等待次数。

约束：

- 不得在 `diskio` 中打印大量 Debug。
- 不得无限等待 SD 卡响应。
- SD 卡无响应时返回错误，由 Storage 状态机处理。

### 5.3 Storage 驱动层

新增文件：

```text
Storage/Inc/storage_sd.h
Storage/Src/storage_sd.c
```

职责：

- 管理 SD 卡挂载、卸载、文件打开、同步和错误状态。
- 封装 FatFS，向上层提供简单结果码。

建议接口：

```c
Px4Lite_Result_t Storage_SD_Init(void);
Px4Lite_Result_t Storage_SD_Service(uint32_t now_ms);
Px4Lite_Result_t Storage_SD_WriteDataLine(const char *line);
Px4Lite_Result_t Storage_SD_WriteErrorLine(const char *line);
Px4Lite_Result_t Storage_SD_Sync(uint32_t now_ms);
Px4Lite_Result_t Storage_SD_GetStatus(Storage_Status_t *out);
```

状态建议：

```text
UNINIT
NO_CARD
MOUNTING
READY
DEGRADED
FAILED
```

### 5.4 CSV 格式层

新增文件：

```text
Storage/Inc/storage_csv.h
Storage/Src/storage_csv.c
```

职责：

- 定义 CSV 文件名、表头和行格式化。
- 把强类型快照转成 CSV 行。
- 把错误事件转成 CSV 行。

文件名：

```text
/SENSOR_DATA.CSV
/SYSTEM_ERROR.CSV
```

数据日志表头：

```csv
time_ms,gnss_valid,lat_e7,lon_e7,roll_deg,pitch_deg,temp_c,pressure_hpa,humidity_pct,voltage_v,battery_pct
```

错误日志表头：

```csv
time_ms,module,state,fault,error_count,message
```

约束：

- CSV 行最大长度固定，例如 `192` 字节。
- CSV 行缓冲不得作为大局部数组放在任务栈中。
- 格式化必须检查输出长度，超长时截断并记录错误。

### 5.5 Storage 任务层

新增文件：

```text
Storage/Inc/storage_task.h
Storage/Src/storage_task.c
Storage/Inc/storage_config.h
```

职责：

- 创建 Storage 任务。
- 创建固定长度记录队列。
- 周期性生成数据日志记录。
- 消费记录队列并写入 SD 卡。
- 周期性 `f_sync()`。
- 维护写入统计和错误统计。

建议配置：

```c
#define STORAGE_TASK_STACK_WORDS       768U
#define STORAGE_TASK_PRIORITY          (tskIDLE_PRIORITY + 1U)
#define STORAGE_SERVICE_PERIOD_MS       50U
#define STORAGE_DATA_PERIOD_MS        1000U
#define STORAGE_SYNC_PERIOD_MS        5000U
#define STORAGE_QUEUE_LENGTH             8U
#define STORAGE_CSV_LINE_MAX           192U
#define STORAGE_MOUNT_RETRY_MS        2000U
#define STORAGE_SPI_TIMEOUT_MS          50U
```

记录类型：

```c
typedef enum
{
    STORAGE_RECORD_DATA = 0,
    STORAGE_RECORD_ERROR = 1
} Storage_RecordType_t;

typedef struct
{
    Storage_RecordType_t type;
    uint32_t enqueue_time_ms;
    char line[STORAGE_CSV_LINE_MAX];
} Storage_Record_t;
```

队列满策略：

- 数据日志队列满：丢弃当前数据行，`drop_count++`。
- 错误日志队列满：优先保留错误，可丢弃最旧的数据记录；第一版若不做优先队列，则直接丢弃并计数。
- 不允许生产者永久阻塞等待队列空间。

## 6. 配置接入

### 6.1 Framework 配置

修改：

```text
Framework/Inc/px4lite_config.h
```

新增或打开：

```c
#define PX4LITE_ENABLE_STORAGE              1U
#define PX4LITE_STORAGE_STARTUP_GRACE_MS 5000U
#define PX4LITE_STORAGE_OFFLINE_MS       3000U
#define PX4LITE_STORAGE_MAX_AGE_MS       5000U
```

第一阶段开发时可保持：

```c
#define PX4LITE_ENABLE_STORAGE              0U
```

等 `bsp_spi + FatFS + storage_task` 编译和单元测试通过后再打开。

### 6.2 Debug 配置

修改：

```text
Debug/Inc/debug_config.h
```

新增：

```c
#define DEBUG_STORAGE_MONITOR_ENABLE        1U
#define DEBUG_STORAGE_REPORT_PERIOD_MS   5000U
```

串口建议打印：

```text
[DBG] STORAGE: state=2 mounted=1 written=120 err=0 drop=0 q=0 sync=1 age=32
```

字段含义：

| 字段 | 含义 |
|---|---|
| state | Storage 模块状态 |
| mounted | FatFS 是否挂载成功 |
| written | 成功写入记录数 |
| err | 写入、挂载、同步失败次数 |
| drop | 队列满或格式化失败导致的丢弃数 |
| q | 当前队列积压数量 |
| sync | 最近一次同步是否成功 |
| age | 最近一次成功写入距当前时间，单位 ms |

## 7. 数据来源

`SENSOR_DATA.CSV` 的数据必须来自正式强类型接口：

| CSV 字段 | 数据来源 |
|---|---|
| time_ms | 当前系统毫秒 |
| gnss_valid | `Px4Lite_CopyGnss()` 结果和 `valid` |
| lat_e7 | `Px4Lite_SensorGnss_t.latitude_e7` |
| lon_e7 | `Px4Lite_SensorGnss_t.longitude_e7` |
| roll_deg | `Px4Lite_CopyNavigation()` 中 `roll_deg100 / 100.0` |
| pitch_deg | `Px4Lite_CopyNavigation()` 中 `pitch_deg100 / 100.0` |
| temp_c | `App_CopyEnvironment()` |
| pressure_hpa | `App_CopyEnvironment()` |
| humidity_pct | `App_CopyEnvironment()` |
| voltage_v | `App_CopyEnvironment()` |
| battery_pct | `App_CopyEnvironment()` |

缺失数据处理：

- GNSS 未就绪时，`gnss_valid=0`，经纬度写 `0`。
- IMU 姿态未就绪时，`roll_deg=0.00`，`pitch_deg=0.00`，并在 flags 或错误日志中记录一次。
- ENV 未就绪时，不写数据行，写入一条错误日志或增加跳过计数。

## 8. 错误信息来源

`SYSTEM_ERROR.CSV` 记录以下事件：

1. SD 卡挂载失败。
2. SD 卡写入失败。
3. SD 卡同步失败。
4. Storage 队列满。
5. CSV 行格式化失败或被截断。
6. GNSS 长时间无有效数据。
7. IMU 离线或 FIFO 溢出。
8. BME280 离线。
9. ADC 电池电压采集失败。
10. 低电压告警。

错误消息建议使用固定英文短字符串，避免中文编码问题：

```text
sd_mount_failed
sd_write_failed
sd_sync_failed
storage_queue_full
csv_line_truncated
gnss_not_ready
imu_offline
baro_offline
battery_offline
low_voltage
```

## 9. 栈溢出风险控制

Storage 模块栈风险来自：

- FatFS 局部对象。
- CSV 格式化。
- SD SPI 扇区缓冲。
- Debug 打印。

控制方案：

1. Storage 任务初始栈配置 `768W`。
2. `FATFS`、`FIL`、扇区缓冲、CSV 工作缓冲放静态区。
3. 单个函数局部对象超过 `128` 字节必须审查。
4. 超过 `256` 字节的缓冲必须移到静态区或固定池。
5. Storage 任务必须注册到 `DebugTaskMonitor_Register()`。
6. 串口验收时必须看到 `storage` 栈监控。
7. 合格标准为剩余栈不少于 `100W`，且不少于配置栈的 `25%`。

验收打印示例：

```text
[DBG] STACK: task=storage cfg=768W peak=320W min_free=448W/1792B OK
```

## 10. 程序卡死风险控制

SD 卡写入可能卡住的位置：

- SD 初始化等待响应。
- SPI 读写等待。
- `f_mount()`。
- `f_open()`。
- `f_write()`。
- `f_sync()`。
- SD 卡拔出或接触不良。

控制方案：

1. SD 初始化有限次数重试。
2. SPI 每次收发必须带超时。
3. Storage 任务状态机运行，失败后延时重试，不阻塞系统启动。
4. 文件写入失败只设置状态和错误计数，不进入死循环。
5. `f_sync()` 周期执行，不每行都同步。
6. 生产者投递队列使用 `0` 超时或短超时。
7. 队列满丢弃日志，不阻塞采集。
8. 不允许 Sensor、Estimator、Health 任务直接调用 FatFS。
9. 不允许 ISR 访问 SD 或 FatFS。
10. Debug 任务只读取 Storage 状态快照，不访问文件。

## 11. 写入策略

### 11.1 数据日志

默认 1Hz 写入：

```text
STORAGE_DATA_PERIOD_MS = 1000U
```

第一版不建议 10ms 写 IMU 原始数据，原因：

- CSV 文本写入开销大。
- SD 卡写入延迟不稳定。
- 10ms 写入会增加队列堆积和卡顿风险。
- 当前目标是验证可靠记录，而不是高频黑匣子。

### 11.2 同步策略

默认 5 秒同步一次：

```text
STORAGE_SYNC_PERIOD_MS = 5000U
```

如果每行都 `f_sync()`，掉电安全性更高，但写入阻塞风险更大。第一版建议每 5 秒同步，后续可按实际需求调整。

### 11.3 表头策略

文件不存在或大小为 0 时写入表头。

如果文件已存在且非空：

- 继续追加。
- 不重复写表头。

## 12. 状态机

Storage 任务状态机：

```text
UNINIT
  -> INIT_SPI
  -> MOUNT
  -> OPEN_FILES
  -> READY
  -> DEGRADED
  -> RETRY_WAIT
```

状态说明：

| 状态 | 行为 |
|---|---|
| UNINIT | 静态对象清零，等待任务启动 |
| INIT_SPI | 初始化 SPI1 和 CS |
| MOUNT | 挂载 FatFS |
| OPEN_FILES | 打开 `SENSOR_DATA.CSV` 和 `SYSTEM_ERROR.CSV` |
| READY | 消费队列并写入 |
| DEGRADED | 写入失败或同步失败，记录错误 |
| RETRY_WAIT | 等待重试，不阻塞其他任务 |

## 13. 开发步骤

### 阶段 1：配置和空模块

1. 新建 `Storage` 目录。
2. 新建 `storage_config.h`。
3. 保持 `PX4LITE_ENABLE_STORAGE = 0U`。
4. 提供空的 `Storage_CreateTask()`，关闭时不创建任务。
5. 编译确认现有功能不受影响。

### 阶段 2：BSP SPI

1. 新建 `bsp_spi.h/.c`。
2. 配置 PB3/PB4/PB5 为 SPI1 AF5。
3. 配置 PA15 为 GPIO 输出。
4. 实现低速和高速 SPI 配置。
5. 实现带超时的收发。
6. 单元测试验证 CS 控制和超时返回。

### 阶段 3：FatFS 接入

1. 加入 FatFS 源码。
2. 配置 `ffconf.h`，只打开 CSV Demo 必需能力。
3. 实现 `diskio_sd_spi.c`。
4. SD 初始化必须有限次重试。
5. 无卡时返回错误，不死等。

### 阶段 4：Storage SD 驱动

1. 实现 `Storage_SD_Init()`。
2. 实现 mount/open/write/sync 封装。
3. 实现 `Storage_Status_t`。
4. 所有 FatFS 错误映射成 `Px4Lite_Result_t` 和故障码。

### 阶段 5：CSV 格式化

1. 定义两个文件名。
2. 定义两个表头。
3. 实现数据 CSV 行格式化。
4. 实现错误 CSV 行格式化。
5. 单元测试覆盖正常、缺失数据、超长消息截断。

### 阶段 6：Storage 任务

1. 创建 Storage 任务。
2. 创建固定长度记录队列。
3. 注册栈监控。
4. 每 1000ms 生成一条数据记录。
5. 消费队列写入 SD。
6. 每 5000ms 执行一次 sync。

### 阶段 7：Debug 状态打印

1. 打开 `DEBUG_STORAGE_MONITOR_ENABLE`。
2. Debug 任务只复制 Storage 状态快照。
3. 每 5000ms 打印一次 Storage 状态。
4. 打印中不得直接访问 FatFS。

### 阶段 8：打开功能

1. 将 `PX4LITE_ENABLE_STORAGE` 改为 `1U`。
2. 将 `DEBUG_STORAGE_MONITOR_ENABLE` 改为 `1U`。
3. 编译下载。
4. 插卡启动验证。
5. 无卡启动验证。
6. 写入过程中拔卡验证。
7. 长时间运行验证。

## 14. 测试计划

### 14.1 单元测试

建议新增：

```text
Tests/Unit/test_storage_csv.c
Tests/Unit/test_storage_queue.c
Tests/Unit/test_storage_status.c
```

覆盖：

- CSV 表头内容正确。
- 数据行字段数量正确。
- 姿态、电压、气压浮点格式正确。
- 错误消息过长时被安全截断。
- 队列满时不阻塞并增加丢弃计数。
- Storage 状态从失败恢复到重试。

### 14.2 硬件测试

测试清单：

| 场景 | 期望结果 |
|---|---|
| 正常插卡启动 | 创建两个 CSV 文件 |
| 无卡启动 | 系统正常运行，Storage 状态为 degraded/offline |
| 写入中拔卡 | 采集继续，Storage 错误计数增加 |
| 重新插卡 | 周期重试后恢复写入 |
| 长时间运行 30 分钟 | 无栈报警，无 heap 异常 |
| 电脑读取 SD | CSV 可直接打开 |

### 14.3 串口验收

正常运行时应看到：

```text
[DBG] STORAGE: state=2 mounted=1 written=... err=0 drop=0 q=0 sync=1 age=...
[DBG] STACK: task=storage cfg=768W ... OK
```

无卡或异常时应看到：

```text
[DBG] STORAGE: state=4 mounted=0 written=0 err=... drop=... q=... sync=0 age=...
```

同时必须继续看到：

```text
[DBG] IMU: ...
[DBG] ENV: ...
[DBG] GNSS: ...
```

这说明 SD 异常没有拖死传感器链路。

## 15. 验收标准

功能验收：

1. SD 卡根目录生成 `SENSOR_DATA.CSV`。
2. SD 卡根目录生成 `SYSTEM_ERROR.CSV`。
3. `SENSOR_DATA.CSV` 有表头和真实采集数据。
4. `SYSTEM_ERROR.CSV` 有表头，异常时能记录事件。
5. 电脑可直接读取两个 CSV 文件。

稳定性验收：

1. 无卡启动不死机。
2. 写入中拔卡不死机。
3. SD 写入失败不影响 IMU、BME280、ADC、GNSS 采集。
4. Storage 任务栈监控为 OK。
5. 系统 heap 峰值使用率不超过规范限制。
6. Debug 打印不出现长时间停顿或无限等待。

代码规范验收：

1. 不在 ISR 中访问 FatFS。
2. 不在 Sensor/Estimator 任务中访问 FatFS。
3. 不使用弱类型通道记录正式数据。
4. 不把大缓冲放到任务栈。
5. 所有慢速外设访问都有超时。
6. 功能关闭时不创建 Storage 任务、队列或大缓冲。

## 16. 推荐结论

本模块按以下推荐方案执行：

```text
SPI1 + PA15 CS
FatFS
独立低优先级 Storage Task
固定长度 CSV 记录队列
SENSOR_DATA.CSV + SYSTEM_ERROR.CSV
1Hz 数据写入
5s 周期同步
Debug 状态 + 栈监控
```

该方案可以实现 SD 卡 CSV 写入，并能在电脑上读取；同时通过任务隔离、固定队列、超时、状态机和栈监控降低栈溢出和程序卡死风险。

