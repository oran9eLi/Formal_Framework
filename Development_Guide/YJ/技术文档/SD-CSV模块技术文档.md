# SD-CSV 存储模块技术文档

## 文档元信息

| 项目 | 内容 |
|---|---|
| 模块名称 | 存储模块 —— SD-CSV 日志（数据文件 `_D` + 事件文件 `_E`） |
| 对应设计书章节 | 《低空 CNS 平台产品设计书 V1.0》存储/黑匣子记录能力 |
| 接入范式 | **数据落地/输出模块**，不属于样板 A（传感器接收）/样板 B（命令应答）采集范式；数据流方向为 Framework Topic → SD 卡（只写不收），沿用统一章节结构 |
| 参考资料 | 本工程源码 `Storage/Src/*.c`、`Storage/Inc/storage_config.h`、`Framework/Src/px4lite_storage.c`；FatFS |
| 文档版本 | V2.0 |
| 作者 / 日期 | YJ / 2026-07-14 |
| 模块状态 | 已跑通（见 [07_移植进度与功能清单.md](../../07_移植进度与功能清单.md)） |

---

## 1. 模块定位

SD-CSV 存储模块把系统采集数据和告警/状态事件写入 SD 卡上按日期命名的两类 CSV 文件（数据文件 `_D`、事件文件 `_E`），相当于无人机的"黑匣子/飞行记录仪"。

该模块是**低优先级阻塞外设服务**：所有 FatFS 和 SD/SPI 写入都限制在 `storage` 任务中执行，`storage` 任务运行在 idle 优先级，不能影响 `sensor`、`estimator`、`display` 等实时任务（优先级选择的原因见 §11 案例 1）。模块只消费 Framework Topic，不产生传感器数据、也不向业务层回读。

正式数据链路如下：

```text
Framework Topics
  -> px4lite_storage.c   (从 Topic 取数、生成数据/事件记录)
  -> storage_queue.c     (固定长度非阻塞记录队列)
  -> storage_sd.c        (按日期开/换文件、write、sync、状态维护)
  -> diskio_sd_spi.c     (FatFS diskio 到 SD SPI 协议适配)
  -> bsp_spi.c           (SPI 收发)
  -> SD Card
```

---

## 2. 硬件与总线

| 项目 | 配置 |
|---|---|
| 总线 | SPI3 |
| 引脚 | PB3/SCK，PB4/MISO，PB5/MOSI，PA15/CS |
| 文件系统 | FatFS |
| 数据文件 | `0:/YYMMDD_D.CSV`（时间未同步时为 `0:/UNSYNC_D.CSV`） |
| 事件文件 | `0:/YYMMDD_E.CSV`（时间未同步时为 `0:/UNSYNC_E.CSV`） |
| 分文件策略 | 按本地日期每天一组（`_D` + `_E`），日期变化时自动关旧文件、开新文件 |
| 服务周期 | 50 ms（`STORAGE_SERVICE_PERIOD_MS`） |
| 数据记录周期 | 1000 ms（`STORAGE_DATA_PERIOD_MS`） |
| 同步周期 | 5000 ms（`STORAGE_SYNC_PERIOD_MS`） |
| 所属任务 | `storage` |
| 任务优先级 | idle（`tskIDLE_PRIORITY`，原因见 §11 案例 1） |

**文件名规则**（见 [storage_file.c](../../../Storage/Src/storage_file.c)）：文件名由本地日期（`YYMMDD`）加类型后缀（`D`=数据、`E`=事件）组成，符合 FatFS 8.3 短文件名限制；本地日期来自时间同步（GNSS/RTC），未同步（`date_ymd==0`）时统一落到 `UNSYNC_*` 文件。后缀常量定义在 [storage_config.h](../../../Storage/Inc/storage_config.h)（`STORAGE_DATA_FILE_SUFFIX`/`STORAGE_EVENT_FILE_SUFFIX`）。

**引脚约束**：PB3、PB4、PA15 与 JTAG 复用，调试时应使用 SWD，避免 JTAG 占用 SD 引脚（见 §11 案例 2）。

---

## 3. 软件分层与依赖

| 层 | 文件 | 职责 |
|---|---|---|
| BSP | `Bsp/Src/bsp_spi.c` | SPI 初始化、速度切换、CS 控制、超时收发 |
| diskio | `Storage/Src/diskio_sd_spi.c` | FatFS diskio 到 SD SPI 协议适配 |
| SD Service | `Storage/Src/storage_sd.c` | mount、按日期开/换文件、write、sync、状态维护 |
| 文件名 | `Storage/Src/storage_file.c` | 由本地日期 + 类型后缀生成 8.3 文件名 |
| 时间 | `Storage/Src/storage_time.c` | 缓存本地日期/时间与同步状态，供文件名与记录取用 |
| CSV | `Storage/Src/storage_csv.c` | CSV 表头和数据/事件行格式化 |
| Queue | `Storage/Src/storage_queue.c` | 固定长度非阻塞记录队列 |
| 事件跟踪 | `Storage/Src/storage_event.c`、`storage_module_event.c`、`storage_drop_event.c` | 告警/状态变化、模块状态跳变、队列丢弃的去抖与事件生成 |
| Framework | `Framework/Src/px4lite_storage.c` | 从 Topic 取数、生成数据/事件记录、驱动存储状态 |

依赖单向：所有 FatFS 调用只允许出现在 Storage 链路中；`sensor`/`estimator`/`display` 等实时任务不得直接调用 FatFS 或 SD 写入。

---

## 4. 数据格式（本节是复现本模块的核心，给真实代码）

表头与行格式化由 [storage_csv.c](../../../Storage/Src/storage_csv.c) 定义。

**数据文件 `_D.CSV`** 表头（`StorageCsv_DataHeader()`，共 36 列）：

```csv
time_ms,local_date,local_time,time_sync_state,gnss_valid,lat_e7,lon_e7,roll_deg,pitch_deg,yaw_deg,temp_c,pressure_hpa,humidity_pct,voltage_v,current_a,power_w,battery_pct,low_voltage,voltage2_v,current2_a,power2_w,battery2_pct,low_voltage2,motor1_pct,motor2_pct,motor3_pct,motor4_pct,motor_run_state,active_alarm_count,highest_fault_code,lora_rx_count,lora_tx_count,lora_parse_error_count,lora_send_error_count,storage_queue_count,storage_drop_count
```

**事件文件 `_E.CSV`** 表头（`StorageCsv_EventHeader()`，共 11 列）：

```csv
time_ms,local_date,local_time,event_type,source,state,fault,severity,active,count,message
```

事件行字段含义：`event_type` 为 `ALARM_ACTIVE`（告警产生）或 `STATUS_RECOVERED`（告警恢复）；`source` 为来源模块名（如 `NAV`、`BARO`、`BATTERY`、`STORAGE`）；`state` 为模块状态；`fault` 为 Framework 故障码；`severity` 为严重度；`active` 为 0/1；`count` 为该事件累计次数；`message` 为文本说明（如 `baro_not_ready`、`storage_queue_drop`）。

**文件名生成**（`storage_file.c`，8.3 短名 + UNSYNC 兜底）：

```c
if (date_ymd == 0U) {
  /* 时间未同步：统一落到 UNSYNC_D.CSV / UNSYNC_E.CSV */
  snprintf(path, path_size, "0:/UNSYNC_%c.CSV", suffix);
} else {
  /* 已同步：0:/YYMMDD_D.CSV 或 0:/YYMMDD_E.CSV */
  snprintf(path, path_size, "0:/%02lu%02lu%02lu_%c.CSV", yy, mm, dd, suffix);
}
```

格式化要求：

- 不使用浮点 `printf`，改用整数定点格式化（见 `Storage_FormatSignedCenti`/`Storage_FormatSignedMilli`）。
- 姿态、温度、气压、湿度使用两位小数；电压、电流、功率使用三位小数。
- 单行最大长度由 `STORAGE_CSV_LINE_MAX` 限制，当前为 512 字节（见 [storage_config.h](../../../Storage/Inc/storage_config.h)）。
- 格式化溢出返回 `PX4LITE_OVERFLOW`，不能写越界。

> 说明：`storage_csv.c` 中还保留了 `StorageCsv_ErrorHeader()`（`time_ms,module,state,fault,error_count,message`）与 `StorageCsv_FormatErrorLine()`，为早期独立错误文件设计的遗留接口，**当前无任何调用方**，实际错误一律走上述 11 列事件文件。后续清理时可移除。

---

## 5. 采样 / 处理流程

`storage` 任务每 50 ms 调用一次 `Px4Lite_StorageWorkRun(now_ms)`（[px4lite_storage.c](../../../Framework/Src/px4lite_storage.c)），一轮完成"取数 → 入队 → 出队写卡 → 同步"的流水：

1. **处理 remount 请求**：若 `recover` 回调置了标志，先 `Storage_SD_Init()` 清状态，让下一步立即重挂。
2. **更新本地时间**：`Px4Lite_CopyTime` → `Storage_TimeUpdate`，供文件名和记录取用。
3. **SD 服务**：`Storage_SD_Service` 按状态机决定是否 `f_mount`（见 §8）。
4. **模块状态事件**：`Storage_CheckModuleStatusEvents` 检查各模块状态跳变，生成事件入队。
5. **生成数据记录（每 1000 ms）**：`Storage_ProduceDataRecord` 从各 Topic 取数、格式化成一行，入队。数据来源逐列如下：

   | CSV 字段 | 来源 Topic / 接口 |
   |---|---|
   | `local_date`、`local_time`、`time_sync_state` | Time（`Px4Lite_CopyTime`） |
   | `gnss_valid`、`lat_e7`、`lon_e7`、`roll_deg`、`pitch_deg`、`yaw_deg` | Navigation（`Px4Lite_CopyNavigation`） |
   | `temp_c`、`pressure_hpa`、`humidity_pct` | Baro（`Px4Lite_CopyBaro`） |
   | `voltage_v`、`current_a`、`power_w`、`battery_pct`、`low_voltage` | Battery（`Px4Lite_CopyBattery`） |
   | `voltage2_v`、`current2_a`、`power2_w`、`battery2_pct`、`low_voltage2` | Battery2（`Px4Lite_CopyBattery2`） |
   | `motor1_pct`~`motor4_pct`、`motor_run_state` | Motor（`Px4Lite_CopyMotor`） |
   | `active_alarm_count`、`highest_fault_code` | Alarm（`Px4Lite_CopyAlarmSnapshot`） |
   | `lora_rx_count`~`lora_send_error_count` | Comm/LoRa（`Px4Lite_GetCommDebugInfo`） |
   | `storage_queue_count`、`storage_drop_count` | Storage 队列自身状态 |

6. **出队写卡（每轮 1 条）**：`Storage_ConsumeOne` 从队列弹出 1 条记录，按类型写入当日 `_D` 或 `_E` 文件。50 ms 一轮、每轮 1 条，最高约 20 条/秒的落盘能力，足以覆盖 1 秒 1 条数据 + 偶发事件。
7. **队列丢弃事件**：`Storage_CheckQueueDropEvent` 把 `drop_count` 变化转成 `storage_queue_drop` 事件，让丢弃可见（见 §8）。
8. **周期同步（每 5000 ms）**：`Storage_SD_Sync` 调 `f_sync` 把缓冲落盘，降低掉电丢数据窗口。
9. **发布模块状态**：`Storage_PublishState` 按 SD 就绪与否升 ONLINE/DEGRADED。

**数据来源缺失处理**：Navigation/Baro/Battery/Battery2 任一源缺失时，不静默写假值，而是向当日事件文件写 `nav_not_ready`/`baro_not_ready`/`battery_not_ready`/`battery2_not_ready` 事件，恢复时补记 `STATUS_RECOVERED`；Motor/Alarm/Comm 源缺失时对应列保持 0，不单独产生事件。

---

## 6. 接口清单

**Framework 生命周期接口**（注册表驱动，由 storage 任务调用）：

```c
Px4Lite_Result_t Px4Lite_StorageModuleInit(void);      /* 初始化时间/队列/事件跟踪/SD */
Px4Lite_Result_t Px4Lite_StorageRecover(void);         /* 置 remount 请求标志，不阻塞 */
void             Px4Lite_StorageWorkRun(uint32_t now_ms); /* storage 任务每 50ms 调一次 */
```

**SD 服务层接口**（`storage_sd.c`，仅供 Storage 链路内部使用）：

```c
void             Storage_SD_Init(void);
void             Storage_SD_Service(uint32_t now_ms);
Px4Lite_Result_t Storage_SD_WriteDataLine(const char *line, uint32_t target_date_ymd, uint32_t now_ms);
Px4Lite_Result_t Storage_SD_WriteEventLine(const char *line, uint32_t target_date_ymd, uint32_t now_ms);
Px4Lite_Result_t Storage_SD_Sync(uint32_t now_ms);
uint8_t          Storage_SD_IsReady(void);             /* ONLINE 且 mounted 才为 1 */
```

**队列接口**（`storage_queue.c`）：`StorageQueue_Init` / `Push` / `Pop` / `Count` / `DropCount`。

> 存储模块是**只写数据落地端**，不向业务/显示层提供回读快照接口，因此没有传感器模块那种 `CopySnapshot`/`App_Copy*` 读取入口。业务层只通过模块状态（ONLINE/DEGRADED）感知 SD 是否可用。

---

## 7. 配置项

| 配置文件 | 关键项 | 值 | 说明 |
|---|---|---|---|
| `Framework/Inc/px4lite_config.h` | `PX4LITE_ENABLE_STORAGE` | — | 关闭时不注册存储模块、不建 storage 任务 |
| `Storage/Inc/storage_config.h` | `STORAGE_TASK_PRIORITY` | `tskIDLE_PRIORITY` | storage 任务优先级，必须最低（原因见 §11 案例 1） |
| 同上 | `STORAGE_TASK_STACK_WORDS` | `768` | storage 任务栈深度 |
| 同上 | `STORAGE_SERVICE_PERIOD_MS` | `50` | `WorkRun` 服务周期 |
| 同上 | `STORAGE_DATA_PERIOD_MS` | `1000` | 数据记录周期 |
| 同上 | `STORAGE_SYNC_PERIOD_MS` | `5000` | `f_sync` 落盘周期 |
| 同上 | `STORAGE_MOUNT_RETRY_MS` | `2000` | mount 失败后的重试退避 |
| 同上 | `STORAGE_SPI_TIMEOUT_MS` | `50` | 单次 SPI 传输超时 |
| 同上 | `STORAGE_QUEUE_LENGTH` | `8` | 记录队列长度 |
| 同上 | `STORAGE_CSV_LINE_MAX` | `512` | 单行 CSV 最大字节数 |
| 同上 | `STORAGE_DATA_FILE_SUFFIX` / `STORAGE_EVENT_FILE_SUFFIX` | `'D'` / `'E'` | 数据/事件文件名后缀 |

---

## 8. 异常与恢复

**SD 状态机**：`storage_sd.c` 用 Framework 的 `Px4Lite_State_t` 维护挂载/写入状态（由 `Storage_SD_Service` 驱动）：

| 状态 | 含义 / 行为 |
|---|---|
| UNINITIALIZED | 静态对象清零，等待首次 Service |
| STARTING | 本轮尝试 `f_mount`（只挂载，**不在此时打开数据/事件文件**） |
| ONLINE | 挂载成功、可写入；数据/事件文件在首次写入对应日期记录时才按需打开 |
| DEGRADED | mount / write / sync 失败；失败时关闭已开文件并等待退避后重挂 |

- 数据文件与事件文件不在挂载时统一打开，而是由 `Storage_SD_EnsureFileForDate()` 在写第一条该日期记录时**按需打开**，日期变化时自动关旧开新（见 §2 分文件策略）。
- 未在线时每 `STORAGE_MOUNT_RETRY_MS`（2000 ms）才重试一次 `f_mount`，不会每个 50 ms 服务周期都去碰卡。
- 无卡启动或写入失败时系统继续运行，Storage 进入 DEGRADED，不影响其他任务。
- **队列满即丢弃**：生产者入队不阻塞，队列满时丢弃新记录并 `drop_count++`，随后由 `Storage_CheckQueueDropEvent` 写一条 `storage_queue_drop` 事件，使丢弃可观测而不是静默丢失。
- OFFLINE / FAILED 属于同一 `Px4Lite_State_t` 枚举，但由 Health 按超时统一判定，不在 `storage_sd.c` 内设置。
- 恢复回调（`Px4Lite_StorageRecover`）只设置 remount 请求标志，真正的重新 `Init`+mount 在 storage 任务中执行，不在回调里做阻塞 I/O。

---

## 9. 调试

- **状态诊断字段**：`storage_sd.c` 维护 `Storage_SdStatus_t`，含 `disk_error`、`card_type`、`mount_result`、`last_command`、`last_response`、`open_phase`、`open_result`、`error_count`、`written_count` 等，挂卡/写入失败时可据此定位是 mount、open、header 还是 seek 阶段出错。
- **软件侧**：看 `error_count`/`written_count` 是否随时间合理增长、模块状态是否稳定 ONLINE。
- **硬件侧**：确认 SD 卡座供电与 CS/SCK/MISO/MOSI 接线；用 SWD 而非 JTAG 调试（PB3/PB4/PA15 与 JTAG 复用，见 §11 案例 2）。
- **成品核对**：把 SD 卡插电脑，用表格软件直接打开 `YYMMDD_D.CSV`；找不到当天日期文件时先看是否落在 `UNSYNC_*`（时间未同步）。

---

## 10. 验收要点

| 用例 | 操作步骤 | 预期结果 | 实际结果 |
|---|---|---|---|
| 按日期建文件 | 插卡启动 | 根目录按本地日期生成 `YYMMDD_D.CSV` 与 `YYMMDD_E.CSV`；未同步时先落 `UNSYNC_*` | |
| 跨日切换 | 运行跨过 0 点 | 日期变化后自动切换到新一天的 `_D`/`_E` 文件 | |
| 无卡容错 | 不插卡或写入失败启动 | 系统继续运行，Storage 进入 DEGRADED，其他任务不受影响 | |
| 实时性隔离 | 长时间记录 | SD 写入不影响 GNSS、IMU、BME280、ADC 采集 | |
| 缺数据不写假值 | 拔掉某传感器 | 对应 `_E.CSV` 出现 `xxx_not_ready` 事件，数据列不写假值 | |
| 栈余量 | 长稳运行 | `storage` 任务栈保持安全余量 | |
| 可离线分析 | 取卡在电脑打开 | CSV 文件能直接打开、列对齐 | |

---

## 11. 开发踩坑与教学案例

> 每条按 **现象 → 定位 → 根因 → 解法 → 教学点/学生易错** 组织。

**问题 1：storage 任务优先级给高了一档，UI 卡在 LOGO 页刷不出来**（类别：软件；RTOS 优先级；源见 [storage_config.h](../../../Storage/Inc/storage_config.h) 注释）

- **现象**：开机后界面停在 LOGO 页，触摸无响应，像是显示任务挂了。
- **定位**：storage 任务做阻塞 SD/SPI I/O（`f_mount`/`disk_initialize` 会忙等，无卡时每 `STORAGE_MOUNT_RETRY_MS` 还重试）；一旦把它放在 `idle+1`，它会和显示任务时间片竞争，饿死触摸扫描。
- **根因**：storage 是低优先级**阻塞** I/O 任务，只要它优先级不严格低于显示任务，就会在显示任务该跑的时候抢到 CPU 做阻塞 I/O。
- **解法**：`STORAGE_TASK_PRIORITY = tskIDLE_PRIORITY`（idle，严格低于 `biz_display`）。让显示和触摸永远能抢占 storage，storage 只在 10 ms 显示任务 `vTaskDelayUntil` 睡眠的空隙里跑。
- **教学点 / 学生易错**：给任务定优先级时只想"这个任务重不重要"，忽略"这个任务里有没有阻塞 I/O"。阻塞型后台任务必须放到最低优先级，否则会用不显眼的方式饿死看起来不相关的实时任务。

**问题 2：SD 引脚与 JTAG 复用，用 JTAG 调试时 SD 读写异常**（类别：硬件/软硬结合）

- **现象**：单独跑固件时 SD 正常，接调试器用 JTAG 调试时 SD 挂载/读写出问题。
- **根因**：PB3、PB4、PA15 与 JTAG 复用，JTAG 会占用这些引脚。
- **解法**：调试统一用 SWD，不用 JTAG；量产也不依赖 JTAG 引脚。
- **教学点 / 学生易错**：忽略调试口和外设引脚的复用关系，"一插调试器外设就出问题"往往是引脚被调试口占了。

**问题 3：队列满了却以为数据没丢**（类别：软件）

- **现象**：某段时间 SD 写入跟不上，事后发现数据行比预期少，但当时没有任何报错。
- **根因**：记录队列固定长度 8，生产者入队不阻塞，队列满时**直接丢弃**新记录——如果丢弃不上报，就成了静默丢数据。
- **解法**：丢弃时 `drop_count++`，并由 `Storage_CheckQueueDropEvent` 写 `storage_queue_drop` 事件、`storage_drop_count` 列也一并落盘，让丢弃可观测、可事后复盘。
- **教学点 / 学生易错**：有损队列必须让"丢了多少"可见。只丢不计数，问题会被永久掩盖。

**问题 4：找不到"今天"的数据文件**（类别：软件/使用）

- **现象**：按当天日期去 SD 卡找 `YYMMDD_D.CSV`，找不到。
- **根因**：GNSS/RTC 还没对上时间时（`date_ymd==0`），数据先落到 `UNSYNC_D.CSV`/`UNSYNC_E.CSV`，不是日期文件。
- **解法/教学点**：分析数据前先确认时间是否同步；`UNSYNC_*` 文件就是"这段记录时系统还不知道当前日期"的标志。

---

## 12. 变更记录

| 日期 | 版本 | 作者 | 变更说明 | 关联记录 |
|---|---|---|---|---|
| 2026-XX-XX | V1.0 | YJ | 首版（自定义 8 节结构，文件名/表头/行长等多处与代码不符） | — |
| 2026-07-14 | V1.1 | YJ | 核对源码修正事实：文件名改为按日期 `YYMMDD_D/E.CSV`（+`UNSYNC` 兜底）、数据表 36 列、事件表 11 列、行长 512、补全数据来源与事件机制、标注遗留死代码 | 本次代码审查 |
| 2026-07-14 | V2.0 | YJ | 按 [00_模块技术文档范本.md](00_模块技术文档范本.md) 重构为元信息 + 12 节结构：补文档元信息、采样/处理流程、接口清单、配置项、调试、开发踩坑教学、变更记录；元信息注明本模块为数据落地/输出模块，不套用样板 A/B 采集范式 | 本次代码审查 |
