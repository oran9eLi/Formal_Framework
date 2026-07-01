# SD-CSV 模块技术文档

## 1. 模块定位

SD-CSV 模块用于把系统采集数据和错误事件写入 SD 卡 CSV 文件。该模块是低优先级阻塞外设服务，所有 FatFS 和 SD/SPI 写入都限制在 `storage` 任务中执行，不能影响 `sensor`、`estimator`、`display` 等实时任务。

正式数据链路如下：

```text
Framework Topics
  -> px4lite_storage.c
  -> storage_csv.c
  -> storage_queue.c
  -> storage_sd.c
  -> diskio_sd_spi.c
  -> bsp_spi.c
  -> SD Card
```

## 2. 硬件与文件

| 项目 | 配置 |
|---|---|
| 总线 | SPI3 |
| 引脚 | PB3/SCK，PB4/MISO，PB5/MOSI，PA15/CS |
| 文件系统 | FatFS |
| 数据文件 | `YYMMDD_D.CSV`，例如 `260622_D.CSV`；时间无效时为 `UNSYNC_D.CSV` |
| 事件文件 | `YYMMDD_E.CSV`，例如 `260622_E.CSV`；时间无效时为 `UNSYNC_E.CSV` |
| 服务周期 | 50 ms |
| 数据记录周期 | 1000 ms |
| 同步周期 | 5000 ms |
| 所属任务 | `storage` |
| 任务优先级 | idle |

PB3、PB4、PA15 与 JTAG 复用，调试时应使用 SWD，避免 JTAG 占用 SD 引脚。

## 3. 软件分层

| 层 | 文件 | 职责 |
|---|---|---|
| BSP | `Bsp/Src/bsp_spi.c` | SPI 初始化、速度切换、CS 控制、超时收发 |
| diskio | `Storage/Src/diskio_sd_spi.c` | FatFS diskio 到 SD SPI 协议适配 |
| SD Service | `Storage/Src/storage_sd.c` | mount、open、write、sync、状态维护 |
| CSV | `Storage/Src/storage_csv.c` | CSV 表头和数据行格式化 |
| Queue | `Storage/Src/storage_queue.c` | 固定长度非阻塞记录队列 |
| Framework | `Framework/Src/px4lite_storage.c` | 从 Topic 取数、生成记录、驱动存储状态 |

## 4. CSV 格式

`YYMMDD_D.CSV` 表头：

```csv
time_ms,local_date,local_time,time_sync_state,gnss_valid,lat_e7,lon_e7,roll_deg,pitch_deg,yaw_deg,temp_c,pressure_hpa,humidity_pct,voltage_v,current_a,power_w,battery_pct,low_voltage,voltage2_v,current2_a,power2_w,battery2_pct,low_voltage2,motor1_pct,motor2_pct,motor3_pct,motor4_pct,motor_run_state,active_alarm_count,highest_fault_code,lora_rx_count,lora_tx_count,lora_parse_error_count,lora_send_error_count,storage_queue_count,storage_drop_count
```

`YYMMDD_E.CSV` 表头：

```csv
time_ms,local_date,local_time,event_type,source,state,fault,severity,active,count,message
```

格式化要求：

- 不使用浮点 `printf`。
- 姿态、温度、气压、湿度、电压使用固定小数格式输出。
- 单行最大长度由 `STORAGE_CSV_LINE_MAX` 限制，当前为 512 字节。
- 格式化溢出返回 `PX4LITE_OVERFLOW`，不能写越界。

## 5. 队列策略

Storage 使用固定长度静态队列，当前长度为 8。

行为规则：

- 生产者入队不阻塞。
- 队列满时丢弃新记录并增加 `drop_count`。
- 临界区只保护 head、tail、count 和记录复制。
- 不在 ISR 中写 SD，不在高优先级任务中调用 FatFS。

## 6. SD 状态机

`storage_sd.c` 维护 SD 文件状态：

| 状态 | 行为 |
|---|---|
| UNINITIALIZED | 静态对象清零，等待初始化 |
| STARTING | 尝试 mount 和打开文件 |
| ONLINE | 文件已打开，可写入 |
| DEGRADED | mount、open、write 或 sync 失败 |

失败后按 `STORAGE_MOUNT_RETRY_MS` 退避，当前为 2000 ms。恢复回调只设置 remount 请求，真正 mount/open 在 storage 任务中执行。

## 7. 数据来源

`px4lite_storage.c` 只读取 Framework Topic：

| CSV 字段 | 来源 |
|---|---|
| GNSS 有效、经纬度 | Navigation Topic |
| roll/pitch | Navigation Topic |
| 温度、气压、湿度 | Baro Topic |
| 第一电池电压、电流、功率、电量、低电压标志 | Battery Topic |
| 第二电池电压、电流、功率、电量、低电压标志 | Battery2 Topic |

第二电池用于四个无刷电机供电，电量百分比使用独立动力电池曲线：`10.50V` 为 0%，`12.60V` 为 100%。`low_voltage2` 使用滤波电压和连续确认防抖，低于 `10.80V` 后进入低压预警，高于 `11.00V` 后解除预警。

第一电池用于开发板供电，`low_voltage` 同样使用滤波电压和连续确认防抖，低于 `10.30V` 后进入低压预警，高于 `10.50V` 后解除预警。

缺失数据不会静默写入假值，而是向 `YYMMDD_E.CSV` 记录如 `nav_not_ready`、`baro_not_ready`、`battery_not_ready`、`battery2_not_ready` 等错误事件。第一电池和第二电池低电压分别记录 `low_voltage`、`low_voltage2` 事件。RTC/GNSS 日期尚未有效时，记录写入 `UNSYNC_E.CSV`，行内 `local_date` 和 `local_time` 为 0。

## 8. 验收要点

- 插卡启动后根目录按日期生成 `YYMMDD_D.CSV` 和 `YYMMDD_E.CSV`；时间未同步前生成 `UNSYNC_D.CSV` 和 `UNSYNC_E.CSV`。
- 无卡启动或写入失败时系统继续运行，Storage 进入 DEGRADED。
- SD 写入不能影响 GNSS、IMU、BME280、ADC 采集。
- `storage` 任务栈监控应保持安全余量。
- CSV 文件能在电脑上直接打开。
- 所有 FatFS 调用只允许出现在 Storage 链路中。
