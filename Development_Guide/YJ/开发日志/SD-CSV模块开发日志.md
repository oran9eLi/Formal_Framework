# SD-CSV 模块开发日志

## 1. 开发目标

为 NW3 工程增加 SD 卡 CSV 日志能力，将正常传感器数据和系统错误事件分别写入文件，同时保证 SD 卡异常不会影响实时采集和显示。

## 2. 开发过程

### 2.1 方案确定

最终采用独立低优先级 `storage` 任务，而不是在 Sensor、Estimator、Debug 或 Business 任务中直接写 SD。

原因：

- FatFS 和 SD SPI 写入可能阻塞；
- SD 卡无卡、接触不良、写失败都不能拖死主系统；
- 低优先级任务可以让 Display、Sensor、Estimator 优先运行。

### 2.2 BSP SPI 与 FatFS

新增 BSP SPI 支持，负责：

- SPI 初始化；
- 低速/高速切换；
- PA15 CS 控制；
- 带超时的发送、接收、收发。

FatFS 通过 `diskio_sd_spi.c` 适配到底层 SD SPI 协议，Storage 上层不直接接触 SD 命令细节。

### 2.3 CSV 与队列

新增 `storage_csv.c`：

- 生成 `YYMMDD_D.CSV` 数据日志表头；
- 生成 `YYMMDD_E.CSV` 事件日志表头；
- 格式化数据行；
- 格式化错误行；
- 避免浮点 printf。

新增 `storage_queue.c`：

- 固定长度静态队列；
- 非阻塞入队；
- 队列满时丢弃并计数；
- 临界区内只做短复制。

### 2.4 Framework 接入

`px4lite_storage.c` 从 Framework Topic 读取数据，不读取 Business API，也不访问 Sensor 私有变量。

当前数据来源：

- Navigation Topic：GNSS 有效标志、经纬度、roll、pitch；
- Baro Topic：温度、气压、湿度；
- Battery Topic：电压、电量；
- 缺失数据写入错误记录。

## 3. 调试记录

调试中发现 Storage 任务如果优先级过高，可能与显示任务抢占，导致触摸和页面刷新受影响。因此当前将 Storage 任务优先级设置为 `tskIDLE_PRIORITY`，低于 `biz_display`。

已做处理：

- Storage 服务周期 50 ms；
- 数据记录周期 1000 ms；
- 文件 sync 周期 5000 ms；
- mount 失败后 2000 ms 退避；
- 写入失败后关闭文件并进入 DEGRADED；
- recover 回调只设置 remount 请求，不在 Health 任务里做 SD I/O。

## 4. 当前结果

当前 SD-CSV 模块已完成：

- SPI BSP；
- FatFS diskio 适配；
- SD mount/open/write/sync 封装；
- CSV 数据和错误格式化；
- 固定长度记录队列；
- Framework Storage 模块；
- Storage 模块状态发布；
- 数据文件 `YYMMDD_D.CSV`；
- 事件文件 `YYMMDD_E.CSV`。

模块状态可按“基础完成”理解：软件链路和隔离策略已建立，仍需结合真实 SD 卡做插拔、掉电和长时间写入测试。

## 5. 后续注意

- SD 卡写入异常不能影响传感器采集。
- 不允许在 Sensor、Estimator、Health、Debug 中直接调用 FatFS。
- 如果界面卡在 LOGO 或触摸不灵，优先检查 Storage 任务优先级和 SD 阻塞时间。
- 长时间测试需要观察 `drop_count`、`error_count`、`written_count` 和 storage 栈余量。
- 量产或正式测试前应补充断电保护和更严格的文件同步策略。
