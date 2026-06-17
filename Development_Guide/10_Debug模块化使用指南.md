# Debug 模块化使用指南

## 1. 设计目标

Debug 使用并列功能开关，不设置“总开关 + 子开关”的人工配置层级。

- 堆栈监控独立运行，报告所有已注册应用任务及 Heap 使用情况。
- GNSS、IMU、气压计、LoRa、存储等模块分别独立启用。
- 可以同时打开堆栈监控和一个被测模块，观察该模块运行时的资源变化。
- 周期打印由低优先级 `DebugTask` 执行，不占用 Sensor、Health 或业务任务。

## 2. 配置位置

文件：

```text
Debug/Inc/debug_config.h
```

当前独立开关：

```c
#define DEBUG_BOOT_LOG_ENABLE              1U
#define DEBUG_BUSINESS_LOG_ENABLE          0U

#define DEBUG_STACK_MONITOR_ENABLE         1U
#define DEBUG_GNSS_MONITOR_ENABLE          1U
#define DEBUG_GNSS_BSP_MONITOR_ENABLE      0U

#define DEBUG_IMU_MONITOR_ENABLE           1U
#define DEBUG_BARO_MONITOR_ENABLE          1U
#define DEBUG_POWER_MONITOR_ENABLE         1U
#define DEBUG_LORA_MONITOR_ENABLE          1U
#define DEBUG_ALARM_MONITOR_ENABLE         1U
#define DEBUG_STORAGE_MONITOR_ENABLE       0U
#define DEBUG_HEALTH_MONITOR_ENABLE        0U
```

`DEBUG_PERIODIC_SERVICE_ENABLE` 和 `DEBUG_CONSOLE_ENABLE` 是内部推导结果，
开发人员不得手动修改。

## 3. 常用组合

### 3.1 只观察所有任务资源

```c
#define DEBUG_STACK_MONITOR_ENABLE         1U
#define DEBUG_GNSS_MONITOR_ENABLE          0U
```

周期输出 Heap 和所有已通过 `DebugTaskMonitor_Register()` 注册的任务。

### 3.2 GNSS 功能与资源联合测试

```c
#define DEBUG_STACK_MONITOR_ENABLE         1U
#define DEBUG_GNSS_MONITOR_ENABLE          1U
```

同时输出：

- 全局 Heap 使用量。
- 所有已注册任务的栈峰值和最小剩余。
- GNSS 定位、星数、经纬度、状态和数据年龄。

### 3.3 只观察 GNSS 数据

```c
#define DEBUG_STACK_MONITOR_ENABLE         0U
#define DEBUG_GNSS_MONITOR_ENABLE          1U
```

此时仍会创建 `DebugTask`，但不打印任务栈和 Heap。

### 3.4 LoRa/MAVLink 链路统计

```c
#define DEBUG_LORA_MONITOR_ENABLE          1U
```

输出 LoRa 模块状态、收发帧数、本地坏帧/丢弃率、发送失败率、MAVLink
已发送消息计数和最近收发年龄。`rx_loss` 当前表示本机可观测的
`CRC/parse/drop/overflow` 统计比例，不等同于真实空口丢包率；真实空口
丢包率需要对端序号或 ACK 回传后再统计。

### 3.5 IMU 姿态和控制输入观察

```c
#define DEBUG_IMU_MONITOR_ENABLE           1U
```

输出 MPU6050 换算后的加速度、角速度、温度，以及 Estimator 生成的
`roll/pitch/yaw` 和 `roll/pitch/yaw rate`。姿态角单位为 cdeg，角速度单位为
cdps，后续 PID、MAVLink 和屏幕均读取同一份 Navigation/App 快照。

### 3.6 环境、电源和告警观察

```c
#define DEBUG_BARO_MONITOR_ENABLE          1U
#define DEBUG_POWER_MONITOR_ENABLE         1U
#define DEBUG_ALARM_MONITOR_ENABLE         1U
```

BARO 输出气压、温度、气压高度和垂直速度。POWER 输出电压、电流、百分比和
低压标志。ALARM 输出活动告警数量、最高告警来源模块、故障码和严重等级。

## 4. 新模块接入规则

以 IMU 为例：

1. 在 `debug_config.h` 增加或启用 `DEBUG_IMU_MONITOR_ENABLE`。
2. 在 `debug_service.c` 中实现只读 `DebugService_ReportImu()`。
3. 调试函数只能复制正式 Topic、状态快照和统计计数。
4. 不得在调试函数中推进采集、清空 FIFO 或修改模块状态。
5. 独立设置输出周期，禁止每个采样周期都打印。
6. 新建任务后必须调用 `DebugTaskMonitor_Register()`。

## 5. 强制约束

- ISR 中禁止串口打印，只允许更新计数器和寄存器快照。
- 正式任务中禁止放置周期调试循环。
- 关闭模块宏后，该模块不得增加运行时处理和输出。
- 堆栈监控关闭后，注册接口保持空操作，不影响任务创建。
- `DebugTask` 使用最低业务优先级，不得加入看门狗必需心跳集合。
- 调试代码不得成为传感器正式数据链路的一部分。
- RAW 调试必须使用独立宏，例如 `DEBUG_GNSS_BSP_MONITOR_ENABLE`，默认关闭。
- 屏幕、LoRa/MAVLink、Alarm、Logger 和 Debug 必须读取同一条 App/Topic 正式链路。

## 6. 当前输出解释

```text
MEM: heap_cfg=40960 heap_free=23048 heap_min=23048 heap_peak_used=17912
STACK: task=sensor cfg=384W peak=240W min_free=144W/576B OK
ATT: roll_cdeg=12 pitch_cdeg=-8 yaw_cdeg=35 rate_cdps=2,0,-1
BARO: state=2 pressure_pa=100864 temp_cC=2631 alt_mm=384 vs_cms=0 age=12
POWER: state=2 voltage_mv=12042 current_ma=0 percent=96 low=0 age=10
LORA: state=2 rx=0 tx=42 rx_loss=0.0% tx_fail=0.0% busy=0 crc=0 parse=0 drop=0 ovf=0 mav=12/12/12/40 stale=0 nodata=0 age_rx=0 age_tx=18 msg=0
ALARM: count=1 highest_src=1 fault=0x0202 sev=2
```

- `cfg`：创建任务时配置的栈深度，单位为 word。
- `peak`：历史最大栈使用量。
- `min_free`：历史最小剩余栈空间。
- `heap_min`：系统启动以来最小剩余 Heap。
- `heap_peak_used`：系统启动以来最大 Heap 使用量。
- `rx_loss`：本地可观测坏帧/丢弃比例，分母为有效接收帧加本地错误计数。
- `tx_fail`：本地发送忙、发送错误和 MAVLink 编码/发送错误比例。
- `mav=a/b/c/d`：分别为 heartbeat、GPS_RAW、GNSS detail、ATTITUDE 发送计数。
- `highest_src`：最高告警来源模块 ID，`fault` 为框架故障码。

F407 的一个 `StackType_t` 为 4 字节，因此 `144W = 576B`。
