# BME280 与 Power ADC 强类型迁移记录

## 1. 迁移目标

本次将 `projext` 工程中的 BME280 环境传感器和 Power ADC 电源采样能力迁入 N1 工程，但不迁移旧的 `Sensor_Driver_t`、`Sensor_Sample_t` 通用通道模型。

迁移后的正式链路为：

```text
Bsp I2C / ADC
  -> Sensor BME280 / Power
  -> Platform Adapter
  -> sensor task
  -> Baro / Battery Topic
  -> Health
  -> App_CopyEnvironment()
```

## 2. 新增文件

| 文件 | 职责 |
|---|---|
| `Bsp/Inc/bsp_status.h` | BSP 通用返回码 |
| `Bsp/Inc/bsp_time.h` / `Bsp/Src/bsp_time.c` | 板级毫秒时间和短延时包装 |
| `Bsp/Inc/bsp_i2c.h` / `Bsp/Src/bsp_i2c.c` | I2C1 8 位寄存器读写和设备探测 |
| `Bsp/Inc/bsp_adc.h` / `Bsp/Src/bsp_adc.c` | ADC1_IN5 电源分压采样和电压 mV 换算 |
| `Sensor/Inc/sensor_bme280.h` / `Sensor/Src/sensor_bme280.c` | BME280 强类型快照驱动 |
| `Sensor/Inc/sensor_power.h` / `Sensor/Src/sensor_power.c` | Power ADC 强类型快照驱动 |
| `Tests/Unit/test_sensor_environment.c` | BME280 补偿和 Power ADC 百分比单元测试 |

## 3. 修改文件

| 文件 | 修改内容 |
|---|---|
| `Bsp/Inc/bsp_config.h` | 增加 I2C1 PB6/PB7、ADC1_IN5 PA5 和分压比例配置 |
| `Core/Src/stm32f4xx_hal_msp.c` | 增加 I2C1 与 ADC1 MSP Init/DeInit |
| `Framework/Inc/px4lite_config.h` | 启用 `PX4LITE_ENABLE_BARO`、`PX4LITE_ENABLE_BATTERY`，增加周期和超时 |
| `Framework/Inc/px4lite_types.h` | Baro Topic 增加 `relative_humidity_x100` |
| `Framework/Inc/px4lite_platform.h` | 增加 Baro/Battery 平台适配接口 |
| `Framework/Src/px4lite_platform_f407.c` | 将 BME280/Power 快照转换为 Framework Measurement |
| `Framework/Src/px4lite_modules.c` | 在现有 `sensor` 任务中按 1000 ms 周期采集 Baro/Battery，并纳入 Health |
| `Business/Inc/app_data_api.h` / `Business/Src/app_data_api.c` | 增加 `App_CopyEnvironment()` |
| `MDK-ARM/formal_framework.uvprojx` | 增加新源文件和 HAL I2C/ADC 源文件 |

## 4. 数据模型

### BME280

驱动输出 `Bme280_Snapshot_t`：

- `temperature_c`：摄氏度，`float`。
- `pressure_hpa`：百帕，`float`。
- `relative_humidity_pct`：相对湿度百分比，`float`。
- `sample_time_ms`：采样发布时间。
- `rx_sequence`：每次完整采样递增。

Framework 发布 `Px4Lite_SensorBaro_t`，保留气压、温度和湿度字段。

### Power ADC

驱动输出 `Power_Snapshot_t`：

- `voltage_v`：输入端电压 V，`float`。
- `percent`：当前线性估算电量百分比。
- `low_voltage`：低电压标志。
- `sample_time_ms`：采样发布时间。
- `rx_sequence`：每次完整采样递增。

Framework 发布 `Px4Lite_BatteryStatus_t`。

## 5. 任务与周期

没有新增 FreeRTOS 任务。

| 模块 | 所属任务 | 周期 |
|---|---|---:|
| BME280 | `sensor` | 1000 ms |
| Power ADC | `sensor` | 1000 ms |

这样符合 N1 规范：普通低频传感器由统一采集任务持有，不为单个传感器创建永久任务。

## 6. 验证

已执行：

```text
gcc -std=c99 -Wall -Wextra -I Bsp\Inc -I Sensor\Inc -I Framework\Inc Tests\Unit\test_sensor_environment.c Sensor\Src\sensor_power.c Sensor\Src\sensor_bme280.c -o Tests\Unit\test_sensor_environment.exe
.\Tests\Unit\test_sensor_environment.exe
```

结果：

```text
sensor environment tests passed
```

已额外执行：

```text
gcc -std=c99 -DSTM32F407xx -Wall -Wextra -fsyntax-only ... Bsp\Src\bsp_i2c.c Bsp\Src\bsp_adc.c Bsp\Src\bsp_time.c
gcc -std=c99 -Wall -Wextra -fsyntax-only -I Bsp\Inc -I Sensor\Inc Sensor\Src\sensor_power.c Sensor\Src\sensor_bme280.c
```

BSP/Sensor 文件语法检查通过。BSP 检查中出现的 warning 来自 STM32 HAL/CMSIS 头在 64 位主机 GCC 下的指针宽度差异，不是本次业务代码告警。

当前环境未找到 `UV4` 或 `armcc` 命令行，因此未在本机完成 Keil ARMCC5 全量构建。Keil 工程文件已经加入新增源文件，后续应在 MDK 中执行一次全量 `0 Error / 0 Warning` 构建确认。
