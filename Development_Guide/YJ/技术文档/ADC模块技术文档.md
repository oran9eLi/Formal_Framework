# 电源模块技术文档（ADC 电池检测）

## 文档元信息

| 项目 | 内容 |
|---|---|
| 模块名称 | 电源模块 —— ADC 电池检测（电池 1 + 电池 2，电压 / 电流 / 功率 / 电量）—— 样板 A |
| 对应设计书章节 | 《低空 CNS 平台产品设计书 V1.0》§3.2.6 电源/环境感知模块（电源部分） |
| 接入范式 | 样板 A：ADC 寄存器读/接收类（只收，Latest Snapshot） |
| 参考资料 | 本工程源码 `Bsp/Src/bsp_adc.c`、`bsp_adc2.c`、`bsp_adc_current.c`、`Sensor/Src/sensor_power.c`、`sensor_power2.c`、`Sensor/Inc/sensor_power_config.h` |
| 文档版本 | V2.0 |
| 作者 / 日期 | YJ / 2026-07-14 |
| 模块状态 | 已跑通（见 [07_移植进度与功能清单.md](../../07_移植进度与功能清单.md)） |

---

## 1. 模块定位

电源模块采集电池电压和电流，换算出电量百分比、输出功率和低压标志，模拟无人机的"电量/油量表"能力（设计书 §3.2.6）。模块接入统一的 `sensor` 采集任务，不单独创建 FreeRTOS 任务。

本工程支持**两块独立电池**：电池 1（主电池，`Sensor_Power`）和电池 2（第二/动力电池，`Sensor_Power2`），两者结构完全并列，各自有独立的 ADC 通道、电量曲线和低压阈值，共用同一套校准/滤波算法框架和 `Power_Snapshot_t` 数据类型。

分层职责的边界很清晰，值得先点明，避免后续维护者找错地方改：

- **BSP 层**只做"原始 ADC → 引脚电压"这一步硬件换算（含 VREFINT 实测 VDDA、浮空检测、多次平均、外部分压系数），**不解释**电流计比例、零点或电池状态。
- **Sensor 层**做全部"业务语义"计算：二次校准、电量曲线插值、电压/电流滤波、电量档位滞回与连续确认、低压滞回判定、电流换算与功率计算。

正式数据链路：

```text
电池 1：飞控电流计 VOLT 分压 -> PA5 / ADC1_IN5 -> BSP_ADC          ┐
        电流计模拟输出        -> PC0 / ADC3_IN10 -> BSP_ADC_Current ┤-> Sensor_Power  -> Battery Topic  ┐
电池 2：分压电压             -> PA4 / ADC2_IN4  -> BSP_ADC2         ┐                                    ├-> Display / MAVLink / Storage / Business
        电流计模拟输出        -> PC1 / ADC3_IN11 -> BSP_ADC_Current ┤-> Sensor_Power2 -> Battery2 Topic ┘
```

---

## 2. 硬件与总线

| 信号 | ADC 外设 | 通道 | 引脚 | 分压/换算 |
|---|---|---|---|---|
| 电池 1 电压 | ADC1 | IN5 | PA5 | 外部分压 `10080/1000` |
| 电池 2 电压 | ADC2 | IN4 | PA4 | 外部分压 `10080/1000`（与电池 1 同规格） |
| 电池 1 电流 | ADC3 | IN10 | PC0 | 电流计比例（见 §4.3） |
| 电池 2 电流 | ADC3 | IN11 | PC1 | 电流计比例（见 §4.3） |
| VDDA 基准 | ADC1 | VREFINT（内部） | — | 出厂校准值实测 VDDA，供三路 ADC 共用 |

| 项目 | 配置 |
|---|---|
| ADC 分辨率 | 12 bit（原始码 0~4095） |
| 参考电压 | 用 VREFINT 实测 VDDA（标称 3300 mV，仅在校准值无效时兜底，见 §4.1） |
| 电压平均次数 | 16 次（`BSP_ADC_AVERAGE_COUNT`） |
| 电流平均次数 | 16 次（`BSP_ADC_CURRENT_AVERAGE_COUNT`） |
| 采样时间 | 480 cycles（长采样：高阻分压充分建立 + VREFINT 要求） |
| 采集周期 | 1000 ms（`sensor` 任务节拍） |
| 所属任务 | `sensor` |

**硬件注意事项**：

- PA5 / PA4 只能输入 0~3.3V，**不能**直接接 10~12V 电池，必须接电流计 `VOLT` 这类已经分压后的电压信号。
- 电池同时给电流计和开发板供电时，读数比空载电池电压低约 0.2V 属正常范围（线材、接头、负载和共地压降造成的工作状态压降）。校准应以开发板工作时电源输入口处电压为基准，不要直接改软件分压系数（见 §11 案例 4）。
- 三路电流/电压 ADC 分属 ADC1/ADC2/ADC3 独立外设，互不冲突。

---

## 3. 软件分层与依赖

| 层 | 文件 | 职责 |
|---|---|---|
| BSP | `Bsp/Src/bsp_adc.c` | ADC1 电池 1 电压：初始化、VREFINT 实测 VDDA、浮空检测、预置放电、16 次平均、原始码→引脚电压→分压电池电压 |
| BSP | `Bsp/Src/bsp_adc2.c` | ADC2 电池 2 电压：与电池 1 同规格 |
| BSP | `Bsp/Src/bsp_adc_current.c` | ADC3 两路电流计引脚电压：通道切换、浮空检测、预置放电、16 次平均、原始码→引脚电压（**不解释电流比例**） |
| Sensor | `Sensor/Src/sensor_power.c` | 电池 1：二次校准、12 点电量曲线、电压滤波、电量档位滞回+连续确认、低压滞回确认、电流换算/滤波、功率 |
| Sensor | `Sensor/Src/sensor_power2.c` | 电池 2：同上，使用 `POWER2_*` 独立曲线与阈值 |
| Sensor 配置 | `Sensor/Inc/sensor_power_config.h` | 校准增益/偏移/负载补偿、双电池电量曲线、低压阈值、滤波、电流标定等全部宏 |
| Framework | Battery / Battery2 Topic | 发布电池快照，升 ONLINE/DEGRADED，Health 判 OFFLINE |
| Business | `Business/Inc/app_data_api.h` | 对业务/显示提供只读电池快照 |

依赖单向：`Sensor → BSP`。Sensor 层不 include `stm32f4xx_hal.h`、不调 `HAL_*`，所有 ADC 访问经 `BSP_ADC_*` 完成。

---

## 4. 协议 / 数据格式（本节是复现本模块的核心，给真实代码）

### 4.1 电压换算（BSP 层，`bsp_adc.c`）

原始 ADC 码先换算成 PA5 引脚电压，再乘外部分压系数得到电池电压。关键点是**引脚电压用 VREFINT 实测的 VDDA 做基准**，而不是写死的 3.3V——这样能消除电机负载导致的 VDDA 下陷误差（见 §11 案例 1）：

```c
/* VDDA = 3300mV * 出厂校准 VREFINT / 实测 VREFINT 原始码；校准值无效时退回标称 3300mV */
vdda   = BSP_ADC_GetVddaMv();
pin_mv = (raw * vdda + (4095 / 2)) / 4095;                       /* 引脚电压 */
voltage_mv = pin_mv * BSP_ADC_DIVIDER_NUM / BSP_ADC_DIVIDER_DEN; /* × 10080/1000 得电池电压 */
```

**浮空检测**：未接电池时采集脚浮空，会读出漏电流/干扰造成的假电压。`BSP_ADC_ReadVoltageMv` 采样前先做浮空判定，判为浮空则直接返回 0V：

```c
/* 预置拉低采一次、预置拉高采一次：真实低阻分压两次都被拉回真值→差≈噪声；
   浮空脚跟随预置电平→两读相差巨大。差值超过阈值(约 300 码≈0.24V)即判浮空。 */
if (BSP_ADC_PinFloating() != 0U) { *voltage_mv = 0U; return BSP_STATUS_OK; }
```

### 4.2 电量百分比（Sensor 层，12 点曲线）

电池电压按一条 12 点标定曲线分段线性插值，再按 5% 步进四舍五入。电池 1 与电池 2 各用一张独立曲线：

**电池 1（`POWER_PERCENT_TABLE_*`）**：

| 电压 (mV) | 9900 | 10200 | 10500 | 10800 | 11100 | 11400 | 11700 | 11950 | 12150 | 12300 | 12450 | 12550 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 电量 (%) | 0 | 5 | 10 | 20 | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 100 |

**电池 2（`POWER2_PERCENT_TABLE_*`）**：

| 电压 (mV) | 10500 | 10550 | 10650 | 10800 | 10950 | 11100 | 11300 | 11500 | 11750 | 12000 | 12300 | 12600 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 电量 (%) | 0 | 5 | 10 | 20 | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 100 |

低于曲线最低点记 0%，高于最高点记 100%；曲线是非线性的（电池放电特性使然，见 §11 案例 5）。插值后按 `POWER_PERCENT_STEP`（5%）四舍五入到档位。

### 4.3 电流换算（BSP 引脚电压 → Sensor 电流）

BSP 只给出电流计模拟脚的引脚电压，Sensor 层用零点电压和灵敏度换算成电流，并做零点死区：

```c
delta_mv   = adc_mv - POWER_CURRENT_ZERO_MV;                       /* 电流计 0A 输出电压 */
current_ma = delta_mv * 1000 / POWER_CURRENT_MV_PER_A + POWER_CURRENT_OFFSET_MA;
if (|current_ma| <= POWER_CURRENT_DEADBAND_MA) { current_ma = 0; } /* 零点死区，抑制 0A 附近抖动 */
```

换算后再做一阶低通滤波（`3/4` 旧值 + `1/4` 新值）。电池 1、电池 2 各有独立的 `POWER_CURRENT_*` / `POWER2_CURRENT_*` 标定参数（当前为按实物标定后的值）。

### 4.4 功率

```c
power_mw = (current_ma > 0) ? (voltage_mv * current_ma / 1000) : 0;  /* 只在放电(正电流)时计功率 */
```

### 4.5 快照结构 `Power_Snapshot_t`

电池 1、电池 2 复用同一数据类型（[sensor_power.h](../../../Sensor/Inc/sensor_power.h)）：

| 字段 | 单位 | 说明 |
|---|---|---|
| `rx_sequence` | 次 | 成功采样递增序号（回绕时跳过 0），读侧去重用 |
| `sample_time_ms` | ms | 采样完成时间 |
| `voltage_v` | V | 电池电压（滤波前的本次校准值转 V） |
| `current_ma` | mA | 电流；电流计未接入或采样失败时为 0 |
| `power_mw` | mW | 功率，由电压和电流计算 |
| `percent` | % | 电量百分比，0~100，按 5% 档位 |
| `low_voltage` | 0/1 | 低电压标志 |
| `error_count` | 次 | 累计错误次数（ADC 读失败等，不清零） |

---

## 5. 采样 / 处理流程

`Sensor_Power_Service(now_ms)`（`sensor` 任务每 1000 ms 调用，电池 2 的 `Sensor_Power2_Service` 完全同构）：

1. 若有重初始化请求，先 `Init()` 复位内部滤波/确认状态。
2. `BSP_ADC_ReadVoltageMv()`：浮空检测（未接→返回 0V）→ VREFINT 实测 VDDA → 16 次平均 → 引脚电压 → 分压电池电压。读失败则 `error_count++`，本次返回 `IO_ERROR`。
3. `Power_ApplyCalibration()` 二次校准（增益/偏移/负载补偿，默认 `1:1` / `0mV` / 关闭，不改变读数）。
4. `rx_sequence++`，写 `voltage_v`。
5. 电压一阶低通滤波（`3/4` 旧 + `1/4` 新）。
6. 电量曲线插值得候选档位；**首帧直接采用**，之后必须越过 `200mV` 滞回边界、且**连续 10 次**得到同一新档位才真正换档（`Power_ApplyPercentConfirm`），避免临界点来回跳。
7. 低压判定：滞回（低于 `10.3V` 进入、高于 `10.5V` 解除，中间保持）+ 连续确认。
8. 电流：`BSP_ADC_Current_ReadVoltageMv(BATTERY1)` → 换算 → 滤波 → 功率 `= V × I`。电流读失败则 `error_count++`、电流/功率记 0，但电压/电量仍照常产出。

> 电池 2 流程相同，读 ADC2（电压）+ ADC3 IN11（电流），使用 `POWER2_*` 曲线（低压 `10.8V` 进 / `11.0V` 出）。

---

## 6. 接口清单

驱动接口（四件套 + 重初始化请求，电池 1 / 电池 2 各一套）：

```c
Power_Result_t Sensor_Power_Init(void);
Power_Result_t Sensor_Power_Service(uint32_t now_ms);
Power_Result_t Sensor_Power_CopySnapshot(Power_Snapshot_t *out);
Power_Result_t Sensor_Power_GetStatus(Power_Status_t *out);   /* 不访问 ADC，只读缓存 */
void           Sensor_Power_RequestReinit(void);              /* 只置位，供 recovery 回调 */

Power_Result_t Sensor_Power2_Init(void);
Power_Result_t Sensor_Power2_Service(uint32_t now_ms);
Power_Result_t Sensor_Power2_CopySnapshot(Power_Snapshot_t *out);
Power_Result_t Sensor_Power2_GetStatus(Power_Status_t *out);
void           Sensor_Power2_RequestReinit(void);
```

业务读取经 `app_data_api.h` 的电池快照接口，不下探 Sensor 层。

---

## 7. 配置项

**BSP 通道/引脚（[bsp_config.h](../../../Bsp/Inc/bsp_config.h)）**：

| 关键项 | 值 | 说明 |
|---|---|---|
| `BSP_ADC_CH` / `BSP_ADC_PIN` | `ADC_CHANNEL_5` / PA5 | 电池 1 电压（ADC1） |
| `BSP_ADC2_CH` / `BSP_ADC2_PIN` | `ADC_CHANNEL_4` / PA4 | 电池 2 电压（ADC2） |
| `BSP_ADC_CURRENT1_CH` / `_PIN` | `ADC_CHANNEL_10` / PC0 | 电池 1 电流（ADC3） |
| `BSP_ADC_CURRENT2_CH` / `_PIN` | `ADC_CHANNEL_11` / PC1 | 电池 2 电流（ADC3） |
| `BSP_ADC_DIVIDER_NUM/DEN` | `10080/1000` | 外部分压系数（两块电池相同） |

**BSP 采样参数（`bsp_adc.c` / `bsp_adc_current.c` 内部宏）**：

| 关键项 | 值 | 说明 |
|---|---|---|
| `BSP_ADC_AVERAGE_COUNT` | `16` | 电池电压通道平均次数 |
| `BSP_ADC_VREF_COUNT` | `8` | VREFINT 平均次数 |
| `BSP_ADC_CURRENT_AVERAGE_COUNT` | `16` | 电流通道平均次数 |
| `BSP_ADC_SAMPLETIME` / `BSP_ADC_CURRENT_SAMPLE_TIME` | `480 cycles` | 长采样时间 |
| `BSP_ADC_FLOAT_DELTA_RAW` | `300`（≈0.24V） | 浮空判定阈值 |
| `BSP_ADC_REF_MV` | `3300` | VREFINT 校准无效时的兜底 VDDA |

**Sensor 校准/曲线/阈值（[sensor_power_config.h](../../../Sensor/Inc/sensor_power_config.h)）**：

| 关键项 | 值 | 说明 |
|---|---|---|
| `POWER_CAL_GAIN_NUM/DEN` | `1000/1000` | 电压二次校准增益，默认 1.0 倍 |
| `POWER_CAL_OFFSET_MV` | `0` | 电压固定偏移（可负） |
| `POWER_LOAD_COMPENSATION_ENABLE` / `_MV` | `0` / `200` | 负载压降补偿，默认关闭 |
| `POWER_PERCENT_STEP` | `5` | 电量显示步进 |
| `POWER_PERCENT_CONFIRM_COUNT` | `10` | 换档连续确认次数 |
| `POWER_PERCENT_HYSTERESIS_MV` | `200` | 电量档位滞回电压 |
| `POWER_LOW_WARNING_MV` / `_RECOVER_MV` | `10300` / `10500` | 电池 1 低压进入/解除阈值 |
| `POWER2_LOW_WARNING_MV` / `_RECOVER_MV` | `10800` / `11000` | 电池 2 低压进入/解除阈值 |
| `POWER_CURRENT_ZERO_MV` / `_MV_PER_A` | `29` / `100` | 电池 1 电流计 0A 输出 / 灵敏度（按实物标定） |
| `POWER_CURRENT_DEADBAND_MA` | `50` | 电池 1 电流零点死区 |
| `POWER2_CURRENT_ZERO_MV` / `_MV_PER_A` | `240` / `100` | 电池 2 电流计标定 |
| `POWER2_CURRENT_DEADBAND_MA` | `200` | 电池 2 电流零点死区 |
| `POWER_CURRENT_FILTER_OLD_WEIGHT` / `_TOTAL` | `3` / `4` | 电流一阶低通（3/4 旧 + 1/4 新） |
| `POWER_FILTER_OLD_WEIGHT` / `_TOTAL` | `3` / `4` | 电压一阶低通（3/4 旧 + 1/4 新） |

> `POWER2_*` 中未单列的显示步进/确认次数/滞回，默认复用电池 1 的对应配置。

---

## 8. 异常与恢复

- 驱动只上报硬件事实与错误计数（`error_count`），不在驱动内判 OFFLINE/FAILED；OFFLINE 由 Health 统一按超时判定。
- `BSP_ADC_ReadVoltageMv()` 失败：`error_count++`，返回 `POWER_RESULT_IO_ERROR`，本次不产出快照。
- 未接电池（采集脚浮空）：判为浮空后电压记 **0V**，这是正常返回而非错误，不计 `error_count`。
- 电流读失败：电流/功率记 0、`error_count++`，但电压/电量/低压仍正常产出（电流是可选增强项，不拖累电压主链路）。
- VDDA 校准值无效或 VREFINT 读失败：退回标称 3300mV，不影响功能。
- `Sensor_Power_RequestReinit()` 只置请求标志，真正的 `Init()`（复位滤波/确认状态）在下一次 `Service()` 执行。

---

## 9. 调试

- **软件侧**：读 `Power_Snapshot_t` 的 `voltage_v`/`percent`/`current_ma`/`power_mw`/`low_voltage`/`error_count`/`rx_sequence`，确认按 1000ms 周期递增、数值合理。
- **硬件侧**：万用表量开发板电源输入口电压和 PA5 引脚电压，对照分压系数 `10080/1000` 核算；量电流计模拟输出脚电压，对照 `ZERO_MV`/`MV_PER_A` 核算电流。
- 若单独测试电压准确、同时供电偏低约 0.2V，优先查负载压降和共地，不直接改软件分压比例（见 §11 案例 4）。

---

## 10. 验收要点

| 用例 | 操作步骤 | 预期结果 | 实际结果 |
|---|---|---|---|
| 电压准确性 | 电流计 `VOLT → PA5`，对照电源输入口实测电压 | ADC 电压接近电池工作状态电压 | |
| 未接电池判 0 | 拔掉电池/断开采集脚 | 浮空检测生效，电压读 `0V`，不读出假电压 | |
| 电量档位稳定 | 缓慢改变电压跨越档位边界 | 电量按 5% 档位、越过 200mV 滞回且连续 10 次确认后才换档，临界点不来回跳 | |
| 低压阈值 | 电池 1 电压降到 10.3V 以下再升回 10.5V 以上 | 低于 10.3V 连续确认后置低压位，高于 10.5V 连续确认后清除，中间保持 | |
| 电流/功率 | 空载与加载对比 | 空载电流在死区内读≈0；加载后电流、功率随之变化 | |
| 双电池独立 | 只接一块电池 | 已接电池正常，未接电池电压判 0、互不影响 | |
| 负载压降误判 | 同供电偏低约 0.2V | 优先查负载/共地压降，不改软件系数 | |

---

## 11. 开发踩坑与教学案例

> 每条按 **现象 → 定位 → 根因 → 解法 → 教学点/学生易错** 组织。

**问题 1：用固定 3.3V 当 ADC 参考，电机负载下电压系统性偏差**（类别：软硬件结合）

- **现象**：静态测电压准，电机一转起来电压读数整体偏低一点，且随负载变化。
- **定位**：对照万用表实测——偏差不是固定值，而是跟着负载走，说明不是分压系数错，是参考电压本身在动。
- **根因**：ADC 换算默认拿标称 3.3V 当满量程基准，但 VDDA 会因大电流负载而下陷，基准一变，所有读数按比例偏。
- **解法**：用芯片出厂校准的 VREFINT 实测 VDDA（`VDDA = 3300 × VREFINT_CAL / VREFINT_raw`）代替写死的 3300，见 §4.1。
- **教学点 / 学生易错**：以为 ADC 参考电压是恒定 3.3V。参考电压随供电波动，精密测量必须用内部基准实测 VDDA 归一化。

**问题 2：未接电池/电流计时，引脚浮空读出假电压**（类别：硬件表现 → 软件防御）

- **现象**：不接电池时电压不是 0，而是一个飘忽的小数值。
- **定位**：万用表量该脚接近 0，但 ADC 读出几百 mV，说明是高阻悬空脚拾取干扰/漏电。
- **根因**：ADC 输入脚悬空时呈高阻态，会跟随周围干扰和漏电流停在不确定电位。
- **解法**：采样前把脚预置拉低放电；并做浮空检测（预置低采一次、预置高采一次，两读差值过大即判浮空，直接返回 0V），见 §4.1。
- **教学点 / 学生易错**：把 ADC 读到的任何数都当真值。悬空输入必须主动判浮空，否则"没接也有读数"。

**问题 3：电量在档位临界点来回跳变**（类别：软件）

- **现象**：电压在某个档位边界附近轻微波动时，显示电量在两档之间反复跳。
- **根因**：只按瞬时电压查表，边界附近的正常噪声就会让档位反复翻。
- **解法**：三重稳定——电压一阶低通滤波 + 200mV 滞回（升/降用不同边界）+ 连续 10 次确认才换档，见 §5。
- **教学点 / 学生易错**：阈值判定不加滞回/防抖。任何"过线就切换"的显示都要加滞回和连续确认。

**问题 4：把工作状态下的 0.2V 压降误当成软件比例错误**（类别：软硬件结合）

- **现象**：电池同时给电流计和开发板供电时，读数比空载电池电压低约 0.2V。
- **根因**：这不是测量错误，是线材、接头、负载和共地造成的真实工作压降。
- **解法**：校准以开发板工作时电源输入口电压为基准；确认是压降而非测量误差时，不改分压系数（真要补偿用 `POWER_LOAD_COMPENSATION_*`，默认关闭）。
- **教学点 / 学生易错**：一看到读数偏低就去改软件系数。要先分清"测量误差"和"真实工作压降"，后者改系数会把空载读数带偏。

**问题 5：电池电压非线性，单一线性换算电量不准**（类别：传感器特性）

- **现象**：用"满电压~空电压线性映射 0~100%"算出来的电量，中段明显和实际不符。
- **根因**：锂/动力电池放电曲线非线性，中间平台段电压变化慢、两端变化快。
- **解法**：用 12 点标定曲线分段线性插值（见 §4.2），两块电池各用一张按实物测的曲线。
- **教学点 / 学生易错**：把电池电压和电量当成线性关系。电量估算必须基于实测放电曲线。

---

## 12. 变更记录

| 日期 | 版本 | 作者 | 变更说明 | 关联记录 |
|---|---|---|---|---|
| 2026-XX-XX | V1.0 | YJ | 首版（简版，仅单电池电压，旧格式，未按 00 范本对齐） | — |
| 2026-07-14 | V2.0 | YJ | 按 [00_模块技术文档范本.md](00_模块技术文档范本.md) 样板 A 重写：补文档元信息、软件分层、采样流程、接口清单、配置项、踩坑教学；核对真实源码补全电流/功率、第二块电池、12 点电量曲线、电量档位滞回+连续确认、低压进入/解除阈值、VREFINT 实测 VDDA、浮空检测、二次校准配置层；更正旧文档"平均 8 次/固定 3.3V 参考/仅测电压/CURR 未接入"等已过时表述 | 本次代码审查 |
