# 2026-06-24_40 ADC 用 VREFINT 实测 VDDA 校正 + 加重滤波

## 一、背景

电压读数有约 0.1V 浮动。链路：12bit ADC、分压 10.08×、原换算用**写死的 Vref=3300mV**。0.1V 浮动主要来自：
1. 分压比把引脚噪声放大约 10 倍；
2. **VDDA 当成固定 3.3V，实际有纹波**——ADC 是相对 VDDA 的比例测量，VDDA 漂 ±0.5% 在 12V 上就是 ±60mV；
3. 高阻分压 + 采样时间不足、负载真实波动。

## 二、改动

### 1. 用内部 VREFINT 实测 VDDA（治第 2 项）

[bsp_adc.c](../../Bsp/Src/bsp_adc.c)：新增 `BSP_ADC_GetVddaMv()`——读内部 `ADC_CHANNEL_VREFINT`，用出厂校准值（`0x1FFF7A2A`，VDDA=3.3V 下测得）算实际 VDDA：

```
VDDA = 3300 * VREFINT_CAL / VREFINT_raw
```

`BSP_ADC_ReadVoltageMv` 改用实测 VDDA 代替常量 3300：`pin_mv = raw * VDDA / 4095`。读取失败/校准值无效时退回 3300mV，不影响功能。

VREFINT 仅接 **ADC1**，故由 bsp_adc(ADC1) 测量；VDDA 为 ADC1/ADC2 公共供电，[bsp_adc2.c](../../Bsp/Src/bsp_adc2.c)（电机电池）改为调用 `BSP_ADC_GetVddaMv()` 用同一 VDDA 换算。

### 2. 加重滤波 / 加长采样（治第 1、3 项）

- 采样时间 144 → **480 周期**（高阻分压充分建立，且 VREFINT 需要长采样）。
- 平均次数 8 → **16 次**（电池通道）；VREFINT 取 8 次平均。
- 新增 `BSP_ADC_ReadChannelAverage(channel, sampletime, count, *raw)` 统一在电池通道与 VREFINT 通道间切换采样；`ReadRaw/ReadAverage` 复用之，行为不变。

> sensor 层原有一阶 IIR + 滞回 + 10 次确认不变；显示层 ±0.2V 档位滞回（记录 39）也仍在——本次是从源头把读数本身做稳。

## 三、改动清单

| 文件 | 改动 |
|------|------|
| [bsp_adc.c](../../Bsp/Src/bsp_adc.c) / [bsp_adc.h](../../Bsp/Inc/bsp_adc.h) | VREFINT 实测 VDDA、`BSP_ADC_GetVddaMv`、通道平均助手、480 周期、16 次平均 |
| [bsp_adc2.c](../../Bsp/Src/bsp_adc2.c) | 复用 ADC1 的 VDDA、480 周期、16 次平均 |
| [test_power_adc_calibration.c](../../Tests/Unit/test_power_adc_calibration.c) | 电量预期值更新到新线性曲线(9.0~12.6V)，low_voltage 用例改跨 9.0V 边界（与记录 39 的曲线改动对齐） |

## 四、验证

- 静止电池电压读数浮动应明显变小（VDDA 纹波被抵消、采样更稳）。
- 电机一上电/负载变化时，电压不再因 VDDA 下陷整体漂移。
- 单元测试 `test_power_adc_calibration` 预期已与线性曲线对齐。
- 未编译验证（无 Keil/编译环境）。注意：VREFINT 首次读取有 ~10µs 基准建立时间，已靠 8 次平均 + 每拍重复读吸收；若首拍 VDDA 略偏，可在使能后加一次性延时。
