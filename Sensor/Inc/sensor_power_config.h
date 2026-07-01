/**
 * @file sensor_power_config.h
 * @brief 电池电压校准、电量曲线和显示稳定性配置。
 *
 * @details
 * 本文件属于 Sensor Driver 配置层，用于集中管理电池 ADC 的现场标定参数。
 * 默认配置不改变 BSP_ADC 输出电压；如更换电流计、分压比例或现场确认存在稳定负载压降，
 * 可通过覆盖本文件宏定义完成校准，而不需要改动 `sensor_power.c` 的算法代码。
 */

#ifndef SENSOR_POWER_CONFIG_H
#define SENSOR_POWER_CONFIG_H

#ifndef POWER_CAL_GAIN_NUM
/** @brief 电压校准增益分子，默认 1000，表示 1.000 倍。 */
#define POWER_CAL_GAIN_NUM 1000U
#endif

#ifndef POWER_CAL_GAIN_DEN
/** @brief 电压校准增益分母，默认 1000，必须非 0。 */
#define POWER_CAL_GAIN_DEN 1000U
#endif

#ifndef POWER_CAL_OFFSET_MV
/** @brief 电压校准固定偏移，单位：mV，可为负值。 */
#define POWER_CAL_OFFSET_MV 0
#endif

#ifndef POWER_LOAD_COMPENSATION_ENABLE
/** @brief 是否启用负载压降补偿；0 表示关闭，1 表示开启。 */
#define POWER_LOAD_COMPENSATION_ENABLE 0U
#endif

#ifndef POWER_LOAD_COMPENSATION_MV
/** @brief 负载压降补偿值，单位：mV，默认按现场观测的约 0.2V 预留。 */
#define POWER_LOAD_COMPENSATION_MV 200U
#endif

#ifndef POWER_CURRENT_ZERO_MV
/** @brief 第一电池电流计 0A 输出电压，单位：mV，需按实物标定。 */
#define POWER_CURRENT_ZERO_MV 0U
#endif

#ifndef POWER_CURRENT_MV_PER_A
/** @brief 第一电池电流计每 1A 对应的输出电压变化，单位：mV/A，需按实物标定。 */
#define POWER_CURRENT_MV_PER_A 100U
#endif

#ifndef POWER_CURRENT_OFFSET_MA
/** @brief 第一电池电流二次校准固定偏移，单位：mA，可为负值。 */
#define POWER_CURRENT_OFFSET_MA 0
#endif

#ifndef POWER_CURRENT_DEADBAND_MA
/** @brief 第一电池电流零点死区，单位：mA，小于该值时按 0A 处理。 */
#define POWER_CURRENT_DEADBAND_MA 50U
#endif

#ifndef POWER_CURRENT_FILTER_OLD_WEIGHT
/** @brief 第一电池电流滤波旧值权重。 */
#define POWER_CURRENT_FILTER_OLD_WEIGHT 3U
#endif

#ifndef POWER_CURRENT_FILTER_TOTAL
/** @brief 第一电池电流滤波总权重，必须非 0。 */
#define POWER_CURRENT_FILTER_TOTAL 4U
#endif

#ifndef POWER2_CURRENT_ZERO_MV
/** @brief 第二电池电流计 0A 输出电压，单位：mV，需按实物标定。 */
#define POWER2_CURRENT_ZERO_MV 0U
#endif

#ifndef POWER2_CURRENT_MV_PER_A
/** @brief 第二电池电流计每 1A 对应的输出电压变化，单位：mV/A，需按实物标定。 */
#define POWER2_CURRENT_MV_PER_A 100U
#endif

#ifndef POWER2_CURRENT_OFFSET_MA
/** @brief 第二电池电流二次校准固定偏移，单位：mA，可为负值。 */
#define POWER2_CURRENT_OFFSET_MA 0
#endif

#ifndef POWER2_CURRENT_DEADBAND_MA
/** @brief 第二电池电流零点死区，单位：mA，小于该值时按 0A 处理。 */
#define POWER2_CURRENT_DEADBAND_MA 200U
#endif

#ifndef POWER2_CURRENT_FILTER_OLD_WEIGHT
/** @brief 第二电池电流滤波旧值权重。 */
#define POWER2_CURRENT_FILTER_OLD_WEIGHT 3U
#endif

#ifndef POWER2_CURRENT_FILTER_TOTAL
/** @brief 第二电池电流滤波总权重，必须非 0。 */
#define POWER2_CURRENT_FILTER_TOTAL 4U
#endif

#ifndef POWER_PERCENT_TABLE_EMPTY_MV
/** @brief 0% 电量曲线点电压，单位：mV。 */
#define POWER_PERCENT_TABLE_EMPTY_MV 9900U
#endif

#ifndef POWER_PERCENT_TABLE_5_MV
/** @brief 5% 电量曲线点电压，单位：mV。 */
#define POWER_PERCENT_TABLE_5_MV 10200U
#endif

#ifndef POWER_PERCENT_TABLE_10_MV
/** @brief 10% 电量曲线点电压，单位：mV。 */
#define POWER_PERCENT_TABLE_10_MV 10500U
#endif

#ifndef POWER_PERCENT_TABLE_20_MV
/** @brief 20% 电量曲线点电压，单位：mV。 */
#define POWER_PERCENT_TABLE_20_MV 10800U
#endif

#ifndef POWER_PERCENT_TABLE_30_MV
/** @brief 30% 电量曲线点电压，单位：mV。 */
#define POWER_PERCENT_TABLE_30_MV 11100U
#endif

#ifndef POWER_PERCENT_TABLE_40_MV
/** @brief 40% 电量曲线点电压，单位：mV。 */
#define POWER_PERCENT_TABLE_40_MV 11400U
#endif

#ifndef POWER_PERCENT_TABLE_50_MV
/** @brief 50% 电量曲线点电压，单位：mV。 */
#define POWER_PERCENT_TABLE_50_MV 11700U
#endif

#ifndef POWER_PERCENT_TABLE_60_MV
/** @brief 60% 电量曲线点电压，单位：mV。 */
#define POWER_PERCENT_TABLE_60_MV 11950U
#endif

#ifndef POWER_PERCENT_TABLE_70_MV
/** @brief 70% 电量曲线点电压，单位：mV。 */
#define POWER_PERCENT_TABLE_70_MV 12150U
#endif

#ifndef POWER_PERCENT_TABLE_80_MV
/** @brief 80% 电量曲线点电压，单位：mV。 */
#define POWER_PERCENT_TABLE_80_MV 12300U
#endif

#ifndef POWER_PERCENT_TABLE_90_MV
/** @brief 90% 电量曲线点电压，单位：mV。 */
#define POWER_PERCENT_TABLE_90_MV 12450U
#endif

#ifndef POWER_PERCENT_TABLE_FULL_MV
/** @brief 100% 电量曲线点电压，单位：mV。 */
#define POWER_PERCENT_TABLE_FULL_MV 12550U
#endif

#ifndef POWER_LOW_WARNING_MV
/** @brief 第一电池低压预警进入阈值，滤波电压低于该值并连续确认后置位，单位：mV。 */
#define POWER_LOW_WARNING_MV 10300U
#endif

#ifndef POWER_LOW_RECOVER_MV
/** @brief 第一电池低压预警解除阈值，滤波电压高于该值并连续确认后清除，单位：mV。 */
#define POWER_LOW_RECOVER_MV 10500U
#endif

#ifndef POWER_LOW_CONFIRM_COUNT
/** @brief 第一电池低压预警连续确认次数，默认复用电量档位确认次数。 */
#define POWER_LOW_CONFIRM_COUNT POWER_PERCENT_CONFIRM_COUNT
#endif

#ifndef POWER2_PERCENT_TABLE_EMPTY_MV
/** @brief 第二电池 0% 电量曲线点电压，单位：mV。 */
#define POWER2_PERCENT_TABLE_EMPTY_MV 10500U
#endif

#ifndef POWER2_PERCENT_TABLE_5_MV
/** @brief 第二电池 5% 电量曲线点电压，单位：mV。 */
#define POWER2_PERCENT_TABLE_5_MV 10550U
#endif

#ifndef POWER2_PERCENT_TABLE_10_MV
/** @brief 第二电池 10% 电量曲线点电压，单位：mV。 */
#define POWER2_PERCENT_TABLE_10_MV 10650U
#endif

#ifndef POWER2_PERCENT_TABLE_20_MV
/** @brief 第二电池 20% 电量曲线点电压，单位：mV。 */
#define POWER2_PERCENT_TABLE_20_MV 10800U
#endif

#ifndef POWER2_PERCENT_TABLE_30_MV
/** @brief 第二电池 30% 电量曲线点电压，单位：mV。 */
#define POWER2_PERCENT_TABLE_30_MV 10950U
#endif

#ifndef POWER2_PERCENT_TABLE_40_MV
/** @brief 第二电池 40% 电量曲线点电压，单位：mV。 */
#define POWER2_PERCENT_TABLE_40_MV 11100U
#endif

#ifndef POWER2_PERCENT_TABLE_50_MV
/** @brief 第二电池 50% 电量曲线点电压，单位：mV。 */
#define POWER2_PERCENT_TABLE_50_MV 11300U
#endif

#ifndef POWER2_PERCENT_TABLE_60_MV
/** @brief 第二电池 60% 电量曲线点电压，单位：mV。 */
#define POWER2_PERCENT_TABLE_60_MV 11500U
#endif

#ifndef POWER2_PERCENT_TABLE_70_MV
/** @brief 第二电池 70% 电量曲线点电压，单位：mV。 */
#define POWER2_PERCENT_TABLE_70_MV 11750U
#endif

#ifndef POWER2_PERCENT_TABLE_80_MV
/** @brief 第二电池 80% 电量曲线点电压，单位：mV。 */
#define POWER2_PERCENT_TABLE_80_MV 12000U
#endif

#ifndef POWER2_PERCENT_TABLE_90_MV
/** @brief 第二电池 90% 电量曲线点电压，单位：mV。 */
#define POWER2_PERCENT_TABLE_90_MV 12300U
#endif

#ifndef POWER2_PERCENT_TABLE_FULL_MV
/** @brief 第二电池 100% 电量曲线点电压，单位：mV。 */
#define POWER2_PERCENT_TABLE_FULL_MV 12600U
#endif

#ifndef POWER2_LOW_WARNING_MV
/** @brief 第二电池低压预警进入阈值，滤波电压低于该值并连续确认后置位，单位：mV。 */
#define POWER2_LOW_WARNING_MV 10800U
#endif

#ifndef POWER2_LOW_RECOVER_MV
/** @brief 第二电池低压预警解除阈值，滤波电压高于该值并连续确认后清除，单位：mV。 */
#define POWER2_LOW_RECOVER_MV 11000U
#endif

#ifndef POWER2_PERCENT_STEP
/** @brief 第二电池电量显示步进，默认复用第一电池 5% 档位。 */
#define POWER2_PERCENT_STEP POWER_PERCENT_STEP
#endif

#ifndef POWER2_PERCENT_CONFIRM_COUNT
/** @brief 第二电池电量档位连续确认次数，默认复用第一电池配置。 */
#define POWER2_PERCENT_CONFIRM_COUNT POWER_PERCENT_CONFIRM_COUNT
#endif

#ifndef POWER2_PERCENT_HYSTERESIS_MV
/** @brief 第二电池电量档位迟滞电压，默认复用第一电池配置，单位：mV。 */
#define POWER2_PERCENT_HYSTERESIS_MV POWER_PERCENT_HYSTERESIS_MV
#endif

#ifndef POWER2_LOW_CONFIRM_COUNT
/** @brief 第二电池低压预警连续确认次数，默认复用电量档位确认次数。 */
#define POWER2_LOW_CONFIRM_COUNT POWER_PERCENT_CONFIRM_COUNT
#endif

#ifndef POWER_FILTER_OLD_WEIGHT
/** @brief 电压滤波旧值权重，默认 3。 */
#define POWER_FILTER_OLD_WEIGHT 3U
#endif

#ifndef POWER_FILTER_TOTAL
/** @brief 电压滤波总权重，默认 4，对应 3/4 旧值 + 1/4 新值。 */
#define POWER_FILTER_TOTAL 4U
#endif

#ifndef POWER_PERCENT_STEP
/** @brief 电量显示步进，单位：%，默认每 5% 一个档位。 */
#define POWER_PERCENT_STEP 5U
#endif

#ifndef POWER_PERCENT_CONFIRM_COUNT
/** @brief 新电量档位连续确认次数，用于抑制临界点来回跳变。 */
#define POWER_PERCENT_CONFIRM_COUNT 10U
#endif

#ifndef POWER_PERCENT_HYSTERESIS_MV
/** @brief 电量档位滞回电压，单位：mV。 */
#define POWER_PERCENT_HYSTERESIS_MV 200U
#endif

#endif
