/**
 * @file storage_csv.h
 * @brief SD 日志 CSV 行格式化辅助接口。
 *
 * @details
 * 本文件只负责把已经复制好的业务快照字段格式化为固定 CSV 文本，不访问 SD、
 * FatFs、Framework topic 或驱动私有变量。
 */

#ifndef STORAGE_CSV_H
#define STORAGE_CSV_H

#include <stddef.h>
#include <stdint.h>
#include "px4lite_types.h"
#include "storage_config.h"

/**
 * @brief 常规数据 CSV 行输入字段。
 */
typedef struct
{
    uint32_t time_ms;            /**< 记录时间，单位：ms。 */
    uint8_t gnss_valid;          /**< GNSS 有效标志，1 表示位置字段可用。 */
    int32_t latitude_e7;         /**< 纬度，单位：degree * 1e7。 */
    int32_t longitude_e7;        /**< 经度，单位：degree * 1e7。 */
    int32_t roll_deg100;         /**< 横滚角，单位：degree * 100。 */
    int32_t pitch_deg100;        /**< 俯仰角，单位：degree * 100。 */
    int32_t temperature_c100;    /**< 温度，单位：摄氏度 * 100。 */
    uint32_t pressure_hpa100;    /**< 气压，单位：hPa * 100。 */
    uint32_t humidity_pct100;    /**< 相对湿度，单位：% * 100。 */
    uint32_t voltage_mv;         /**< 电压，单位：mV。 */
    uint8_t battery_pct;         /**< 电量百分比，范围：0 到 100。 */
} Storage_CsvData_t;

/**
 * @brief 获取常规数据 CSV 表头。
 *
 * @return 静态表头字符串，不需要释放。
 */
const char *StorageCsv_DataHeader(void);

/**
 * @brief 获取错误记录 CSV 表头。
 *
 * @return 静态表头字符串，不需要释放。
 */
const char *StorageCsv_ErrorHeader(void);

/**
 * @brief 格式化一行常规数据 CSV。
 *
 * @param[in] data 输入数据，不能为 NULL。
 * @param[out] line 输出行缓冲区，不能为 NULL。
 * @param[in] line_size 输出缓冲区长度，单位：byte。
 *
 * @return 格式化结果。
 */
Px4Lite_Result_t StorageCsv_FormatDataLine(
    const Storage_CsvData_t *data,
    char *line,
    size_t line_size);

/**
 * @brief 格式化一行错误记录 CSV。
 *
 * @param[in] time_ms 记录时间，单位：ms。
 * @param[in] module 模块名称字符串，不能为 NULL。
 * @param[in] state 模块状态值。
 * @param[in] fault 故障码。
 * @param[in] error_count 累计错误次数。
 * @param[in] message 错误说明字符串，不能为 NULL。
 * @param[out] line 输出行缓冲区，不能为 NULL。
 * @param[in] line_size 输出缓冲区长度，单位：byte。
 *
 * @return 格式化结果。
 */
Px4Lite_Result_t StorageCsv_FormatErrorLine(
    uint32_t time_ms,
    const char *module,
    uint32_t state,
    uint32_t fault,
    uint32_t error_count,
    const char *message,
    char *line,
    size_t line_size);

#endif
