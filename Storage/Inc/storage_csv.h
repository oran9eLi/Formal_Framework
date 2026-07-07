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
typedef struct {
  uint32_t time_ms;                /**< 记录时间，单位：ms。 */
  uint32_t local_date_ymd;         /**< 本地日期，编码 YYYYMMDD；未知时为 0。 */
  uint32_t local_time_hhmmss;      /**< 本地时间，编码 HHMMSS；未知时为 0。 */
  uint8_t time_sync_state;         /**< 时间同步状态，见 `Px4Lite_TimeSyncState_t`。 */
  uint8_t gnss_valid;              /**< GNSS 有效标志，1 表示位置字段可用。 */
  int32_t latitude_e7;             /**< 纬度，单位：degree * 1e7。 */
  int32_t longitude_e7;            /**< 经度，单位：degree * 1e7。 */
  int32_t roll_deg100;             /**< 横滚角，单位：degree * 100。 */
  int32_t pitch_deg100;            /**< 俯仰角，单位：degree * 100。 */
  int32_t yaw_deg100;              /**< 相对偏航角，单位：degree * 100。 */
  int32_t temperature_c100;        /**< 温度，单位：摄氏度 * 100。 */
  uint32_t pressure_hpa100;        /**< 气压，单位：hPa * 100。 */
  uint32_t humidity_pct100;        /**< 相对湿度，单位：% * 100。 */
  uint32_t voltage_mv;             /**< 电压，单位：mV。 */
  int32_t current_ma;              /**< 第一电池电流，单位：mA。 */
  uint32_t power_mw;               /**< 第一电池功率，单位：mW。 */
  uint8_t battery_pct;             /**< 电量百分比，范围：0 到 100。 */
  uint8_t low_voltage;             /**< 第一电池低电压标志，0 表示正常，1 表示低电压。 */
  uint32_t voltage2_mv;            /**< 第二电池电压，单位：mV。 */
  int32_t current2_ma;             /**< 第二电池电流，单位：mA。 */
  uint32_t power2_mw;              /**< 第二电池功率，单位：mW。 */
  uint8_t battery2_pct;            /**< 第二电池电量百分比，范围：0 到 100。 */
  uint8_t low_voltage2;            /**< 第二电池低电压标志，0 表示正常，1 表示低电压。 */
  uint8_t motor_pct[PX4LITE_MOTOR_COUNT];            /**< 四路电机目标油门百分比，范围：0 到 100。 */
  uint8_t motor_run_state;         /**< 电机运行状态，1 表示允许输出目标油门。 */
  uint16_t active_alarm_count;     /**< 当前活动告警数量。 */
  uint16_t highest_fault_code;     /**< 当前最高严重度告警故障码。 */
  uint32_t lora_rx_count;          /**< LoRa/MAVLink 已接收完整帧数量。 */
  uint32_t lora_tx_count;          /**< LoRa/MAVLink DMA 发送完成帧数量。 */
  uint32_t lora_parse_error_count; /**< LoRa/MAVLink 接收解析错误累计次数。 */
  uint32_t lora_send_error_count;  /**< LoRa 底层发送错误累计次数。 */
  uint16_t storage_queue_count;    /**< Storage 记录队列当前占用数量。 */
  uint32_t storage_drop_count;     /**< Storage 记录队列累计丢弃次数。 */
} Storage_CsvData_t;

/**
 * @brief 事件 CSV 行输入字段。
 */
typedef struct {
  uint32_t time_ms;           /**< 记录时间，单位：ms。 */
  uint32_t local_date_ymd;    /**< 本地日期，编码：YYYYMMDD。 */
  uint32_t local_time_hhmmss; /**< 本地时间，编码：HHMMSS。 */
  const char *event_type;     /**< 事件类型，如 ALARM_ACTIVE、ALARM_CLEAR。 */
  const char *source;         /**< 事件来源模块。 */
  uint32_t state;             /**< 来源状态值。 */
  uint32_t fault;             /**< 故障码。 */
  uint32_t severity;          /**< 严重度或保留等级。 */
  uint8_t active;             /**< 1 表示触发，0 表示清除。 */
  uint32_t count;             /**< 同类事件累计次数。 */
  const char *message;        /**< 事件说明。 */
} Storage_CsvEvent_t;

/**
 * @brief 获取常规数据 CSV 表头。
 *
 * @return 静态表头字符串，不需要释放。
 */
const char *StorageCsv_DataHeader(void);

/**
 * @brief 获取事件 CSV 表头。
 *
 * @return 静态表头字符串，不需要释放。
 */
const char *StorageCsv_EventHeader(void);

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
Px4Lite_Result_t StorageCsv_FormatDataLine(const Storage_CsvData_t *data, char *line, size_t line_size);

/**
 * @brief 格式化一行事件 CSV。
 *
 * @param[in] event 输入事件，不能为 NULL。
 * @param[out] line 输出行缓冲区，不能为 NULL。
 * @param[in] line_size 输出缓冲区长度，单位：byte。
 *
 * @return 格式化结果。
 */
Px4Lite_Result_t StorageCsv_FormatEventLine(const Storage_CsvEvent_t *event, char *line, size_t line_size);

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
Px4Lite_Result_t StorageCsv_FormatErrorLine(uint32_t time_ms, const char *module, uint32_t state, uint32_t fault, uint32_t error_count, const char *message, char *line, size_t line_size);

#endif
