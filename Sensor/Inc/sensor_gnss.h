/**
 * @file sensor_gnss.h
 * @brief ATGM336H GNSS 强类型快照和驱动接口。
 *
 * @details
 * GNSS 驱动负责 NMEA 校验、拆句、解析和设备事实记录。驱动层不判定 Framework
 * OFFLINE/FAILED 状态，不直接发布 Framework topic；数据必须经 Platform Adapter
 * 转换后进入 Framework。
 */

#ifndef SENSOR_GNSS_H
#define SENSOR_GNSS_H

#include <stdint.h>

/**
 * @brief GNSS 数据事实状态。
 */
typedef enum {
  GNSS_DATA_NONE = 0, /**< 尚未解析到有效句子。 */
  GNSS_DATA_NO_FIX,   /**< 已收到 GNSS 数据，但当前无有效定位。 */
  GNSS_DATA_FIX_OK    /**< 当前定位有效。 */
} Gnss_DataState_t;

/**
 * @brief GNSS 驱动返回值。
 */
typedef enum {
  GNSS_RESULT_OK = 0,       /**< 操作成功。 */
  GNSS_RESULT_NO_DATA,      /**< 当前无新数据。 */
  GNSS_RESULT_NO_FIX,       /**< 有数据但无有效定位。 */
  GNSS_RESULT_INVALID_PARAM, /**< 参数非法。 */
  GNSS_RESULT_IO_ERROR      /**< UART/DMA 或 BSP 访问失败。 */
} Gnss_Result_t;

#define GNSS_VALID_POSITION (1UL << 0) /**< GGA 经纬度和定位质量字段有效。 */
#define GNSS_VALID_VELOCITY (1UL << 1) /**< RMC 地速和航迹角字段有效。 */
#define GNSS_VALID_TIME (1UL << 2) /**< RMC 完整 UTC 日期与时间有效。 */
#define GNSS_VALID_ALTITUDE (1UL << 3) /**< GGA 海拔高度字段有效。 */

/**
 * @brief GNSS 最新解析快照；记录测量事实，年龄策略由 Framework 消费者判断。
 */
typedef struct {
  Gnss_DataState_t data_state; /**< 数据事实状态；组件离线/失败归 Framework 判定。 */
  uint32_t valid_mask;          /**< 最新各测量字段有效位，使用 GNSS_VALID_*。 */
  uint32_t position_update_ms;  /**< 最近有效 GGA 位置时间，单位：ms。 */
  uint32_t velocity_update_ms;  /**< 最近有效 RMC 地速/航迹角时间，单位：ms。 */
  uint32_t time_update_ms;      /**< 最近有效 RMC 日期/时间接收时刻，单位：ms。 */
  uint32_t altitude_update_ms;  /**< 最近有效 GGA 海拔时间，单位：ms。 */

  uint8_t fix_valid;     /**< 定位有效标志，1 表示定位有效。 */
  uint8_t fix_quality;   /**< GGA 定位质量字段。 */
  uint8_t fix_dimension; /**< GSA 定位维度，0/2D/3D 按 NMEA 字段解析。 */

  uint8_t satellites; /**< 当前 GGA fix 使用的总卫星数。 */

  uint8_t gps_visible_sats; /**< GSV 报告的 GPS 可见卫星数。 */
  uint8_t bds_visible_sats; /**< GSV 报告的北斗可见卫星数。 */

  uint8_t gps_used_sats; /**< GSA 报告的 GPS 参与解算卫星数。 */
  uint8_t bds_used_sats; /**< GSA 报告的北斗参与解算卫星数。 */

  uint16_t hdop_cm;  /**< 水平精度因子，单位：HDOP * 100。 */
  uint32_t utc_sec;  /**< UTC 当日秒数，单位：s。 */
  uint32_t utc_date; /**< RMC 日期，压缩格式 yymmdd；无有效日期时为 0。 */

  int32_t latitude_deg_e7;  /**< 纬度，单位：degree * 1e7。 */
  int32_t longitude_deg_e7; /**< 经度，单位：degree * 1e7。 */
  int32_t altitude_mm;      /**< 海拔高度，单位：mm。 */

  uint32_t speed_cms;      /**< 地速，单位：cm/s。 */
  uint32_t heading_deg100; /**< 地面航向，单位：degree * 100。 */

  uint32_t rx_sequence; /**< 接收解析序号，每次有效解析推进。 */
  uint32_t last_rx_ms;  /**< 最近收到完整 NMEA 句子的时间，单位：ms。 */
  uint32_t last_fix_ms; /**< 最近有效定位时间，单位：ms。 */
} Gnss_Snapshot_t;

/**
 * @brief GNSS 轻量状态。
 *
 * @details
 * 状态查询不携带位置负载，不访问慢速总线，只复制驱动内部缓存。
 */
typedef struct {
  Gnss_DataState_t data_state; /**< 数据事实状态。 */
  uint8_t fix_valid;           /**< 定位有效标志。 */
  uint8_t satellites;          /**< 当前使用卫星数。 */
  uint32_t rx_sequence;        /**< 接收解析序号。 */
  uint32_t last_rx_ms;         /**< 最近收到完整 NMEA 句子的时间，单位：ms。 */
  uint32_t last_fix_ms;        /**< 最近有效定位时间，单位：ms。 */
} Gnss_Status_t;

/**
 * @brief 复位 GNSS 解析器和快照状态。
 *
 * @return 初始化结果。
 */
Gnss_Result_t Sensor_GNSS_Init(void);

/**
 * @brief 处理待解析 GNSS 输入并更新内部快照。
 *
 * @param[in] now_ms 当前 sensor 周期时间，单位：ms。
 *
 * @return 服务结果。
 *
 * @note 本函数由 sensor 任务调用，不在 ISR 中调用。
 */
Gnss_Result_t Sensor_GNSS_Service(uint32_t now_ms);

/**
 * @brief 复制最近一次由 `Sensor_GNSS_Service()` 生成的完整 GNSS 快照。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 */
Gnss_Result_t Sensor_GNSS_CopySnapshot(Gnss_Snapshot_t *out);

/**
 * @brief 复制不含位置负载的 GNSS 轻量状态。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 *
 * @note 本函数不访问 UART/DMA，只复制缓存。
 */
Gnss_Result_t Sensor_GNSS_GetStatus(Gnss_Status_t *out);

/**
 * @brief 请求 GNSS 解析器和快照在下一次 Service 中重新初始化。
 *
 * @note 本函数只置位请求标志，适合 recovery 回调调用。
 */
void Sensor_GNSS_RequestReinit(void);

/** @brief 完整原始句的只读诊断回调；type为0其他/1GGA/2RMC/3GSA/4GSV，不能阻塞。 */
typedef void (*Gnss_DiagnosticSentence_t)(const uint8_t *bytes, uint16_t length, uint8_t type, uint8_t checksum_valid, uint32_t now_ms);
/** @brief 调度前由Adapter登记；Driver不依赖Framework。 */
void Gnss_SetDiagnosticCallback(Gnss_DiagnosticSentence_t callback);
#endif
