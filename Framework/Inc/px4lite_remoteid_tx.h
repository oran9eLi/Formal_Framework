/**
 * @file px4lite_remoteid_tx.h
 * @brief 声明 STM32 到 ESP32-S3 的 RemoteID MAVLink 发送调度接口。
 *
 * @details
 * 本模块属于 Framework 通信层，只读取 Framework topic、逐字段编码 OpenDroneID 消息，
 * 并通过 Platform Adapter 提交到 UART4。不得包含 BSP/HAL 头文件。
 */

#ifndef PX4LITE_REMOTEID_TX_H
#define PX4LITE_REMOTEID_TX_H

#include <stdint.h>
#include "px4lite_types.h"

/**
 * @brief RemoteID MAVLink 发送统计。
 */
typedef struct {
  uint32_t heartbeat_count;     /**< HEARTBEAT 成功提交次数。 */
  uint32_t basic_id_count;      /**< OPEN_DRONE_ID_BASIC_ID 成功提交次数。 */
  uint32_t location_count;      /**< OPEN_DRONE_ID_LOCATION 成功提交次数。 */
  uint32_t system_count;        /**< OPEN_DRONE_ID_SYSTEM 成功提交次数。 */
  uint32_t operator_id_count;   /**< OPEN_DRONE_ID_OPERATOR_ID 成功提交次数。 */
  uint32_t self_id_count;       /**< OPEN_DRONE_ID_SELF_ID 成功提交次数。 */
  uint32_t no_data_count;       /**< 因导航 topic 暂不可用而跳过次数。 */
  uint32_t stale_count;         /**< 因导航 topic 过期而跳过次数。 */
  uint32_t busy_count;          /**< UART4 DMA 忙导致重试次数。 */
  uint32_t error_count;         /**< 编码或底层提交失败次数。 */
  uint32_t last_message_id;     /**< 最近成功提交的 MAVLink message id。 */
  uint32_t last_location_seq;   /**< 最近发送位置对应的 Navigation sequence。 */
} Px4Lite_RemoteIdTxStats_t;

/**
 * @brief 初始化 RemoteID 发送调度器。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 */
Px4Lite_Result_t Px4Lite_RemoteIdTxInit(uint32_t now_ms);

/**
 * @brief 执行一次 RemoteID 发送调度，单次最多提交一帧。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 */
Px4Lite_Result_t Px4Lite_RemoteIdTxRun(uint32_t now_ms);

/**
 * @brief 复制 RemoteID 发送统计。
 */
void Px4Lite_RemoteIdTxGetStats(Px4Lite_RemoteIdTxStats_t *out);

#endif
