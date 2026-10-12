/**
 * @file px4lite_remoteid_tx.h
 * @brief 声明 STM32 到 ESP32-S3 的 RemoteID MAVLink 发送调度接口。
 *
 * @details
 * 本模块属于 Framework 通信层，只读取 Framework topic、逐字段编码 OpenDroneID 消息，
 * 并通过 Platform Adapter 提交到 UART4。不得包含 BSP/HAL 头文件。
 * LOCATION 使用有效导航 TIME 域的小时内 UTC 秒；SYSTEM 使用统一 UTC 的 2019 年基准秒。
 * 各导航字段独立检查有效位；速度/航迹遵守 MAVLink OpenDroneID 单位、未知值与限幅。
 */

#ifndef PX4LITE_REMOTEID_TX_H
#define PX4LITE_REMOTEID_TX_H

#include <stdint.h>
#include "px4lite_types.h"

/**
 * @brief RemoteID MAVLink 发送统计。
 * @details error_count 按本次上电累计；其他字段统计本轮初始化之后的发送事实。
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
  uint32_t error_count;         /**< 本次上电累计编码/提交失败次数；热恢复保留，按 uint32_t 自然回绕。 */
  uint32_t last_message_id;     /**< 最近成功提交的 MAVLink message id。 */
  uint32_t last_location_seq;   /**< 最近发送位置对应的 Navigation sequence。 */
  uint32_t last_success_ms;      /**< 最近一次成功提交到 UART4 DMA 的时间，单位 ms。 */
  uint32_t last_busy_ms;         /**< 最近一次发现 UART4 DMA 忙的时间，单位 ms。 */
  uint32_t last_error_ms;        /**< 最近一次编码或底层提交失败的时间，单位 ms。 */
} Px4Lite_RemoteIdTxStats_t;

/**
 * @brief 初始化 RemoteID 发送调度器。
 * @details 由 CommTask 调用；热恢复保留 error_count，其他统计及发送时序重置。
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
 * @brief 向树莓派 USART6 发送 OPEN_DRONE_ID_BASIC_ID，1Hz，与 ESP32 在位无关。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @note 必须在 ESP32 在位判定之外无条件调用：树莓派靠 BASIC_ID.uas_id 提取 vendor_id
 *       建 MQTT 客户端，该身份通路不得依赖 RemoteID 广播模块是否插着。ESP32 出口的
 *       BASIC_ID 仍由 Px4Lite_RemoteIdTxRun 负责，两者载荷一致、互不重复。
 */
Px4Lite_Result_t Px4Lite_RemoteIdTxRunRpiIdentity(uint32_t now_ms);

/**
 * @brief 复制 RemoteID 发送统计。
 * @details 调用者须与调度/初始化同属 CommTask；跨任务消费者读取已发布的模块状态。
 */
void Px4Lite_RemoteIdTxGetStats(Px4Lite_RemoteIdTxStats_t *out);

#endif
