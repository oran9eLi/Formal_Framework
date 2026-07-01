/**
 * @file px4lite_mavlink_tx.h
 * @brief Framework 快照到 MAVLink 遥测帧的编码和发送调度接口。
 *
 * @details
 * 本模块从 Framework topic 读取快照，按低速 LoRa 链路预算调度 MAVLink 消息。
 * 通信层禁止直接发送 C 结构体内存，所有 MAVLink 消息必须逐字段编码。
 */

#ifndef PX4LITE_MAVLINK_TX_H
#define PX4LITE_MAVLINK_TX_H

#include <stdint.h>
#include "px4lite_types.h"

/**
 * @brief MAVLink 遥测发送统计。
 */
typedef struct {
  uint32_t heartbeat_count;        /**< HEARTBEAT 已调度次数。 */
  uint32_t gps_raw_count;          /**< GPS_RAW_INT 已调度次数。 */
  uint32_t gnss_detail_count;      /**< GNSS 详情或状态文本已调度次数。 */
  uint32_t attitude_count;         /**< ATTITUDE 已调度次数。 */
  uint32_t position_count;         /**< GLOBAL_POSITION_INT 已调度次数。 */
  uint32_t sys_status_count;       /**< SYS_STATUS 已调度次数。 */
  uint32_t module_state_count;     /**< MODSTAT0/MODSTAT1 NAMED_VALUE_INT 已调度次数。 */
  uint32_t battery_status_count;   /**< BATTERY_STATUS 已调度次数。 */
  uint32_t scaled_pressure_count;  /**< SCALED_PRESSURE 已调度次数。 */
  uint32_t statustext_count;       /**< STATUSTEXT 已调度次数。 */
  uint32_t remote_status_count;    /**< REMOTE_STATUS 扩展帧已调度次数。 */
  uint32_t remote_motor_count;     /**< REMOTE_MOTOR 扩展帧已调度次数。 */
  uint32_t remote_alarm_count;     /**< REMOTE_ALARM TUNNEL 已调度次数。 */
  uint32_t remote_log_count;       /**< REMOTE_LOG TUNNEL 已调度次数。 */
  uint32_t command_count;          /**< COMMAND_LONG 已调度次数。 */
  uint32_t command_ack_tx_count;   /**< COMMAND_ACK 已发送次数。 */
  uint32_t command_ack_rx_count;   /**< COMMAND_ACK 已接收次数。 */
  uint32_t no_data_count;          /**< 因无可用 topic 数据跳过发送的次数。 */
  uint32_t stale_count;            /**< 因 topic 数据过期跳过发送的次数。 */
  uint32_t busy_count;             /**< 因 LoRa 发送忙跳过发送的次数。 */
  uint32_t error_count;            /**< 编码或发送提交错误次数。 */
  uint32_t last_gps_sequence;      /**< 最近发送 GPS 数据对应的 topic sequence。 */
  uint32_t last_detail_sequence;   /**< 最近发送 GNSS 详情对应的 topic sequence。 */
  uint32_t last_attitude_sequence; /**< 最近发送姿态对应的 topic sequence。 */
  uint32_t last_battery_sequence;  /**< 最近发送电池对应的 topic sequence。 */
  uint32_t last_pressure_sequence; /**< 最近发送气压对应的 topic sequence。 */
  uint32_t last_alarm_sequence;    /**< 最近发送告警对应的 topic sequence。 */
  uint32_t last_message_id;        /**< 最近调度的 MAVLink message id。 */
} Px4Lite_MavlinkTxStats_t;

/**
 * @brief 初始化 MAVLink 遥测调度器。
 *
 * @param[in] now_ms 当前系统毫秒时间，用于初始化发送周期 deadline。
 *
 * @return 初始化结果。
 * @retval PX4LITE_OK 初始化成功。
 */
Px4Lite_Result_t Px4Lite_MavlinkTxInit(uint32_t now_ms);

/**
 * @brief 执行一次遥测调度周期，最多提交一帧。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 调度结果。
 * @retval PX4LITE_OK 已成功提交一帧。
 * @retval PX4LITE_BUSY LoRa 仍有上一帧在发送。
 * @retval PX4LITE_IO_ERROR 编码或发送提交失败。
 * @retval PX4LITE_IDLE 本周期没有到期且可发送的消息。
 *
 * @note 本函数由 comm 任务周期调用，不在 ISR 中调用。topic 暂无数据或过期时
 * 会更新统计并尝试下一个调度 slot，通常不会直接把 NOT_READY/STALE 返回给调用方。
 */
Px4Lite_Result_t Px4Lite_MavlinkTxRun(uint32_t now_ms);

/**
 * @brief 复制当前 MAVLink 发送统计。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @note 本函数只复制内存统计，不访问 LoRa 硬件。
 */
void Px4Lite_MavlinkTxGetStats(Px4Lite_MavlinkTxStats_t *out);

Px4Lite_Result_t Px4Lite_MavlinkSetRemoteView(uint8_t enabled, uint8_t target_node_id, uint32_t now_ms);
uint8_t Px4Lite_MavlinkRemoteViewExpired(uint32_t now_ms);
uint8_t Px4Lite_MavlinkShouldAcceptFullFrom(uint8_t source_node_id, uint32_t now_ms);
void Px4Lite_MavlinkHandleLoRaSummary(uint8_t source_node_id, uint8_t active_viewer_node_id, uint8_t lease_id, uint16_t remaining_s, uint32_t now_ms);
void Px4Lite_MavlinkQueueCommandAck(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component);
void Px4Lite_MavlinkRecordCommandAck(uint16_t command, uint8_t result);
Px4Lite_Result_t Px4Lite_MavlinkApplyStreamControl(uint8_t requester_node_id, uint8_t action, uint32_t stream_mask, uint32_t lease_ms, uint32_t now_ms);

#endif
