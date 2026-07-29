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
  uint32_t command_count;          /**< COMMAND_LONG 已调度次数。 */
  uint32_t command_ack_tx_count;   /**< COMMAND_ACK 已发送次数。 */
  uint32_t command_ack_rx_count;   /**< COMMAND_ACK 已接收次数。 */
  uint32_t no_data_count;          /**< 因无可用 topic 数据跳过发送的次数。 */
  uint32_t stale_count;            /**< 因 topic 数据过期跳过发送的次数。 */
  uint32_t busy_count;             /**< 因 LoRa 发送忙跳过发送的次数。 */
  uint32_t error_count;            /**< 编码或发送提交错误次数。 */
  /* GPS/详情/姿态/电池/气压的去重序号已按发送目标(LoRa/RPi)拆到 px4lite_mavlink_tx.c
     内的 s_last_*_sequence[2]，不再放本共享结构，避免两链路互相误判"已发"而漏帧。 */
  uint32_t last_alarm_sequence;    /**< 最近发送告警对应的 topic sequence(仅 LoRa 出口用)。 */
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
 * @brief 调度树莓派专属出口(LORASTAT/RIDSTAT/告警表/日志表)一帧。
 *
 * @param[in] now_ms 当前系统时间，单位 ms。
 *
 * @retval PX4LITE_OK 本周期提交了一帧 RPi 专属数据。
 * @retval PX4LITE_IDLE 无到期 RPi 专属数据。
 *
 * @note 与 Px4Lite_MavlinkTxRun 分离：只写 USART6，不依赖 LoRa 服务状态或半双工
 * 空闲，须由 comm 任务无条件周期调用，保证 LoRa 忙/掉线时 RPi 全量出口不断流。
 * PX4LITE_ENABLE_RPI_MAVLINK 关闭时为空操作。
 */
Px4Lite_Result_t Px4Lite_MavlinkTxRunRpi(uint32_t now_ms);

/**
 * @brief 复制当前 MAVLink 发送统计。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @note 本函数只复制内存统计，不访问 LoRa 硬件。
 */
void Px4Lite_MavlinkTxGetStats(Px4Lite_MavlinkTxStats_t *out);

Px4Lite_Result_t Px4Lite_MavlinkStartRemoteView(uint8_t target_node_id, uint32_t now_ms);
Px4Lite_Result_t Px4Lite_MavlinkStopRemoteView(uint8_t target_node_id, uint32_t now_ms);
Px4Lite_Result_t Px4Lite_MavlinkAcceptRemoteViewRequest(uint8_t requester_node_id, uint32_t lease_ms, uint32_t now_ms);
void Px4Lite_MavlinkAcceptRemoteViewStop(uint8_t requester_node_id, uint32_t now_ms);
void Px4Lite_MavlinkSetTxEnabled(uint8_t enabled);
uint8_t Px4Lite_MavlinkGetTxEnabled(void);
/**
 * @brief 排队一条 COMMAND_ACK。
 *
 * @param[in] command ACK 对应的 MAVLink 命令号。
 * @param[in] result MAVLink ACK 结果枚举值。
 * @param[in] target_system ACK 目标 system id，取命令来源 sysid。
 * @param[in] target_component ACK 目标 component id，取命令来源 compid。
 * @param[in] link ACK 出口链路，取收到命令的链路。
 */
void Px4Lite_MavlinkQueueCommandAck(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component, Px4Lite_MavlinkLink_t link);
void Px4Lite_MavlinkRecordCommandAck(uint16_t command, uint8_t result, uint32_t now_ms);

/* 前向声明 MAVLink 消息结构标签，避免在本头引入 common/mavlink.h(保持与 types.h 同层)。 */
struct __mavlink_message;

/**
 * @brief 把一帧已编码的 MAVLink 消息镜像到树莓派 USART1(compid 193, RPi 独立连续序号)。
 *
 * @details
 * 供 RemoteID 发送器把 OPEN_DRONE_ID_* 身份帧同步给树莓派链路，复用 RPi 侧现成的
 * OpenDroneID 解码。函数临时改写 msg 的 compid/seq/checksum 并在返回前恢复，不影响
 * 调用方对该帧的后续使用；只写 USART1，不触碰 LoRa/COMM_0 通道序号。
 * 仅在 PX4LITE_ENABLE_RPI_MAVLINK 为真时存在实体。
 */
Px4Lite_Result_t Px4Lite_MavlinkTxMirrorToRpi(struct __mavlink_message *msg);

#endif
