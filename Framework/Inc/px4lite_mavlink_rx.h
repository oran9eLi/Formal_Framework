/**
 * @file px4lite_mavlink_rx.h
 * @brief LoRa 接收侧 MAVLink 消息分发接口。
 *
 * @details
 * 本模块属于 Framework 通信层，由 CommTask 调用。它只负责解码 LoRa 驱动已经识别
 * 的 MAVLink payload，并写入 RemoteTelemetry；不做 Display 渲染、模式切换或控制执行。
 */

#ifndef PX4LITE_MAVLINK_RX_H
#define PX4LITE_MAVLINK_RX_H

#include <stdint.h>
#include "px4lite_types.h"
#include "px4lite_platform.h"

/**
 * @brief MAVLink RX 统计。
 */
typedef struct {
  uint32_t handled_count;     /**< 已处理并写入远端快照的消息数量。 */
  uint32_t filtered_count;    /**< 因模式或 system id 不匹配被过滤的消息数量。 */
  uint32_t unsupported_count; /**< 当前不支持的 MAVLink 消息数量。 */
  uint32_t invalid_count;     /**< 输入参数或 payload 长度非法的消息数量。 */
  uint32_t last_msg_id;       /**< 最近处理的 MAVLink message id。 */
  uint8_t last_sysid;         /**< 最近处理的 MAVLink system id。 */
  uint8_t last_compid;        /**< 最近处理的 MAVLink component id。 */
  uint16_t rx_loss_permille;  /**< 接收链路丢包率，靠 MAVLink seq 跳变推断，单位：‰(0~1000)。 */
} Px4Lite_MavlinkRxStats_t;

/**
 * @brief 初始化 MAVLink RX 分发统计。
 *
 * @param[in] now_ms 当前系统时间，单位：ms。
 */
void Px4Lite_MavlinkRxInit(uint32_t now_ms);

/**
 * @brief 处理一帧 LoRa 驱动输出的 MAVLink 消息。
 *
 * @param[in] frame LoRa 接收帧，不能为 NULL。
 * @param[in] now_ms 当前系统时间，单位：ms。
 *
 * @return 处理结果。
 */
Px4Lite_Result_t Px4Lite_MavlinkRxHandleFrame(const Px4Lite_LoRaRxFrame_t *frame, uint32_t now_ms);

/**
 * @brief 复制 MAVLink RX 统计。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 */
void Px4Lite_MavlinkRxGetStats(Px4Lite_MavlinkRxStats_t *out);

#endif


