/**
 * @file px4lite_mavlink_rx.h
 * @brief Framework 层 MAVLink 接收解码和远端遥测快照接口。
 *
 * @details
 * 本模块只消费通信层提供的 MAVLink 帧事实，逐字段解码为远端遥测快照。
 * LoRa 驱动不解释业务语义，Business/Display 不直接读取 LoRa RX 帧。
 */

#ifndef PX4LITE_MAVLINK_RX_H
#define PX4LITE_MAVLINK_RX_H

#include <stdint.h>
#include "px4lite_types.h"

/**
 * @brief 执行一次 MAVLink RX 解码调度。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 解码结果。
 */
Px4Lite_Result_t Px4Lite_MavlinkRxRun(uint32_t now_ms);

/**
 * @brief 复制最近一份远端遥测快照。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 */
Px4Lite_Result_t Px4Lite_CopyRemoteTelemetry(Px4Lite_RemoteTelemetry_t *out);

/**
 * @brief 选择 Display/Business 默认读取的远端节点。
 *
 * @param[in] node_id 远端节点 ID，主机为 0，从机从 1 开始。
 *
 * @return 选择结果。
 */
Px4Lite_Result_t Px4Lite_SelectRemoteNode(uint8_t node_id);

/**
 * @brief 获取当前默认远端节点 ID。
 *
 * @param[out] node_id 输出节点 ID，不能为 NULL。
 *
 * @return 复制结果。
 */
Px4Lite_Result_t Px4Lite_GetSelectedRemoteNode(uint8_t *node_id);

#endif
