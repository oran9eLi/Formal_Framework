/**
 * @file px4lite_command.c
 * @brief Framework 命令执行器。
 *
 * @details
 * 本文件是 COMMAND_LONG / COMMAND_ACK 的唯一执行边界。通信解码层不得在 RX 文件中直接
 * 修改业务状态，发送层不得承载命令分发策略。当前仅实现 LoRa 主从流控命令，5G-A 和
 * Remote ID 后续控制面命令必须在本文件登记和分发。
 */

#include "px4lite_command.h"

#include "px4lite_config.h"
#include "px4lite_mavlink_tx.h"

#if defined(__CC_ARM)
#define MAVLINK_ALIGNED_FIELDS   0
#define MAVLINK_COMM_NUM_BUFFERS 1
#pragma diag_suppress 66
#endif

#include "common/mavlink.h"

#if defined(__CC_ARM)
#pragma diag_default 66
#endif

#define PX4LITE_COMMAND_MAVLINK_STREAM ((uint16_t)MAV_CMD_USER_1)
#define PX4LITE_COMMAND_NODE_POLL      ((uint16_t)MAV_CMD_USER_2)
#define COMMAND_STREAM_ACTION_STOP     0U
#define COMMAND_STREAM_ACTION_START    1U

/**
 * @brief 判断 MAVLink 目标是否指向本机。
 */
static uint8_t Command_IsForThisSystem(uint8_t target_system, uint8_t target_component)
{
  if ((target_system != 0U) && (target_system != (uint8_t)PX4LITE_MAVLINK_SYSTEM_ID)) { return 0U; }
  if ((target_component != 0U) && (target_component != (uint8_t)PX4LITE_MAVLINK_COMPONENT_ID)) { return 0U; }
  return 1U;
}

static uint8_t Command_SourceSystemToNode(uint8_t source_system, uint8_t *node_id)
{
  if ((node_id == 0) || (source_system == 0U)) { return 0U; }
  *node_id = source_system;
  return (*node_id < PX4LITE_REMOTE_NODE_MAX) ? 1U : 0U;
}

static uint8_t Command_StreamControlAllowed(uint8_t source_system)
{
  uint8_t source_node;

  if (Command_SourceSystemToNode(source_system, &source_node) == 0U) { return 0U; }
#if PX4LITE_NODE_ROLE == PX4LITE_NODE_ROLE_SLAVE
  return (source_node == (uint8_t)PX4LITE_MASTER_NODE_ID) ? 1U : 0U;
#else
  return (source_node != (uint8_t)PX4LITE_NODE_ID) ? 1U : 0U;
#endif
}

static uint8_t Command_NodePollAllowed(uint8_t source_system)
{
  uint8_t source_node;

  if (Command_SourceSystemToNode(source_system, &source_node) == 0U) { return 0U; }
#if PX4LITE_NODE_ROLE == PX4LITE_NODE_ROLE_SLAVE
  return (source_node == (uint8_t)PX4LITE_MASTER_NODE_ID) ? 1U : 0U;
#else
  return 0U;
#endif
}

/**
 * @brief 执行 LoRa/MAVLink 遥测流控命令。
 */
static Px4Lite_Result_t Command_HandleStreamControl(uint8_t requester_node_id, uint8_t action, uint32_t stream_mask, uint32_t lease_ms, uint32_t now_ms)
{
  if (lease_ms == 0U) { lease_ms = PX4LITE_MAVLINK_STREAM_LEASE_MS; }
  if ((action != COMMAND_STREAM_ACTION_START) && (action != COMMAND_STREAM_ACTION_STOP)) { return PX4LITE_INVALID_PARAM; }

  return Px4Lite_MavlinkApplyStreamControl(requester_node_id, action, stream_mask, lease_ms, now_ms);
}

Px4Lite_Result_t Px4Lite_CommandHandleMavlinkLong(uint16_t command, uint8_t source_system, uint8_t source_component, uint8_t target_system, uint8_t target_component, float param1, float param2, float param3, float param4, uint32_t now_ms)
{
  Px4Lite_Result_t result;

  uint8_t requester_node_id = 0U;

  if (Command_IsForThisSystem(target_system, target_component) == 0U) { return PX4LITE_IDLE; }
  (void)Command_SourceSystemToNode(source_system, &requester_node_id);
  if ((requester_node_id == 0U) && (param2 >= 1.0f) && (param2 < (float)PX4LITE_REMOTE_NODE_MAX)) { requester_node_id = (uint8_t)param2; }

  switch (command) {
    case PX4LITE_COMMAND_MAVLINK_STREAM:
      result = (Command_StreamControlAllowed(source_system) != 0U) ? Command_HandleStreamControl(requester_node_id, (uint8_t)param1, (uint32_t)param3, (uint32_t)param4, now_ms) : PX4LITE_INVALID_PARAM;
      Px4Lite_MavlinkQueueCommandAck(command, (result == PX4LITE_OK) ? (uint8_t)MAV_RESULT_ACCEPTED : (uint8_t)MAV_RESULT_DENIED, source_system, source_component);
      return PX4LITE_OK;

    case PX4LITE_COMMAND_NODE_POLL:
      result = (Command_NodePollAllowed(source_system) != 0U) ? PX4LITE_OK : PX4LITE_INVALID_PARAM;
      Px4Lite_MavlinkQueueCommandAck(command, (result == PX4LITE_OK) ? (uint8_t)MAV_RESULT_ACCEPTED : (uint8_t)MAV_RESULT_DENIED, source_system, source_component);
      return PX4LITE_OK;

    default:
      Px4Lite_MavlinkQueueCommandAck(command, (uint8_t)MAV_RESULT_UNSUPPORTED, source_system, source_component);
      return PX4LITE_OK;
  }
}

Px4Lite_Result_t Px4Lite_CommandHandleMavlinkAck(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component, uint32_t now_ms)
{
  (void)now_ms;
  if (Command_IsForThisSystem(target_system, target_component) == 0U) { return PX4LITE_IDLE; }
  if ((command != PX4LITE_COMMAND_MAVLINK_STREAM) && (command != PX4LITE_COMMAND_NODE_POLL)) { return PX4LITE_IDLE; }

  Px4Lite_MavlinkRecordCommandAck(command, result);
  return PX4LITE_OK;
}
