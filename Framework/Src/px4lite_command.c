/**
 * @file px4lite_command.c
 * @brief Framework 命令执行器。
 *
 * @details
 * 本文件是 COMMAND_LONG / COMMAND_ACK 的唯一执行边界。通信解码层不得在 RX 文件中直接
 * 修改业务状态，发送层不得承载命令分发策略。当前仅保留 COMMAND_ACK 边界，5G-A 和
 * Remote ID 后续控制面命令必须在本文件登记和分发。
 */

#include "px4lite_command.h"

#include "px4lite_config.h"
#include "px4lite_identity.h"
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


/**
 * @brief 判断 MAVLink 目标是否指向本机。
 */
static uint8_t Command_IsForThisSystem(uint8_t target_system, uint8_t target_component)
{
  if ((target_system != 0U) && (target_system != (uint8_t)Px4Lite_IdentityGetMavlinkSystemId())) { return 0U; }
  if ((target_component != 0U) && (target_component != (uint8_t)PX4LITE_MAVLINK_COMPONENT_ID)) { return 0U; }
  return 1U;
}

Px4Lite_Result_t Px4Lite_CommandHandleMavlinkLong(uint16_t command, uint8_t source_system, uint8_t source_component, uint8_t target_system, uint8_t target_component, float param1, float param2, float param3, float param4, uint32_t now_ms)
{
  (void)now_ms;
  (void)param3;
  (void)param4;
  if (Command_IsForThisSystem(target_system, target_component) == 0U) { return PX4LITE_IDLE; }
  Px4Lite_MavlinkQueueCommandAck(command, (uint8_t)MAV_RESULT_UNSUPPORTED, source_system, source_component);
  (void)param1;
  (void)param2;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_CommandHandleMavlinkAck(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component, uint32_t now_ms)
{
  if (Command_IsForThisSystem(target_system, target_component) == 0U) { return PX4LITE_IDLE; }
  Px4Lite_MavlinkRecordCommandAck(command, result, now_ms);
  return PX4LITE_OK;
}
