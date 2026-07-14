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
#include "px4lite_control.h"
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

#define PX4LITE_CMD_SET_MOTOR_THROTTLE_PERCENT 31011U
#define PX4LITE_CMD_SET_MOTOR_CONTROL_MODE     31012U
#define PX4LITE_CMD_MOTOR_EMERGENCY_STOP       31090U

#if PX4LITE_MOTOR_COUNT != 4U
#error "Motor throttle COMMAND_LONG mapping requires exactly four motor channels"
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

/**
 * @brief 将 MAVLink 浮点参数转换为 0 到 100 的油门百分比。
 */
static uint8_t Command_ParamToPercent(float value, uint8_t *out)
{
  if (out == 0) { return 0U; }
  if (!((value >= 0.0f) && (value <= 100.0f))) { return 0U; }
  *out = (uint8_t)(value + 0.5f);
  if (*out > 100U) { *out = 100U; }
  return 1U;
}

/**
 * @brief 按指定绝对角度上限把 MAVLink 浮点角度转换为 degree * 100。
 */
static uint8_t Command_ParamToDeg100(float value, float limit_deg, int32_t *out)
{
  if (out == 0) { return 0U; }
  if (!((limit_deg >= 0.0f) && (value >= -limit_deg) && (value <= limit_deg))) { return 0U; }

  if (value >= 0.0f) {
    *out = (int32_t)((value * 100.0f) + 0.5f);
  } else {
    *out = (int32_t)((value * 100.0f) - 0.5f);
  }
  return 1U;
}

/**
 * @brief 将 Framework 执行结果映射为 MAVLink COMMAND_ACK 结果。
 */
static uint8_t Command_MapControlResult(Px4Lite_Result_t result)
{
  switch (result) {
    case PX4LITE_OK:
      return (uint8_t)MAV_RESULT_ACCEPTED;
    case PX4LITE_BUSY:
    case PX4LITE_NOT_READY:
      return (uint8_t)MAV_RESULT_TEMPORARILY_REJECTED;
    case PX4LITE_INVALID_PARAM:
      return (uint8_t)MAV_RESULT_DENIED;
    default:
      break;
  }

  return (uint8_t)MAV_RESULT_FAILED;
}

/**
 * @brief 执行四电机油门百分比下行命令。
 */
static uint8_t Command_HandleSetMotorThrottle(float param1, float param2, float param3, float param4)
{
  float params[PX4LITE_MOTOR_COUNT];
  uint8_t throttle[PX4LITE_MOTOR_COUNT];
  uint8_t i;
  Px4Lite_Result_t result;

  params[0] = param1;
  params[1] = param2;
  params[2] = param3;
  params[3] = param4;

  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if (Command_ParamToPercent(params[i], &throttle[i]) == 0U) { return (uint8_t)MAV_RESULT_DENIED; }
  }

  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    result = Px4Lite_ControlSetMotorThrottlePercent(i, throttle[i]);
    if (result != PX4LITE_OK) { return Command_MapControlResult(result); }
  }

  return (uint8_t)MAV_RESULT_ACCEPTED;
}

static uint8_t Command_HandleSetMotorControlMode(float param1, float param2, float param3, float param4)
{
  Px4Lite_ControlMode_t mode;
  int32_t roll_target_deg100;
  int32_t pitch_target_deg100;
  int32_t yaw_target_deg100;

  if ((param1 >= -0.01f) && (param1 <= 0.01f)) {
    mode = PX4LITE_CONTROL_MODE_DIRECT;
  } else if ((param1 >= 0.99f) && (param1 <= 1.01f)) {
    mode = PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST;
  } else {
    return (uint8_t)MAV_RESULT_DENIED;
  }
  if (Command_ParamToDeg100(param2, (float)PX4LITE_CONTROL_ATTITUDE_TARGET_LIMIT_DEG100 / 100.0f, &roll_target_deg100) == 0U) { return (uint8_t)MAV_RESULT_DENIED; }
  if (Command_ParamToDeg100(param3, (float)PX4LITE_CONTROL_ATTITUDE_TARGET_LIMIT_DEG100 / 100.0f, &pitch_target_deg100) == 0U) { return (uint8_t)MAV_RESULT_DENIED; }
  if (Command_ParamToDeg100(param4, (float)PX4LITE_CONTROL_ATTITUDE_YAW_TARGET_LIMIT_DEG100 / 100.0f, &yaw_target_deg100) == 0U) { return (uint8_t)MAV_RESULT_DENIED; }
  if (Px4Lite_ControlSetAttitudeTarget(roll_target_deg100, pitch_target_deg100, yaw_target_deg100) != PX4LITE_OK) { return (uint8_t)MAV_RESULT_FAILED; }
  return Command_MapControlResult(Px4Lite_ControlSetMode(mode));
}

Px4Lite_Result_t Px4Lite_CommandHandleMavlinkLong(uint16_t command, uint8_t source_system, uint8_t source_component, uint8_t target_system, uint8_t target_component, float param1, float param2, float param3, float param4, uint32_t now_ms)
{
  uint8_t ack_result;

  if (Command_IsForThisSystem(target_system, target_component) == 0U) { return PX4LITE_IDLE; }

  switch (command) {
    case PX4LITE_CMD_SET_MOTOR_THROTTLE_PERCENT:
      ack_result = Command_HandleSetMotorThrottle(param1, param2, param3, param4);
      break;
    case PX4LITE_CMD_SET_MOTOR_CONTROL_MODE:
      ack_result = Command_HandleSetMotorControlMode(param1, param2, param3, param4);
      break;
    case PX4LITE_CMD_MOTOR_EMERGENCY_STOP:
      (void)param1;
      (void)param2;
      (void)param3;
      (void)param4;
      ack_result = Command_MapControlResult(Px4Lite_ControlEmergencyStop(now_ms));
      break;
    default:
      ack_result = (uint8_t)MAV_RESULT_UNSUPPORTED;
      break;
  }

  Px4Lite_MavlinkQueueCommandAck(command, ack_result, source_system, source_component);
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_CommandHandleMavlinkAck(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component, uint32_t now_ms)
{
  if (Command_IsForThisSystem(target_system, target_component) == 0U) { return PX4LITE_IDLE; }
  Px4Lite_MavlinkRecordCommandAck(command, result, now_ms);
  return PX4LITE_OK;
}
