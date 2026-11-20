/**
 * @file px4lite_command.c
 * @brief Framework 命令执行器。
 *
 * @details
 * 本文件是COMMAND_LONG/COMMAND_ACK及Pi终态通知的执行/权限边界。RX只标准解码，
 * 不直接修改业务状态；0x8040只登记现有声光，不执行控制或返回ACK。
 * 5G-A和Remote ID后续控制面命令必须在本文件登记和分发。
 */

#include "px4lite_command.h"
#include "px4lite_startup_tone.h"

#include "px4lite_config.h"
#include "px4lite_control.h"
#include "px4lite_identity.h"
#include "px4lite_mavlink_tx.h"
#include "px4lite_modules.h"

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
#define PX4LITE_CMD_SET_MOTOR_PWM_US           31013U
#define PX4LITE_CMD_MOTOR_EMERGENCY_STOP       31090U
#define PX4LITE_CMD_AUTO_TAKEOFF               31091U
#define PX4LITE_CMD_AUTO_LANDING               31092U

#if PX4LITE_MOTOR_COUNT != 4U
#error "Motor throttle COMMAND_LONG mapping requires exactly four motor channels"
#endif

/**
 * @brief 判断 MAVLink component 是否指向本机。
 *
 * @details
 * 本机是单一逻辑 component，但在两条链路上以不同 compid 对外呈现：LoRa 空口用 191
 * (PX4LITE_MAVLINK_COMPONENT_ID)，USART6 出口在发送时被改写为 193
 * (PX4LITE_RPI_MAVLINK_COMPONENT_ID)。树莓派从心跳学到的正是 193，并据此寻址命令。
 * 因此两个 compid 都必须视为"发给本机"，否则树莓派用它心跳里看到的 193 寻址会被静默拒绝、
 * 连 COMMAND_ACK 都收不到。0 为广播。
 */
static uint8_t Command_IsForThisComponent(uint8_t target_component)
{
  if (target_component == 0U) { return 1U; }
  if (target_component == (uint8_t)PX4LITE_MAVLINK_COMPONENT_ID) { return 1U; }
#if PX4LITE_ENABLE_RPI_MAVLINK
  if (target_component == (uint8_t)PX4LITE_RPI_MAVLINK_COMPONENT_ID) { return 1U; }
#endif
  return 0U;
}

/**
 * @brief 判断 MAVLink 目标是否指向本机。
 */
static uint8_t Command_IsForThisSystem(uint8_t target_system, uint8_t target_component)
{
  if ((target_system != 0U) && (target_system != (uint8_t)Px4Lite_IdentityGetMavlinkSystemId())) { return 0U; }
  if (Command_IsForThisComponent(target_component) == 0U) { return 0U; }
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

static uint8_t Command_ParamToPulseUs(float value, uint16_t *out)
{
  if (out == 0) { return 0U; }
  if (!((value >= (float)PX4LITE_CONTROL_ESC_MIN_PULSE_US) &&
        (value <= (float)PX4LITE_CONTROL_ESC_MAX_PULSE_US))) { return 0U; }
  *out = (uint16_t)(value + 0.5f);
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
static uint8_t Command_HandleSetMotorThrottle(float param1, float param2, float param3, float param4, uint8_t rpi)
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

  result = rpi ? Px4Lite_ControlSetRpiMotorThrottles(throttle) : Px4Lite_ControlSetMotorThrottles(throttle);
  return Command_MapControlResult(result);
}

static uint8_t Command_HandleSetMotorPulseUs(float param1, float param2, float param3, float param4, uint8_t rpi)
{
  float params[PX4LITE_MOTOR_COUNT];
  uint16_t pulse_us[PX4LITE_MOTOR_COUNT];
  uint8_t i;
  Px4Lite_Result_t result;

  params[0] = param1;
  params[1] = param2;
  params[2] = param3;
  params[3] = param4;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if (Command_ParamToPulseUs(params[i], &pulse_us[i]) == 0U) { return (uint8_t)MAV_RESULT_DENIED; }
  }
  result = rpi ? Px4Lite_ControlSetRpiMotorPulses(pulse_us) : Px4Lite_ControlSetMotorPulses(pulse_us);
  return Command_MapControlResult(result);
}

static uint8_t Command_HandleSetMotorControlMode(float param1, float param2, float param3, float param4)
{
  Px4Lite_ControlMode_t mode;
  int32_t roll_target_deg100;
  int32_t pitch_target_deg100;
  int32_t yaw_target_deg100;
  Px4Lite_Result_t result;

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
  result = Px4Lite_ControlSetModeAndAttitudeTarget(mode, roll_target_deg100, pitch_target_deg100, yaw_target_deg100);
  /* 保留 Control 未编入时旧 31012 的失败应答，降落忙则可在降落完成后重试。 */
  if (result == PX4LITE_NOT_READY) { return (uint8_t)MAV_RESULT_FAILED; }
  return Command_MapControlResult(result);
}

/** @brief 透传同步升速结果；真实接受/拒绝与过程事件由 Control 统一记录。 */
static uint8_t Command_HandleAutoTakeoff(uint32_t now_ms, uint8_t rpi)
{
  Px4Lite_Result_t result = rpi ? Px4Lite_ControlStartRpiAutoTakeoff() : Px4Lite_ControlStartAutoTakeoff();

  (void)now_ms;
  return Command_MapControlResult(result);
}

/**
 * @brief 唯一控制命令执行入口；控制及ACK语义不变，提示来源由配置统一选择。
 * @details 通信层已完成MAVLink校验与补零解码；MAVLink2合法尾零裁剪不另加长度门槛。
 *          本地模式由实际执行结果提示；Pi模式只等0x8040，避免同一请求重复发声。
 */
Px4Lite_Result_t Px4Lite_CommandHandleMavlinkLong(uint16_t command, uint8_t source_system, uint8_t source_component, uint8_t target_system, uint8_t target_component, float param1, float param2, float param3, float param4, Px4Lite_MavlinkLink_t link, uint32_t now_ms)
{
  uint8_t ack_result;
  uint8_t rpi = (link == PX4LITE_MAVLINK_LINK_RPI) ? 1U : 0U;
  uint8_t deferred = 0U;

  if (Command_IsForThisSystem(target_system, target_component) == 0U) { return PX4LITE_IDLE; }

  switch (command) {
    case PX4LITE_CMD_SET_MOTOR_THROTTLE_PERCENT:
      deferred = 1U;
      ack_result = Command_HandleSetMotorThrottle(param1, param2, param3, param4, rpi);
      break;
    case PX4LITE_CMD_SET_MOTOR_CONTROL_MODE:
      ack_result = Command_HandleSetMotorControlMode(param1, param2, param3, param4);
      break;
    case PX4LITE_CMD_SET_MOTOR_PWM_US:
      deferred = 1U;
      ack_result = Command_HandleSetMotorPulseUs(param1, param2, param3, param4, rpi);
      break;
    case PX4LITE_CMD_MOTOR_EMERGENCY_STOP:
      (void)param1;
      (void)param2;
      (void)param3;
      (void)param4;
      ack_result = Command_MapControlResult(Px4Lite_ControlEmergencyStop(now_ms));
      break;
    case PX4LITE_CMD_AUTO_TAKEOFF:
      deferred = 1U;
      ack_result = Command_HandleAutoTakeoff(now_ms, rpi);
      break;
    case PX4LITE_CMD_AUTO_LANDING:
      deferred = 1U;
      ack_result = Command_MapControlResult(rpi ? Px4Lite_ControlStartRpiAutoLanding() : Px4Lite_ControlStartAutoLanding());
      break;
    default:
      ack_result = (uint8_t)MAV_RESULT_UNSUPPORTED;
      break;
  }

#if PX4LITE_RPI_LOCAL_RESULT_FEEDBACK
  /* 接受目标/斜坡不等于执行完成；这些请求的成功提示由Control事实边界发出。 */
  if (rpi && (ack_result != (uint8_t)MAV_RESULT_ACCEPTED || !deferred)) {
    Px4Lite_StartupToneNotifyRpiResult((uint8_t)(ack_result == (uint8_t)MAV_RESULT_ACCEPTED));
  }
#else
  (void)deferred;
#endif

  /* target 原样回填命令来源，link 决定出口：对端用哪个 compid 发来的都能收到应答。 */
  Px4Lite_MavlinkQueueCommandAck(command, ack_result, source_system, source_component, link);
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_CommandHandleMavlinkAck(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component, uint32_t now_ms)
{
  if (Command_IsForThisSystem(target_system, target_component) == 0U) { return PX4LITE_IDLE; }
  Px4Lite_MavlinkRecordCommandAck(command, result, now_ms);
  return PX4LITE_OK;
}

#if PX4LITE_ENABLE_CNS_CONTROL
/** @brief 解码模块命令的固定线号及参数。浮点比较同时拒绝NaN和无穷值。 */
static uint8_t Command_DecodeModuleSwitch(uint8_t confirmation, const float params[7],
                                         Px4Lite_ModuleId_t *id)
{
  uint8_t i;
  if ((confirmation != 0U) || ((params[1] != 0.0f) && (params[1] != 1.0f))) { return 0U; }
  for (i = 2U; i < 7U; ++i) {
    if (params[i] != 0.0f) { return 0U; }
  }
  if (params[0] == 0.0f) { *id = PX4LITE_MODULE_GNSS; }
  else if (params[0] == 1.0f) { *id = PX4LITE_MODULE_IMU; }
  else if (params[0] == 2.0f) { *id = PX4LITE_MODULE_BARO; }
  else if (params[0] == 4.0f) { *id = PX4LITE_MODULE_LORA; }
  else if (params[0] == 7.0f) { *id = PX4LITE_MODULE_REMOTE_ID; }
  else { return 0U; }
  return 1U;
}

/** @brief UART6 CommTask在已有ACK空间时受理模块状态，发送成功后再释放切屏和提示。 */
static Px4Lite_Result_t Command_HandleModuleSwitch(uint8_t confirmation, const float params[7],
                                                  uint8_t local_system, uint32_t now_ms)
{
  Px4Lite_ModuleId_t id;
  uint8_t enabled, result, reason;
  uint32_t token = 0U;

  if (!Px4Lite_MavlinkCnsAckAvailable()) { return PX4LITE_BUSY; }
  if (!Command_DecodeModuleSwitch(confirmation, params, &id)) {
    return Px4Lite_MavlinkQueueCnsAck(PX4LITE_CNS_MODULE_SWITCH_COMMAND, MAV_RESULT_DENIED, 3U,
                                    local_system, 0U, now_ms);
  }

  enabled = (uint8_t)(params[1] == 1.0f);
  switch (Px4Lite_ModuleExperimentRequest(id, enabled, now_ms)) {
    case PX4LITE_OK:
      result = MAV_RESULT_ACCEPTED; reason = 0U;
      /* 31015独占该令牌解释：低位为开关，其余为已校验的模块枚举。 */
      token = ((uint32_t)id << 1U) | enabled;
      break;
    case PX4LITE_BUSY:
      result = MAV_RESULT_TEMPORARILY_REJECTED; reason = 2U;
      break;
    case PX4LITE_IO_ERROR:
      result = MAV_RESULT_FAILED; reason = 0U;
      break;
    default:
      result = MAV_RESULT_DENIED; reason = 4U;
      break;
  }
  return Px4Lite_MavlinkQueueCnsAck(PX4LITE_CNS_MODULE_SWITCH_COMMAND, result, reason,
                                  local_system, token, now_ms);
}
#endif

/**
 * @brief CNS V0.1的UART6受理边界；旧LoRa入口及旧命令参数布局保持独立。
 * @details 先校验寻址/参数和应答容量，再请求Control；拒绝不登记失败声光。
 *          急停在应答拥塞时仍停车，正常动作不会因没有ACK容量而被执行。
 */
Px4Lite_Result_t Px4Lite_CommandHandleRpiLongV01(uint16_t command,
    uint8_t source_system, uint8_t source_component, uint8_t target_system,
    uint8_t target_component, uint8_t confirmation, const float params[7], uint32_t now_ms)
{
#if PX4LITE_ENABLE_CNS_CONTROL
  Px4Lite_CnsAction_t action;
  Px4Lite_Result_t result;
  uint8_t reason = 0U, ack_result, i, motor = 0U;
  uint16_t pulse = 0U;
  uint32_t token = 0U;
  uint8_t local_system = Px4Lite_IdentityGetMavlinkSystemId();
  if ((local_system == 0U) || (source_system != local_system) ||
      (target_system != local_system) || (source_component != PX4LITE_RPI_RESULT_PI_COMPONENT_ID) ||
      (target_component != PX4LITE_RPI_MAVLINK_COMPONENT_ID)) { return PX4LITE_IDLE; }
  if (params == 0) { return PX4LITE_INVALID_PARAM; }
  switch (command) {
    case 31093U: action = PX4LITE_CNS_ARM; break;
    case 31091U: action = PX4LITE_CNS_TAKEOFF; break;
    case 31092U: action = PX4LITE_CNS_LAND; break;
    case 31094U: action = PX4LITE_CNS_DISARM; break;
    case 31090U: action = PX4LITE_CNS_EMERGENCY_STOP; break;
    case 31014U: action = PX4LITE_CNS_SET_MOTOR_PWM; break;
    case PX4LITE_CNS_MODULE_SWITCH_COMMAND:
      return Command_HandleModuleSwitch(confirmation, params, local_system, now_ms);
    case 31011U:
    case 31012U:
    case 31013U:
      /* 原有四路设置、模式配置等不属于本轮六动作，保留其原控制语义。 */
      return Px4Lite_CommandHandleMavlinkLong(command, source_system, source_component,
          target_system, target_component, params[0], params[1], params[2], params[3],
          PX4LITE_MAVLINK_LINK_RPI, now_ms);
    default:
      return Px4Lite_MavlinkQueueCnsAck(command, MAV_RESULT_UNSUPPORTED, 0U,
                                      local_system, 0U, now_ms);
  }
  if (confirmation != 0U) { reason = 3U; }
  for (i = (action == PX4LITE_CNS_SET_MOTOR_PWM) ? 2U : 0U; i < 7U; ++i) {
    /* 与零比较同时拒绝NaN/Inf；在范围检查前绝不把不可信float转为整数。 */
    if (params[i] != 0.0f) { reason = 3U; }
  }
  if (action == PX4LITE_CNS_SET_MOTOR_PWM) {
    if (!((params[0] >= 1.0f) && (params[0] <= 4.0f) &&
          (params[1] >= 1000.0f) && (params[1] <= 2000.0f))) {
      reason = 3U;
    } else if (((float)(uint8_t)params[0] != params[0]) ||
               ((float)(uint16_t)params[1] != params[1])) {
      reason = 3U;
    } else {
      motor = (uint8_t)params[0] - 1U;
      pulse = (uint16_t)params[1];
    }
  }
  if (reason != 0U) {
    return Px4Lite_MavlinkQueueCnsAck(command, MAV_RESULT_DENIED, reason,
                                    local_system, 0U, now_ms);
  }
  if (!Px4Lite_MavlinkCnsAckAvailable() && action != PX4LITE_CNS_EMERGENCY_STOP &&
      action != PX4LITE_CNS_DISARM) {
    /* 最后一项留给急停；满队不覆盖旧受理，Pi将按协议显示是否受理未知。 */
    return PX4LITE_BUSY;
  }
  result = Px4Lite_ControlCnsRequest(action, motor, pulse, &reason, &token);
  if (result == PX4LITE_OK) { ack_result = MAV_RESULT_ACCEPTED; reason = 0U; }
  else if (reason == 2U || result == PX4LITE_BUSY) { ack_result = MAV_RESULT_TEMPORARILY_REJECTED; reason = 2U; }
  else if (reason == 5U || result == PX4LITE_IO_ERROR) { ack_result = MAV_RESULT_FAILED; }
  else { ack_result = MAV_RESULT_DENIED; if (!reason) { reason = 4U; } }
  if (result != PX4LITE_OK) { token = 0U; }
  return Px4Lite_MavlinkQueueCnsAck(command, ack_result, reason, local_system, token, now_ms);
#else
  if (params == 0) { return PX4LITE_INVALID_PARAM; }
  (void)confirmation;
  return Px4Lite_CommandHandleMavlinkLong(command, source_system, source_component,
      target_system, target_component, params[0], params[1], params[2], params[3],
      PX4LITE_MAVLINK_LINK_RPI, now_ms);
#endif
}

/**
 * @brief 处理已标准解码的0x8040终态通知，严格寻址后仅交给现有声光队列。
 * @details 不接触控制输出/ACK，不按0或1去重；相邻两条成功可属于不同请求。
 *          失败外层len6由标准解码补result=0，完整TUNNEL未用填充不参与内层协议。
 */
Px4Lite_Result_t Px4Lite_CommandHandleRpiResultTunnel(uint8_t source_system,
                                                     uint8_t source_component,
                                                     uint8_t target_system,
                                                     uint8_t target_component,
                                                     uint16_t payload_type,
                                                     const uint8_t *payload,
                                                     uint8_t payload_length,
                                                     uint8_t wire_payload_length,
                                                     Px4Lite_MavlinkLink_t link)
{
  if (payload_type != PX4LITE_RPI_RESULT_TUNNEL_TYPE) { return PX4LITE_IDLE; }
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK && PX4LITE_RPI_RESULT_SOURCE_PI
  {
    uint8_t local_system = Px4Lite_IdentityGetMavlinkSystemId();
    if (link != PX4LITE_MAVLINK_LINK_RPI || local_system == 0U ||
        source_system != local_system || target_system != local_system ||
        source_component != PX4LITE_RPI_RESULT_PI_COMPONENT_ID ||
        target_component != PX4LITE_RPI_MAVLINK_COMPONENT_ID || payload == 0 ||
        payload_length != PX4LITE_RPI_RESULT_PAYLOAD_LENGTH ||
        wire_payload_length < 1U || wire_payload_length > 128U) {
      return PX4LITE_INVALID_PARAM;
    }
    if (payload[0] != PX4LITE_RPI_RESULT_VERSION || payload[1] > 1U ||
        (wire_payload_length == 1U && payload[1] != 0U)) {
      return PX4LITE_INVALID_PARAM;
    }
    Px4Lite_StartupToneNotifyRpiResult(payload[1]);
    return PX4LITE_OK;
  }
#else
  (void)source_system; (void)source_component; (void)target_system; (void)target_component;
  (void)payload; (void)payload_length; (void)wire_payload_length; (void)link;
  return PX4LITE_IDLE;
#endif
}
