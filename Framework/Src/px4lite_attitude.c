/**
 * @file px4lite_attitude.c
 * @brief 实现一阶互补滤波姿态估计器。
 *
 * @details
 * 当前实现使用加速度计校正 roll/pitch，使用陀螺 Z 轴积分得到相对 yaw。
 * yaw 尚未引入磁力计或双天线 GNSS 绝对航向约束。
 */

#include "px4lite_attitude.h"

#include <math.h>
#include <string.h>
#include "px4lite_config.h"

#define PX4LITE_ATTITUDE_RAD_TO_DEG              57.2957795f
#define PX4LITE_ATTITUDE_DEFAULT_DT_S            0.02f
#define PX4LITE_ATTITUDE_MAX_DT_S                0.2f
#define PX4LITE_ATTITUDE_FILTER_TAU_S            0.49f
#define PX4LITE_ATTITUDE_ALPHA_MIN               0.90f
#define PX4LITE_ATTITUDE_ALPHA_MAX               0.999f
#define PX4LITE_ATTITUDE_ACCEL_FULL_MIN_G        0.95f
#define PX4LITE_ATTITUDE_ACCEL_FULL_MAX_G        1.05f
#define PX4LITE_ATTITUDE_ACCEL_TRUST_MIN_G       0.85f
#define PX4LITE_ATTITUDE_ACCEL_TRUST_MAX_G       1.15f
#define PX4LITE_ATTITUDE_GYRO_STATIC_LIMIT_DPS   0.5f
#define PX4LITE_ATTITUDE_GYRO_BIAS_SAMPLE_COUNT  128U
#define PX4LITE_ATTITUDE_CAL_TIMEOUT_MS         10000U
/* 校准仅在学生明确放稳后启动；判据是采样准入，不用于冻结测量姿态。 */
#define PX4LITE_ATTITUDE_STATIC_ACCEL_NORM_TOL_G 0.05f
#define PX4LITE_ATTITUDE_STATIC_ACCEL_DELTA_G    0.03f
#define PX4LITE_ATTITUDE_STATIC_GYRO_DELTA_DPS   2.0f

/**
 * @brief 计算浮点绝对值，避免依赖库实现差异。
 *
 * @param[in] value 输入值。
 *
 * @return 输入值的绝对值。
 */
static float Px4Lite_AttitudeAbsFloat(float value)
{
  return (value >= 0.0f) ? value : -value;
}

/**
 * @brief 将浮点值限制在指定闭区间内。
 *
 * @param[in] value 输入值。
 * @param[in] min_value 下限。
 * @param[in] max_value 上限。
 *
 * @return 限幅后的值。
 */
static float Px4Lite_AttitudeClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value) { return min_value; }
  if (value > max_value) { return max_value; }
  return value;
}

/**
 * @brief 将 degree 浮点角度转换为 degree*100 定点值。
 *
 * @param[in] value 角度值，单位 degree。
 *
 * @return 四舍五入后的 degree * 100。
 */
static int32_t Px4Lite_AttitudeRoundDeg100(float value)
{
  if (value >= 0.0f) { return (int32_t)((value * 100.0f) + 0.5f); }
  return (int32_t)((value * 100.0f) - 0.5f);
}

/**
 * @brief 将角度归一化到 [-180, 180) 范围。
 *
 * @param[in] value 待归一化角度，单位 degree。
 *
 * @return 归一化后的角度，单位 degree。
 */
static float Px4Lite_AttitudeWrapDeg(float value)
{
  while (value >= 180.0f) { value -= 360.0f; }
  while (value < -180.0f) { value += 360.0f; }
  return value;
}

/**
 * @brief 计算加速度模长。
 *
 * @param[in] accel_x_g X 轴加速度，单位 g。
 * @param[in] accel_y_g Y 轴加速度，单位 g。
 * @param[in] accel_z_g Z 轴加速度，单位 g。
 *
 * @return 加速度模长，单位 g。
 */
static float Px4Lite_AttitudeAccelNorm(float accel_x_g, float accel_y_g, float accel_z_g)
{
  return sqrtf((accel_x_g * accel_x_g) + (accel_y_g * accel_y_g) + (accel_z_g * accel_z_g));
}

/**
 * @brief 按加速度模长估算重力方向可信度。
 *
 * @details 加速度模长接近 1g 时认为主要由重力贡献，可用于修正 roll/pitch；
 * 偏离 1g 过多时通常包含线加速度或冲击，此时降低或关闭加速度修正。
 *
 * @param[in] accel_norm_g 加速度模长，单位 g。
 *
 * @return 0.0 表示不可信，1.0 表示完全可信。
 */
static float Px4Lite_AttitudeAccelTrust(float accel_norm_g)
{
  if ((accel_norm_g <= PX4LITE_ATTITUDE_ACCEL_TRUST_MIN_G) || (accel_norm_g >= PX4LITE_ATTITUDE_ACCEL_TRUST_MAX_G)) { return 0.0f; }
  if ((accel_norm_g >= PX4LITE_ATTITUDE_ACCEL_FULL_MIN_G) && (accel_norm_g <= PX4LITE_ATTITUDE_ACCEL_FULL_MAX_G)) { return 1.0f; }
  if (accel_norm_g < PX4LITE_ATTITUDE_ACCEL_FULL_MIN_G) {
    return (accel_norm_g - PX4LITE_ATTITUDE_ACCEL_TRUST_MIN_G) / (PX4LITE_ATTITUDE_ACCEL_FULL_MIN_G - PX4LITE_ATTITUDE_ACCEL_TRUST_MIN_G);
  }
  return (PX4LITE_ATTITUDE_ACCEL_TRUST_MAX_G - accel_norm_g) / (PX4LITE_ATTITUDE_ACCEL_TRUST_MAX_G - PX4LITE_ATTITUDE_ACCEL_FULL_MAX_G);
}

/**
 * @brief 计算随采样周期和加速度可信度变化的互补滤波系数。
 *
 * @param[in] dt_s 当前采样间隔，单位 s。
 * @param[in] accel_trust 加速度可信度，范围 0.0 到 1.0。
 *
 * @return 陀螺预测项权重，范围 0.90 到 1.0。
 */
static float Px4Lite_AttitudeDynamicAlpha(float dt_s, float accel_trust)
{
  float base_alpha;
  float correction_weight;

  base_alpha = PX4LITE_ATTITUDE_FILTER_TAU_S / (PX4LITE_ATTITUDE_FILTER_TAU_S + dt_s);
  base_alpha = Px4Lite_AttitudeClampFloat(base_alpha, PX4LITE_ATTITUDE_ALPHA_MIN, PX4LITE_ATTITUDE_ALPHA_MAX);
  accel_trust = Px4Lite_AttitudeClampFloat(accel_trust, 0.0f, 1.0f);
  correction_weight = (1.0f - base_alpha) * accel_trust;
  return Px4Lite_AttitudeClampFloat(1.0f - correction_weight, PX4LITE_ATTITUDE_ALPHA_MIN, 1.0f);
}

/**
 * @brief 判断设备当前是否物理静止。
 *
 * @details
 * 同时检查原始角速度绝对值、加速度模长及相邻变化，避免把匀速转动
 * 当成稳定零偏。此准入无法物理区分阈值以下的慢速转动与零偏，仍需学生放稳。
 * 超过准入范围的零偏不得强行接受，应报告未放稳并交实板诊断。
 *
 * @param[in] state 估计器状态(读取上一帧加速度/陀螺)。
 * @retval 1 静止；0 运动。
 */
static uint8_t Px4Lite_AttitudeIsStatic(const Px4Lite_AttitudeState_t *state, float accel_norm_g, float accel_x_g, float accel_y_g, float accel_z_g, float gyro_x_dps, float gyro_y_dps, float gyro_z_dps)
{
  if ((Px4Lite_AttitudeAbsFloat(gyro_x_dps) > PX4LITE_ATTITUDE_GYRO_STATIC_LIMIT_DPS) ||
      (Px4Lite_AttitudeAbsFloat(gyro_y_dps) > PX4LITE_ATTITUDE_GYRO_STATIC_LIMIT_DPS) ||
      (Px4Lite_AttitudeAbsFloat(gyro_z_dps) > PX4LITE_ATTITUDE_GYRO_STATIC_LIMIT_DPS)) { return 0U; }
  if (Px4Lite_AttitudeAbsFloat(accel_norm_g - 1.0f) > PX4LITE_ATTITUDE_STATIC_ACCEL_NORM_TOL_G) { return 0U; }
  if (state->motion_ref_valid == 0U) { return 0U; }
  if (Px4Lite_AttitudeAbsFloat(accel_x_g - state->accel_prev_g[0]) > PX4LITE_ATTITUDE_STATIC_ACCEL_DELTA_G) { return 0U; }
  if (Px4Lite_AttitudeAbsFloat(accel_y_g - state->accel_prev_g[1]) > PX4LITE_ATTITUDE_STATIC_ACCEL_DELTA_G) { return 0U; }
  if (Px4Lite_AttitudeAbsFloat(accel_z_g - state->accel_prev_g[2]) > PX4LITE_ATTITUDE_STATIC_ACCEL_DELTA_G) { return 0U; }
  if (Px4Lite_AttitudeAbsFloat(gyro_x_dps - state->gyro_prev_dps[0]) > PX4LITE_ATTITUDE_STATIC_GYRO_DELTA_DPS) { return 0U; }
  if (Px4Lite_AttitudeAbsFloat(gyro_y_dps - state->gyro_prev_dps[1]) > PX4LITE_ATTITUDE_STATIC_GYRO_DELTA_DPS) { return 0U; }
  if (Px4Lite_AttitudeAbsFloat(gyro_z_dps - state->gyro_prev_dps[2]) > PX4LITE_ATTITUDE_STATIC_GYRO_DELTA_DPS) { return 0U; }
  return 1U;
}

/**
 * @brief 更新陀螺零偏校准状态。
 *
 * @param[in,out] state 姿态估计器状态。
 * @param[in] is_static 当前样本是否满足静止判据。
 * @param[in] gyro_x_dps X 轴角速度，单位 degree/s。
 * @param[in] gyro_y_dps Y 轴角速度，单位 degree/s。
 * @param[in] gyro_z_dps Z 轴角速度，单位 degree/s。
 *
 * @note 仅在显式校准期间采样，完成前继续使用旧零偏，测量姿态始终正常积分。
 */
static void Px4Lite_AttitudeUpdateGyroBias(Px4Lite_AttitudeState_t *state, uint8_t is_static, float gyro_x_dps, float gyro_y_dps, float gyro_z_dps)
{
  if ((state->operation.phase != PX4LITE_ATTITUDE_WAIT_STILL) &&
      (state->operation.phase != PX4LITE_ATTITUDE_COLLECTING)) { return; }

  if (is_static == 0U) {
    state->gyro_bias_sum_dps[0] = 0.0f;
    state->gyro_bias_sum_dps[1] = 0.0f;
    state->gyro_bias_sum_dps[2] = 0.0f;
    state->gyro_bias_sample_count = 0U;
    state->operation.phase = PX4LITE_ATTITUDE_WAIT_STILL;
    state->operation.progress = 0U;
    return;
  }

  state->gyro_bias_sum_dps[0] += gyro_x_dps;
  state->gyro_bias_sum_dps[1] += gyro_y_dps;
  state->gyro_bias_sum_dps[2] += gyro_z_dps;
  state->gyro_bias_sample_count++;
  state->operation.phase = PX4LITE_ATTITUDE_COLLECTING;
  state->operation.progress = (uint8_t)((state->gyro_bias_sample_count * 100U) / PX4LITE_ATTITUDE_GYRO_BIAS_SAMPLE_COUNT);

  if (state->gyro_bias_sample_count >= PX4LITE_ATTITUDE_GYRO_BIAS_SAMPLE_COUNT) {
    const float sample_count = (float)state->gyro_bias_sample_count;
    state->gyro_bias_dps[0] = state->gyro_bias_sum_dps[0] / sample_count;
    state->gyro_bias_dps[1] = state->gyro_bias_sum_dps[1] / sample_count;
    state->gyro_bias_dps[2] = state->gyro_bias_sum_dps[2] / sample_count;
    state->gyro_bias_valid = 1U;
    state->operation.bias_valid = 1U;
    state->operation.phase = PX4LITE_ATTITUDE_SUCCEEDED;
  }
}

/**
 * @brief 计算当前 IMU 样本与上一样本之间的时间间隔。
 *
 * @param[in] state 姿态估计器状态。
 * @param[in] imu 当前 IMU 样本。
 *
 * @return 可用于积分的时间间隔，单位 s。
 */
static float Px4Lite_AttitudeDtSeconds(Px4Lite_AttitudeState_t *state, const Px4Lite_SensorImu_t *imu)
{
  float dt_s = PX4LITE_ATTITUDE_DEFAULT_DT_S;

  if (state->valid != 0U) {
    dt_s = ((float)(uint32_t)(imu->header.sample_time_ms - state->last_sample_time_ms)) / 1000.0f;
  } else if (imu->sample_period_us != 0U) {
    dt_s = ((float)imu->sample_period_us) / 1000000.0f;
  }

  /* 丢样本后的未知运动不能用当前角速度补算。 */
  if ((dt_s <= 0.0f) || (dt_s > PX4LITE_ATTITUDE_MAX_DT_S)) { dt_s = 0.0f; }
  return dt_s;
}

void Px4Lite_AttitudeInit(Px4Lite_AttitudeState_t *state)
{
  if (state != 0) { memset(state, 0, sizeof(*state)); }
}

/** @brief 更新样本新鲜度及校准失联/超时，失败时保留旧零偏和相对基准。 */
void Px4Lite_AttitudeService(Px4Lite_AttitudeState_t *state, uint32_t now_ms)
{
  if (state == 0) { return; }
  state->operation.last_sample_ms = state->last_sample_time_ms;
  state->operation.measurement_valid = (uint8_t)((state->valid != 0U) &&
      ((uint32_t)(now_ms - state->last_sample_time_ms) <= PX4LITE_IMU_MAX_AGE_MS));
  if ((state->operation.phase != PX4LITE_ATTITUDE_WAIT_STILL) &&
      (state->operation.phase != PX4LITE_ATTITUDE_COLLECTING)) { return; }
  if (state->operation.measurement_valid == 0U) {
    state->operation.phase = PX4LITE_ATTITUDE_FAILED;
    state->operation.reason = PX4LITE_ATTITUDE_REASON_NO_DATA;
  } else if ((uint32_t)(now_ms - state->operation.started_ms) >= PX4LITE_ATTITUDE_CAL_TIMEOUT_MS) {
    state->operation.phase = PX4LITE_ATTITUDE_FAILED;
    state->operation.reason = PX4LITE_ATTITUDE_REASON_NOT_STILL;
  }
}

/** @brief 仅估计任务调用；校准、屏幕相对归零和恢复基准相互独立。 */
Px4Lite_Result_t Px4Lite_AttitudeRequest(Px4Lite_AttitudeState_t *state, Px4Lite_AttitudeAction_t action, uint32_t now_ms)
{
  if ((state == 0) || (action < PX4LITE_ATTITUDE_ACTION_BIAS) || (action > PX4LITE_ATTITUDE_ACTION_RESTORE)) { return PX4LITE_INVALID_PARAM; }
  Px4Lite_AttitudeService(state, now_ms);
  if ((state->operation.phase == PX4LITE_ATTITUDE_WAIT_STILL) ||
      (state->operation.phase == PX4LITE_ATTITUDE_COLLECTING)) { return PX4LITE_BUSY; }
  state->operation.action = (uint8_t)action;
  state->operation.started_ms = now_ms;
  state->operation.reason = PX4LITE_ATTITUDE_REASON_NONE;
  state->operation.progress = 0U;
  if (state->operation.measurement_valid == 0U) {
    state->operation.phase = PX4LITE_ATTITUDE_FAILED;
    state->operation.reason = PX4LITE_ATTITUDE_REASON_NO_DATA;
    return PX4LITE_NOT_READY;
  }
  if (action == PX4LITE_ATTITUDE_ACTION_BIAS) {
    memset(state->gyro_bias_sum_dps, 0, sizeof(state->gyro_bias_sum_dps));
    state->gyro_bias_sample_count = 0U;
    state->motion_ref_valid = 0U;
    state->operation.phase = PX4LITE_ATTITUDE_WAIT_STILL;
  } else {
    state->operation.reference_active = (uint8_t)(action == PX4LITE_ATTITUDE_ACTION_ZERO);
    state->operation.offset_deg100[0] = (action == PX4LITE_ATTITUDE_ACTION_ZERO) ? Px4Lite_AttitudeRoundDeg100(state->roll_deg) : 0;
    state->operation.offset_deg100[1] = (action == PX4LITE_ATTITUDE_ACTION_ZERO) ? Px4Lite_AttitudeRoundDeg100(state->pitch_deg) : 0;
    state->operation.offset_deg100[2] = (action == PX4LITE_ATTITUDE_ACTION_ZERO) ? Px4Lite_AttitudeRoundDeg100(state->yaw_deg) : 0;
    state->operation.phase = PX4LITE_ATTITUDE_SUCCEEDED;
    state->operation.progress = 100U;
  }
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_AttitudeUpdate(Px4Lite_AttitudeState_t *state, const Px4Lite_SensorImu_t *imu, int32_t *roll_deg100, int32_t *pitch_deg100, int32_t *yaw_deg100)
{
  float accel_x_g;
  float accel_y_g;
  float accel_z_g;
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
  float roll_acc;
  float pitch_acc;
  float roll_gyro;
  float pitch_gyro;
  float dt_s;
  float accel_norm_g;
  float accel_trust;
  float alpha;
  uint8_t is_static;

  if ((state == 0) || (imu == 0) || (roll_deg100 == 0) || (pitch_deg100 == 0) || (yaw_deg100 == 0)) { return PX4LITE_INVALID_PARAM; }
  if (imu->header.valid == 0U) { return PX4LITE_NOT_READY; }
  if ((state->valid != 0U) && ((int32_t)(imu->header.sample_time_ms - state->last_sample_time_ms) <= 0)) { return PX4LITE_IDLE; }

  accel_x_g  = ((float)imu->accel_mg[0]) / 1000.0f;
  accel_y_g  = ((float)imu->accel_mg[1]) / 1000.0f;
  accel_z_g  = ((float)imu->accel_mg[2]) / 1000.0f;
  gyro_x_dps = ((float)imu->gyro_mdps[0]) / 1000.0f;
  gyro_y_dps = ((float)imu->gyro_mdps[1]) / 1000.0f;
  gyro_z_dps = ((float)imu->gyro_mdps[2]) / 1000.0f;

  accel_norm_g = Px4Lite_AttitudeAccelNorm(accel_x_g, accel_y_g, accel_z_g);
  accel_trust  = Px4Lite_AttitudeAccelTrust(accel_norm_g);
  if ((state->valid == 0U) && (accel_trust <= 0.0f)) { return PX4LITE_NOT_READY; }
  if ((state->valid != 0U) && ((uint32_t)(imu->header.sample_time_ms - state->last_sample_time_ms) > 200U)) {
    state->motion_ref_valid = 0U;
  }
  is_static    = Px4Lite_AttitudeIsStatic(state, accel_norm_g, accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps);
  /* 记录本帧原始加速度/陀螺，供下一帧静止检测做变化量比较(必须在零偏扣除之前)。 */
  state->accel_prev_g[0]  = accel_x_g;
  state->accel_prev_g[1]  = accel_y_g;
  state->accel_prev_g[2]  = accel_z_g;
  state->gyro_prev_dps[0] = gyro_x_dps;
  state->gyro_prev_dps[1] = gyro_y_dps;
  state->gyro_prev_dps[2] = gyro_z_dps;
  state->motion_ref_valid = 1U;
  Px4Lite_AttitudeUpdateGyroBias(state, is_static, gyro_x_dps, gyro_y_dps, gyro_z_dps);
  if (state->gyro_bias_valid != 0U) {
    gyro_x_dps -= state->gyro_bias_dps[0];
    gyro_y_dps -= state->gyro_bias_dps[1];
    gyro_z_dps -= state->gyro_bias_dps[2];
  }
  roll_acc  = atan2f(accel_y_g, accel_z_g) * PX4LITE_ATTITUDE_RAD_TO_DEG;
  pitch_acc = atan2f(-accel_x_g, sqrtf((accel_y_g * accel_y_g) + (accel_z_g * accel_z_g))) * PX4LITE_ATTITUDE_RAD_TO_DEG;

  if (state->valid == 0U) {
    if (accel_trust > 0.0f) {
      state->roll_deg  = roll_acc;
      state->pitch_deg = pitch_acc;
    } else {
      state->roll_deg  = 0.0f;
      state->pitch_deg = 0.0f;
    }
    state->yaw_deg   = 0.0f;
    state->valid     = 1U;
  } else {
    dt_s             = Px4Lite_AttitudeDtSeconds(state, imu);
    roll_gyro        = state->roll_deg + (gyro_x_dps * dt_s);
    pitch_gyro       = state->pitch_deg + (gyro_y_dps * dt_s);
    alpha            = Px4Lite_AttitudeDynamicAlpha(dt_s, accel_trust);
    state->roll_deg  = (alpha * roll_gyro) + ((1.0f - alpha) * roll_acc);
    state->pitch_deg = (alpha * pitch_gyro) + ((1.0f - alpha) * pitch_acc);
    state->yaw_deg   = Px4Lite_AttitudeWrapDeg(state->yaw_deg + (gyro_z_dps * dt_s));
  }

  state->last_sample_time_ms = imu->header.sample_time_ms;

  *roll_deg100               = Px4Lite_AttitudeRoundDeg100(state->roll_deg);
  *pitch_deg100              = Px4Lite_AttitudeRoundDeg100(state->pitch_deg);
  *yaw_deg100                = Px4Lite_AttitudeRoundDeg100(state->yaw_deg);
  return PX4LITE_OK;
}
