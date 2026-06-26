/**
 * @file px4lite_attitude.h
 * @brief 声明轻量级 IMU 姿态估计器接口。
 */

#ifndef PX4LITE_ATTITUDE_H
#define PX4LITE_ATTITUDE_H

#include <stdint.h>
#include "px4lite_types.h"

/**
 * @brief 姿态估计器内部状态。
 */
typedef struct {
  float roll_deg;                /**< 当前横滚角，单位 degree。 */
  float pitch_deg;               /**< 当前俯仰角，单位 degree。 */
  float yaw_deg;                 /**< 当前相对航向角，单位 degree。 */
  float gyro_bias_dps[3];        /**< 静止校准得到的陀螺零偏，单位 degree/s。 */
  float gyro_bias_sum_dps[3];    /**< 静止校准阶段的陀螺零偏累加值，单位 degree/s。 */
  uint32_t last_sample_time_ms;  /**< 上一次使用的 IMU 样本时间，单位 ms。 */
  uint16_t gyro_bias_sample_count; /**< 当前零偏校准累计样本数。 */
  uint8_t valid;                 /**< 1 表示估计器已完成首帧初始化。 */
  uint8_t gyro_bias_valid;       /**< 1 表示陀螺零偏已完成静止校准。 */
} Px4Lite_AttitudeState_t;

/**
 * @brief 复位姿态估计器状态。
 *
 * @param[out] state 姿态估计器状态，允许为 NULL。
 */
void Px4Lite_AttitudeInit(Px4Lite_AttitudeState_t *state);

/**
 * @brief 使用一帧 IMU 样本更新姿态估计并输出定点角度。
 *
 * @param[in,out] state 姿态估计器状态，不能为 NULL。
 * @param[in] imu IMU 样本，不能为 NULL。
 * @param[out] roll_deg100 横滚角，单位 degree * 100。
 * @param[out] pitch_deg100 俯仰角，单位 degree * 100。
 * @param[out] yaw_deg100 相对航向角，单位 degree * 100。
 *
 * @retval PX4LITE_OK 更新成功。
 * @retval PX4LITE_INVALID_PARAM 参数非法。
 */
Px4Lite_Result_t Px4Lite_AttitudeUpdate(Px4Lite_AttitudeState_t *state, const Px4Lite_SensorImu_t *imu, int32_t *roll_deg100, int32_t *pitch_deg100, int32_t *yaw_deg100);

#endif
