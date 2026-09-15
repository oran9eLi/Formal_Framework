/**
 * @file px4lite_attitude.h
 * @brief 声明轻量级 IMU 姿态估计器接口。
 */

#ifndef PX4LITE_ATTITUDE_H
#define PX4LITE_ATTITUDE_H

#include <stdint.h>
#include "px4lite_types.h"

/** @brief 学生主动发起的姿态操作；归零不改变控制/日志测量值。 */
typedef enum {
  PX4LITE_ATTITUDE_ACTION_NONE = 0,
  PX4LITE_ATTITUDE_ACTION_BIAS,
  PX4LITE_ATTITUDE_ACTION_ZERO,
  PX4LITE_ATTITUDE_ACTION_RESTORE
} Px4Lite_AttitudeAction_t;

/** @brief 可供界面解释的操作阶段。 */
typedef enum {
  PX4LITE_ATTITUDE_IDLE = 0,
  PX4LITE_ATTITUDE_REQUESTED,
  PX4LITE_ATTITUDE_WAIT_STILL,
  PX4LITE_ATTITUDE_COLLECTING,
  PX4LITE_ATTITUDE_SUCCEEDED,
  PX4LITE_ATTITUDE_FAILED
} Px4Lite_AttitudePhase_t;

/** @brief 操作失败原因，避免把请求入队当作校准成功。 */
typedef enum {
  PX4LITE_ATTITUDE_REASON_NONE = 0,
  PX4LITE_ATTITUDE_REASON_NO_DATA,
  PX4LITE_ATTITUDE_REASON_NOT_STILL
} Px4Lite_AttitudeReason_t;

/** @brief 估计任务发布的只读操作状态与显示相对基准，时间单位 ms。 */
typedef struct {
  uint32_t started_ms;             /**< 当前操作开始时刻。 */
  uint32_t last_sample_ms;         /**< 最近实际 IMU 样本时刻。 */
  int32_t offset_deg100[3];        /**< 屏幕相对基准，roll/pitch/yaw，单位0.01度。 */
  uint8_t action;                 /**< Px4Lite_AttitudeAction_t。 */
  uint8_t phase;                  /**< Px4Lite_AttitudePhase_t。 */
  uint8_t reason;                 /**< Px4Lite_AttitudeReason_t。 */
  uint8_t progress;               /**< 校准完成百分比0~100。 */
  uint8_t measurement_valid;      /**< 最近一次发布时是否有新鲜姿态。 */
  uint8_t bias_valid;             /**< 是否已有成功校准的零偏。 */
  uint8_t reference_active;       /**< 是否使用学生设置的相对显示基准。 */
  uint8_t reserved;               /**< 对齐。 */
} Px4Lite_AttitudeStatus_t;

/**
 * @brief 姿态估计器内部状态。
 */
typedef struct {
  Px4Lite_AttitudeStatus_t operation; /**< 校准与相对观察基准状态。 */
  float roll_deg;                /**< 当前横滚角，单位 degree。 */
  float pitch_deg;               /**< 当前俯仰角，单位 degree。 */
  float yaw_deg;                 /**< 当前相对航向角，单位 degree。 */
  float gyro_bias_dps[3];        /**< 静止校准得到的陀螺零偏，单位 degree/s。 */
  float gyro_bias_sum_dps[3];    /**< 静止校准阶段的陀螺零偏累加值，单位 degree/s。 */
  float accel_prev_g[3];         /**< 上一帧加速度，用于静止检测，单位 g。 */
  float gyro_prev_dps[3];        /**< 上一帧角速度，用于静止检测，单位 degree/s。 */
  uint32_t last_sample_time_ms;  /**< 上一次使用的 IMU 样本时间，单位 ms。 */
  uint16_t gyro_bias_sample_count; /**< 当前零偏校准累计样本数。 */
  uint8_t valid;                 /**< 1 表示估计器已完成首帧初始化。 */
  uint8_t gyro_bias_valid;       /**< 1 表示陀螺零偏已完成静止校准。 */
  uint8_t motion_ref_valid;      /**< 1 表示 accel_prev/gyro_prev 已有上一帧数据。 */
} Px4Lite_AttitudeState_t;

/**
 * @brief 复位姿态估计器状态。
 *
 * @param[out] state 姿态估计器状态，允许为 NULL。
 */
void Px4Lite_AttitudeInit(Px4Lite_AttitudeState_t *state);

/** @brief 估计任务内执行操作请求；无新鲜测量时拒绝，正在校准时返回 BUSY。 */
Px4Lite_Result_t Px4Lite_AttitudeRequest(Px4Lite_AttitudeState_t *state, Px4Lite_AttitudeAction_t action, uint32_t now_ms);
/** @brief 每个估计周期调用；无样本时也推进失联/校准超时状态。 */
void Px4Lite_AttitudeService(Px4Lite_AttitudeState_t *state, uint32_t now_ms);

/**
 * @brief 使用一帧 IMU 样本更新姿态估计并输出定点角度。
 *
 * @param[in,out] state 姿态估计器状态，不能为 NULL。
 * @param[in] imu IMU 样本，不能为 NULL。
 * @param[out] roll_deg100 测量横滚角，不扣除屏幕相对基准，单位 degree * 100。
 * @param[out] pitch_deg100 俯仰角，单位 degree * 100。
 * @param[out] yaw_deg100 相对航向角，单位 degree * 100。
 *
 * @retval PX4LITE_OK 更新成功。
 * @retval PX4LITE_INVALID_PARAM 参数非法。
 */
Px4Lite_Result_t Px4Lite_AttitudeUpdate(Px4Lite_AttitudeState_t *state, const Px4Lite_SensorImu_t *imu, int32_t *roll_deg100, int32_t *pitch_deg100, int32_t *yaw_deg100);

#endif
