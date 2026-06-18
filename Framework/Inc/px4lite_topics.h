/**
 * @file px4lite_topics.h
 * @brief Framework 强类型 topic、FIFO、告警队列和命令队列接口。
 *
 * @details
 * 本文件定义 Framework 内部发布和复制数据的唯一接口。低频状态使用 latest-value
 * snapshot，高频 IMU 数据使用有界 FIFO。发布方必须提交完整样本，消费者不得直接
 * 读取 topic 存储区或驱动私有变量。
 */

#ifndef PX4LITE_TOPICS_H
#define PX4LITE_TOPICS_H

#include "px4lite_types.h"

/**
 * @brief 有界 FIFO 运行统计。
 */
typedef struct {
  uint16_t count;          /**< 当前 FIFO 中的元素数量。 */
  uint16_t peak;           /**< 历史峰值占用。 */
  uint32_t push_count;     /**< 累计 push 次数。 */
  uint32_t pop_count;      /**< 累计 pop 次数。 */
  uint32_t overflow_count; /**< 累计溢出次数。 */
} Px4Lite_FifoStats_t;

/**
 * @brief 初始化 topic 存储和已启用的可选队列。
 *
 * @return 初始化结果。
 * @retval PX4LITE_OK 初始化成功。
 * @retval PX4LITE_IO_ERROR 可选 RTOS 队列创建失败。
 *
 * @note 本函数应在调度器启动前调用。
 */
Px4Lite_Result_t Px4Lite_TopicsInit(void);

/**
 * @brief 原子发布一份完整 GNSS 测量快照。
 *
 * @param[in] data GNSS 快照指针，不能为 NULL。
 *
 * @note 发布方必须在传入前完成字段转换和有效性标记。
 */
void Px4Lite_PublishGnss(const Px4Lite_SensorGnss_t *data);

/**
 * @brief 复制最新完整 GNSS 测量快照。
 *
 * @param[out] data 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 * @retval PX4LITE_OK 复制成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 尚未发布有效快照。
 */
Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *data);

/**
 * @brief 向 IMU 有界 FIFO 推入一个样本。
 *
 * @param[in] data IMU 样本指针，不能为 NULL。
 *
 * @return 写入结果。
 * @retval PX4LITE_OK 写入成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 *
 * @note FIFO 满时丢弃最旧样本后写入新样本，并增加 overflow 统计。
 */
Px4Lite_Result_t Px4Lite_PushImu(const Px4Lite_SensorImu_t *data);

/**
 * @brief 从 IMU FIFO 取出最旧样本。
 *
 * @param[out] data 输出缓冲区，不能为 NULL。
 *
 * @return 读取结果。
 * @retval PX4LITE_OK 读取成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY FIFO 当前为空。
 */
Px4Lite_Result_t Px4Lite_PopImu(Px4Lite_SensorImu_t *data);

/**
 * @brief 复制当前 IMU FIFO 使用量和溢出统计。
 *
 * @param[out] stats 输出缓冲区，不能为 NULL。
 */
void Px4Lite_GetImuStats(Px4Lite_FifoStats_t *stats);

/**
 * @brief 原子发布一份完整气压计测量快照。
 *
 * @param[in] data 气压计快照指针，不能为 NULL。
 */
void Px4Lite_PublishBaro(const Px4Lite_SensorBaro_t *data);

/**
 * @brief 复制最新完整气压计测量快照。
 *
 * @param[out] data 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 * @retval PX4LITE_OK 复制成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 尚未发布有效快照。
 */
Px4Lite_Result_t Px4Lite_CopyBaro(Px4Lite_SensorBaro_t *data);

/**
 * @brief 原子发布一份完整电池状态快照。
 *
 * @param[in] data 电池状态快照指针，不能为 NULL。
 */
void Px4Lite_PublishBattery(const Px4Lite_BatteryStatus_t *data);

/**
 * @brief 复制最新完整电池状态快照。
 *
 * @param[out] data 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 * @retval PX4LITE_OK 复制成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 尚未发布有效快照。
 */
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *data);

/**
 * @brief 原子发布一份完整导航域快照。
 *
 * @param[in] data 导航域快照指针，不能为 NULL。
 */
void Px4Lite_PublishNavigation(const Px4Lite_VehicleNavigation_t *data);

/**
 * @brief 复制最新完整导航域快照。
 *
 * @param[out] data 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 * @retval PX4LITE_OK 复制成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 尚未发布有效快照。
 */
Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *data);

/**
 * @brief 原子发布一份完整系统健康快照。
 *
 * @param[in] data 系统健康快照指针，不能为 NULL。
 */
void Px4Lite_PublishHealth(const Px4Lite_SystemHealth_t *data);

/**
 * @brief 复制最新完整系统健康快照。
 *
 * @param[out] data 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 * @retval PX4LITE_OK 复制成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 尚未发布有效快照。
 */
Px4Lite_Result_t Px4Lite_CopyHealth(Px4Lite_SystemHealth_t *data);

/**
 * @brief 发布一个告警事件到有界告警队列。
 *
 * @param[in] event 告警事件指针，不能为 NULL。
 *
 * @return 发布结果。
 * @retval PX4LITE_OK 发布成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 告警队列未启用、未创建或发送失败。
 */
Px4Lite_Result_t Px4Lite_PublishAlarm(const Px4Lite_AlarmEvent_t *event);

/**
 * @brief 非阻塞读取一个待处理告警事件。
 *
 * @param[out] event 输出缓冲区，不能为 NULL。
 *
 * @return 读取结果。
 * @retval PX4LITE_OK 读取成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 当前无待处理告警。
 */
Px4Lite_Result_t Px4Lite_TakeAlarm(Px4Lite_AlarmEvent_t *event);

/**
 * @brief 发布一个应用命令到有界命令队列。
 *
 * @param[in] command 命令指针，不能为 NULL。
 *
 * @return 发布结果。
 * @retval PX4LITE_OK 发布成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 命令队列未启用、未创建或发送失败。
 */
Px4Lite_Result_t Px4Lite_PublishCommand(const Px4Lite_Command_t *command);

/**
 * @brief 非阻塞读取一个待处理应用命令。
 *
 * @param[out] command 输出缓冲区，不能为 NULL。
 *
 * @return 读取结果。
 * @retval PX4LITE_OK 读取成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 当前无待处理命令。
 */
Px4Lite_Result_t Px4Lite_TakeCommand(Px4Lite_Command_t *command);

/**
 * @brief 发布一个命令回执到有界 ACK 队列。
 *
 * @param[in] ack 命令回执指针，不能为 NULL。
 *
 * @return 发布结果。
 * @retval PX4LITE_OK 发布成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY ACK 队列未启用、未创建或发送失败。
 */
Px4Lite_Result_t Px4Lite_PublishCommandAck(const Px4Lite_CommandAck_t *ack);

/**
 * @brief 非阻塞读取一个待处理命令回执。
 *
 * @param[out] ack 输出缓冲区，不能为 NULL。
 *
 * @return 读取结果。
 * @retval PX4LITE_OK 读取成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 当前无待处理 ACK。
 */
Px4Lite_Result_t Px4Lite_TakeCommandAck(Px4Lite_CommandAck_t *ack);

#endif
