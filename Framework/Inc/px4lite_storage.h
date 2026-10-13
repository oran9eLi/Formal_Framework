/**
 * @file px4lite_storage.h
 * @brief 声明注册表驱动的 SD CSV 存储模块接口。
 *
 * @details
 * 存储模块像普通 Framework 模块一样接入注册表，提供 init/recover 生命周期回调。
 * 数据只从 Framework topic 的 copy API 读取，格式化为 CSV 后交给 Storage 驱动写入
 * SD 卡。本层不得包含 Business 头文件。
 */

#ifndef PX4LITE_STORAGE_H
#define PX4LITE_STORAGE_H

#include "px4lite_types.h"
/** @brief 手动数据CSV状态；事件CSV独立继续。 */
typedef enum { PX4LITE_RECORD_STOPPED=0,PX4LITE_RECORD_RECORDING,PX4LITE_RECORD_STOPPING,
 PX4LITE_RECORD_SAVED,PX4LITE_RECORD_PARTIAL,PX4LITE_RECORD_ERROR } Px4Lite_RecordingState_t;
/** @brief 单槽请求，操作0Start/1Stop/2Edit，间隔0=1/1=2/2=5/3=10秒。 */
typedef struct { uint8_t operation; /**< 有界操作。 */uint8_t interval; /**< 间隔编号0..3。 */uint16_t reserved; /**< 对齐。 */ } Px4Lite_RecordingRequest_t;
/** @brief 数据会话计数，单位均为本会话记录条数，不能混用SD总written。 */
typedef struct {
 uint32_t request_id; /**< 动作递增身份。 */uint32_t completed_request_id; /**< 已处理动作。 */
 uint32_t session_id; /**< 非零数据会话，事件使用0。 */uint32_t accepted_count; /**< 实际采样/格式化尝试条数。 */
 uint32_t queued_count; /**< 成功入队条数。 */uint32_t written_count; /**< 同会话完整写入条数。 */
 uint32_t synced_count; /**< 最后成功fsync覆盖的written条数。 */uint32_t outstanding; /**< 同会话未处理条数。 */
 uint32_t queue_drop; /**< 本会话队满丢条数。 */uint32_t write_drop; /**< 格式化/出队写错/终止丢条数。 */
 uint32_t started_ms; /**< 开始毫秒。 */uint32_t finished_ms; /**< Saved/Partial/Error时毫秒。 */
 Px4Lite_Result_t result; /**< 最后动作结果。 */Px4Lite_Result_t reason; /**< 错误首因，OK无错误。 */
 uint8_t state; /**< RecordingState。 */uint8_t interval; /**< 间隔编号。 */uint8_t pending; /**< 动作待处理。 */uint8_t reserved; /**< 对齐。 */
 uint8_t sd_state; /**< 本机SD服务状态。 */uint8_t card_type; /**< 卡类型。 */uint8_t open_phase; /**< 最近SD错误阶段。 */uint8_t fatfs_result; /**< 最近真实FatFs错误码。 */
} Px4Lite_RecordingStatus_t;
/** @brief 固定请求槽，仅由storage执行。 */
Px4Lite_Result_t Px4Lite_RequestRecording(const Px4Lite_RecordingRequest_t *request);
/** @brief 原子复制已发布会话，不访问SD。 */
Px4Lite_Result_t Px4Lite_CopyRecording(Px4Lite_RecordingStatus_t *out);

/**
 * @brief 模块初始化回调：复位记录队列和 SD 服务状态。
 *
 * @retval PX4LITE_OK 初始化请求完成。
 */
Px4Lite_Result_t Px4Lite_StorageModuleInit(void);

/**
 * @brief 模块恢复回调：仅请求重新挂载。
 *
 * @details
 * 该函数运行在 Health 任务上下文，只设置请求标志并立即返回。实际 SD 重新初始化由
 * storage 任务执行，避免在 Health 任务内进行阻塞总线 I/O。
 *
 * @retval PX4LITE_OK 恢复请求已记录。
 */
Px4Lite_Result_t Px4Lite_StorageRecover(void);

/**
 * @brief 执行一次有界存储工作周期。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 */
void Px4Lite_StorageWorkRun(uint32_t now_ms);

#endif
