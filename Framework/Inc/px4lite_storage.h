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
