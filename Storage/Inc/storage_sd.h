/**
 * @file storage_sd.h
 * @brief 基于 FatFs 的 SD 卡日志服务接口。
 *
 * @details
 * SD 服务由低优先级 storage task 调用，独占 FatFs/SD 阻塞 I/O。其他任务不得
 * 直接写 SD，应通过 Storage 队列或上层封装提交完整记录。
 */

#ifndef STORAGE_SD_H
#define STORAGE_SD_H

#include <stdint.h>
#include "px4lite_types.h"

/**
 * @brief SD 日志服务状态。
 */
typedef struct
{
    Px4Lite_State_t state;   /**< Storage 模块公开状态。 */
    uint8_t mounted;         /**< 挂载标志，1 表示 FatFs 已挂载。 */
    uint8_t files_open;      /**< 文件打开标志，1 表示数据/错误文件已打开。 */
    uint8_t last_sync_ok;    /**< 最近一次 sync 成功标志。 */
    uint8_t disk_error;      /**< 磁盘错误标志。 */
    uint8_t card_type;       /**< SD 卡类型，来自底层识别结果。 */
    uint8_t mount_result;    /**< 最近一次挂载结果。 */
    uint8_t last_command;    /**< 最近一次底层 SD 命令。 */
    uint8_t last_response;   /**< 最近一次底层 SD 响应。 */
    uint8_t open_phase;      /**< 文件打开阶段。 */
    uint8_t open_result;     /**< 最近一次文件打开结果。 */
    uint8_t reserved[2];     /**< 保留字段，保持结构体对齐。 */
    uint32_t written_count;  /**< 累计成功写入记录数。 */
    uint32_t error_count;    /**< 累计写入或挂载错误数。 */
    uint32_t last_write_ms;  /**< 最近成功写入时间，单位：ms。 */
    uint32_t last_attempt_ms;/**< 最近一次挂载或写入尝试时间，单位：ms。 */
} Storage_SdStatus_t;

/**
 * @brief 初始化 SD 日志服务状态。
 *
 * @note 实际挂载和文件打开可在 `Storage_SD_Service()` 中按需完成。
 */
void Storage_SD_Init(void);

/**
 * @brief 执行 SD 日志服务周期处理。
 *
 * @param[in] now_ms 当前 storage 周期时间，单位：ms。
 *
 * @note 本函数可执行阻塞 FatFs/SPI I/O，只能在低优先级 storage task 中调用。
 */
void Storage_SD_Service(uint32_t now_ms);

/**
 * @brief 写入一行常规数据 CSV。
 *
 * @param[in] line 完整 CSV 行字符串，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 写入结果。
 */
Px4Lite_Result_t Storage_SD_WriteDataLine(const char *line,
                                          uint32_t now_ms);

/**
 * @brief 写入一行错误记录 CSV。
 *
 * @param[in] line 完整 CSV 行字符串，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 写入结果。
 */
Px4Lite_Result_t Storage_SD_WriteErrorLine(const char *line,
                                           uint32_t now_ms);

/**
 * @brief 同步 SD 文件缓存。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 同步结果。
 */
Px4Lite_Result_t Storage_SD_Sync(uint32_t now_ms);

/**
 * @brief 复制 SD 日志服务状态。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @note 本函数只复制状态，不访问 SD 卡。
 */
void Storage_SD_CopyStatus(Storage_SdStatus_t *out);

/**
 * @brief 判断 SD 日志服务是否可写入。
 *
 * @return 1 表示已挂载且文件已打开，0 表示不可写入。
 */
uint8_t Storage_SD_IsReady(void);

#endif
