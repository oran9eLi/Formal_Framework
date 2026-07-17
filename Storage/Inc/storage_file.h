/**
 * @file storage_file.h
 * @brief 按 RTC 日期生成 DATA/EVENT 日志文件路径。
 */

#ifndef STORAGE_FILE_H
#define STORAGE_FILE_H

#include <stddef.h>
#include <stdint.h>
#include "px4lite_types.h"
#include "storage_queue.h"

Px4Lite_Result_t StorageFile_FormatPath(Storage_RecordType_t type, uint32_t date_ymd, char *path, size_t path_size);

#endif
