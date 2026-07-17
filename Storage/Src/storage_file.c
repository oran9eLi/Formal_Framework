/**
 * @file storage_file.c
 * @brief 生成兼容 FatFs 8.3 模式的日期日志文件名。
 */

#include "storage_file.h"

#include <stdio.h>

#include "storage_config.h"

Px4Lite_Result_t StorageFile_FormatPath(Storage_RecordType_t type, uint32_t date_ymd, char *path, size_t path_size)
{
  uint32_t yy;
  uint32_t mm;
  uint32_t dd;
  char suffix;
  int written;

  if ((path == 0) || (path_size == 0U)) { return PX4LITE_INVALID_PARAM; }

  suffix = (type == STORAGE_RECORD_EVENT) ? STORAGE_EVENT_FILE_SUFFIX : STORAGE_DATA_FILE_SUFFIX;
  if (date_ymd == 0U) {
    written = snprintf(path, path_size, "0:/UNSYNC_%c.CSV", suffix);
    return ((written > 0) && ((size_t)written < path_size)) ? PX4LITE_OK : PX4LITE_OVERFLOW;
  }

  yy = (date_ymd / 10000U) % 100U;
  mm = (date_ymd / 100U) % 100U;
  dd = date_ymd % 100U;
  if ((mm < 1U) || (mm > 12U) || (dd < 1U) || (dd > 31U)) { return PX4LITE_INVALID_PARAM; }

  written = snprintf(path, path_size, "0:/%02lu%02lu%02lu_%c.CSV",
                     (unsigned long)yy,
                     (unsigned long)mm,
                     (unsigned long)dd,
                     suffix);

  return ((written > 0) && ((size_t)written < path_size)) ? PX4LITE_OK : PX4LITE_OVERFLOW;
}
