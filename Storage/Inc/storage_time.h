/**
 * @file storage_time.h
 * @brief Storage 层时间缓存接口，用于日志日期和 FatFs 文件时间。
 */

#ifndef STORAGE_TIME_H
#define STORAGE_TIME_H

#include <stdint.h>

void Storage_TimeInit(void);
void Storage_TimeUpdate(uint32_t local_date_ymd, uint32_t local_time_hhmmss, uint8_t valid);
uint8_t Storage_TimeIsValid(void);
uint32_t Storage_TimeDateYmd(void);
uint32_t Storage_TimeTimeHhmmss(void);
uint32_t Storage_TimeGetFatFsTime(void);

#endif
