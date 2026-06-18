/**
 * @file storage_config.h
 * @brief Configure SD CSV logging periods, buffers, and task resources.
 */

#ifndef STORAGE_CONFIG_H
#define STORAGE_CONFIG_H

#include "FreeRTOS.h"
#include "task.h"

#define STORAGE_TASK_STACK_WORDS 768U
/*
 * Lowest runnable priority — strictly below biz_display (idle+1). The logger
 * does blocking SD/SPI I/O (f_mount/disk_initialize can busy-poll, and it
 * retries every STORAGE_MOUNT_RETRY_MS when no card is mounted). At idle+1 it
 * time-slices against the display task and starves the touch scan, leaving the
 * UI stuck on the LOGO page. Keeping it at idle priority lets the display and
 * touch always preempt the logger; storage still runs in the gaps when the
 * 10ms display task is asleep on vTaskDelayUntil. (N1 hid this because its
 * display task was a NOT_READY stub that did no work.)
 */
#define STORAGE_TASK_PRIORITY      (tskIDLE_PRIORITY)
#define STORAGE_SERVICE_PERIOD_MS  50U
#define STORAGE_DATA_PERIOD_MS     1000U
#define STORAGE_SYNC_PERIOD_MS     5000U
#define STORAGE_MOUNT_RETRY_MS     2000U
#define STORAGE_SPI_TIMEOUT_MS     50U
#define STORAGE_SD_CS_PROBE_LOW_MS 0U
#define STORAGE_QUEUE_LENGTH       8U
#define STORAGE_CSV_LINE_MAX       192U

#define STORAGE_SENSOR_DATA_FILE   "SENSOR.CSV"
#define STORAGE_SYSTEM_ERROR_FILE  "ERROR.CSV"

#endif
