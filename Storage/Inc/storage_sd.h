/**
 * @file storage_sd.h
 * @brief Declare FatFS-backed SD card logging service.
 */

#ifndef STORAGE_SD_H
#define STORAGE_SD_H

#include <stdint.h>
#include "px4lite_types.h"

typedef struct
{
    Px4Lite_State_t state;
    uint8_t mounted;
    uint8_t files_open;
    uint8_t last_sync_ok;
    uint8_t disk_error;
    uint8_t card_type;
    uint8_t mount_result;
    uint8_t last_command;
    uint8_t last_response;
    uint8_t open_phase;
    uint8_t open_result;
    uint8_t reserved[2];
    uint32_t written_count;
    uint32_t error_count;
    uint32_t last_write_ms;
    uint32_t last_attempt_ms;
} Storage_SdStatus_t;

void Storage_SD_Init(void);
void Storage_SD_Service(uint32_t now_ms);
Px4Lite_Result_t Storage_SD_WriteDataLine(const char *line,
                                          uint32_t now_ms);
Px4Lite_Result_t Storage_SD_WriteErrorLine(const char *line,
                                           uint32_t now_ms);
Px4Lite_Result_t Storage_SD_Sync(uint32_t now_ms);
void Storage_SD_CopyStatus(Storage_SdStatus_t *out);
uint8_t Storage_SD_IsReady(void);

#endif
