/**
 * @file storage_sd.c
 * @brief Implement FatFS-backed SD card logging service.
 */

#include "storage_sd.h"

#include <string.h>
#include "diskio_sd_spi.h"
#include "ff.h"
#include "storage_config.h"
#include "storage_csv.h"

static FATFS s_fatfs;
static FIL s_data_file;
static FIL s_error_file;
static Storage_SdStatus_t s_status;

typedef enum {
  STORAGE_OPEN_PHASE_NONE         = 0,
  STORAGE_OPEN_PHASE_DATA_OPEN    = 1,
  STORAGE_OPEN_PHASE_DATA_HEADER  = 2,
  STORAGE_OPEN_PHASE_DATA_SEEK    = 3,
  STORAGE_OPEN_PHASE_ERROR_OPEN   = 4,
  STORAGE_OPEN_PHASE_ERROR_HEADER = 5,
  STORAGE_OPEN_PHASE_ERROR_SEEK   = 6
} Storage_OpenPhase_t;

static void Storage_SD_SetState(Px4Lite_State_t state, uint32_t now_ms)
{
  s_status.state           = state;
  s_status.last_attempt_ms = now_ms;
}

static Px4Lite_Result_t Storage_SD_MapResult(FRESULT result)
{
  return (result == FR_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

static void Storage_SD_UpdateDiskDiag(FRESULT mount_result)
{
  s_status.disk_error    = (uint8_t)DiskioSdSpi_LastError();
  s_status.card_type     = DiskioSdSpi_CardType();
  s_status.mount_result  = (uint8_t)mount_result;
  s_status.last_command  = DiskioSdSpi_LastCommand();
  s_status.last_response = DiskioSdSpi_LastResponse();
}

static void Storage_SD_CloseFiles(void)
{
  if (s_status.files_open != 0U) {
    (void)f_close(&s_data_file);
    (void)f_close(&s_error_file);
  }
  s_status.files_open = 0U;
}

static Px4Lite_Result_t Storage_SD_WriteRaw(FIL *file, const char *line)
{
  UINT written = 0U;
  UINT length;
  FRESULT result;

  if ((file == 0) || (line == 0)) { return PX4LITE_INVALID_PARAM; }

  length = (UINT)strlen(line);
  result = f_write(file, line, length, &written);
  if ((result != FR_OK) || (written != length)) { return PX4LITE_IO_ERROR; }
  return PX4LITE_OK;
}

static Px4Lite_Result_t Storage_SD_OpenFile(FIL *file, const char *path, const char *header, Storage_OpenPhase_t open_phase, Storage_OpenPhase_t header_phase, Storage_OpenPhase_t seek_phase)
{
  FRESULT result;

  result = f_open(file, path, FA_OPEN_ALWAYS | FA_WRITE);
  if (result != FR_OK) {
    s_status.open_phase  = (uint8_t)open_phase;
    s_status.open_result = (uint8_t)result;
    return Storage_SD_MapResult(result);
  }

  if (f_size(file) == 0U) {
    if (Storage_SD_WriteRaw(file, header) != PX4LITE_OK) {
      (void)f_close(file);
      s_status.open_phase  = (uint8_t)header_phase;
      s_status.open_result = (uint8_t)FR_DISK_ERR;
      return PX4LITE_IO_ERROR;
    }
  } else {
    result = f_lseek(file, f_size(file));
    if (result != FR_OK) {
      (void)f_close(file);
      s_status.open_phase  = (uint8_t)seek_phase;
      s_status.open_result = (uint8_t)result;
      return PX4LITE_IO_ERROR;
    }
  }

  return PX4LITE_OK;
}

static Px4Lite_Result_t Storage_SD_OpenFiles(void)
{
  s_status.open_phase  = STORAGE_OPEN_PHASE_NONE;
  s_status.open_result = (uint8_t)FR_OK;

  if (Storage_SD_OpenFile(&s_data_file, "0:/" STORAGE_SENSOR_DATA_FILE, StorageCsv_DataHeader(), STORAGE_OPEN_PHASE_DATA_OPEN, STORAGE_OPEN_PHASE_DATA_HEADER, STORAGE_OPEN_PHASE_DATA_SEEK) != PX4LITE_OK) { return PX4LITE_IO_ERROR; }

  if (Storage_SD_OpenFile(&s_error_file, "0:/" STORAGE_SYSTEM_ERROR_FILE, StorageCsv_ErrorHeader(), STORAGE_OPEN_PHASE_ERROR_OPEN, STORAGE_OPEN_PHASE_ERROR_HEADER, STORAGE_OPEN_PHASE_ERROR_SEEK) != PX4LITE_OK) {
    (void)f_close(&s_data_file);
    return PX4LITE_IO_ERROR;
  }

  s_status.files_open = 1U;
  return PX4LITE_OK;
}

void Storage_SD_Init(void)
{
  memset(&s_fatfs, 0, sizeof(s_fatfs));
  memset(&s_data_file, 0, sizeof(s_data_file));
  memset(&s_error_file, 0, sizeof(s_error_file));
  memset(&s_status, 0, sizeof(s_status));
  s_status.state = PX4LITE_STATE_UNINITIALIZED;
}

void Storage_SD_Service(uint32_t now_ms)
{
  FRESULT result;

  if ((s_status.state == PX4LITE_STATE_ONLINE) || ((s_status.last_attempt_ms != 0U) && ((uint32_t)(now_ms - s_status.last_attempt_ms) < STORAGE_MOUNT_RETRY_MS))) { return; }

  Storage_SD_SetState(PX4LITE_STATE_STARTING, now_ms);
  result = f_mount(&s_fatfs, "0:", 1U);
  Storage_SD_UpdateDiskDiag(result);
  if (result != FR_OK) {
    s_status.mounted    = 0U;
    s_status.files_open = 0U;
    s_status.error_count++;
    Storage_SD_SetState(PX4LITE_STATE_DEGRADED, now_ms);
    return;
  }
  s_status.mounted = 1U;

  if (Storage_SD_OpenFiles() != PX4LITE_OK) {
    Storage_SD_UpdateDiskDiag(result);
    Storage_SD_CloseFiles();
    s_status.error_count++;
    Storage_SD_SetState(PX4LITE_STATE_DEGRADED, now_ms);
    return;
  }

  s_status.last_sync_ok = 1U;
  Storage_SD_UpdateDiskDiag(FR_OK);
  Storage_SD_SetState(PX4LITE_STATE_ONLINE, now_ms);
}

static Px4Lite_Result_t Storage_SD_WriteLine(FIL *file, const char *line, uint32_t now_ms)
{
  Px4Lite_Result_t result;

  if (Storage_SD_IsReady() == 0U) { return PX4LITE_NOT_READY; }

  result = Storage_SD_WriteRaw(file, line);
  if (result == PX4LITE_OK) {
    s_status.written_count++;
    s_status.last_write_ms = now_ms;
    return PX4LITE_OK;
  }

  s_status.error_count++;
  Storage_SD_CloseFiles();
  Storage_SD_SetState(PX4LITE_STATE_DEGRADED, now_ms);
  return result;
}

Px4Lite_Result_t Storage_SD_WriteDataLine(const char *line, uint32_t now_ms)
{
  return Storage_SD_WriteLine(&s_data_file, line, now_ms);
}

Px4Lite_Result_t Storage_SD_WriteErrorLine(const char *line, uint32_t now_ms)
{
  return Storage_SD_WriteLine(&s_error_file, line, now_ms);
}

Px4Lite_Result_t Storage_SD_Sync(uint32_t now_ms)
{
  FRESULT data_result;
  FRESULT error_result;

  if (Storage_SD_IsReady() == 0U) { return PX4LITE_NOT_READY; }

  data_result  = f_sync(&s_data_file);
  error_result = f_sync(&s_error_file);
  if ((data_result == FR_OK) && (error_result == FR_OK)) {
    s_status.last_sync_ok = 1U;
    return PX4LITE_OK;
  }

  s_status.last_sync_ok = 0U;
  s_status.error_count++;
  Storage_SD_CloseFiles();
  Storage_SD_SetState(PX4LITE_STATE_DEGRADED, now_ms);
  return PX4LITE_IO_ERROR;
}

void Storage_SD_CopyStatus(Storage_SdStatus_t *out)
{
  if (out != 0) { *out = s_status; }
}

uint8_t Storage_SD_IsReady(void)
{
  return ((s_status.state == PX4LITE_STATE_ONLINE) && (s_status.mounted != 0U) && (s_status.files_open != 0U)) ? 1U : 0U;
}
