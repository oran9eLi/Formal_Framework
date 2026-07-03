/**
 * @file storage_sd.c
 * @brief Implement FatFS-backed SD card logging service.
 */

#include "storage_sd.h"

#include <string.h>
#include "bsp_critical.h"
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
  uint32_t primask;

  primask = BSP_Critical_Enter();
  s_status.state           = state;
  s_status.last_attempt_ms = now_ms;
  BSP_Critical_Exit(primask);
}

static Px4Lite_Result_t Storage_SD_MapResult(FRESULT result)
{
  return (result == FR_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

static void Storage_SD_UpdateDiskDiag(FRESULT mount_result)
{
  uint8_t disk_error;
  uint8_t card_type;
  uint8_t last_command;
  uint8_t last_response;
  uint32_t primask;

  disk_error    = (uint8_t)DiskioSdSpi_LastError();
  card_type     = DiskioSdSpi_CardType();
  last_command  = DiskioSdSpi_LastCommand();
  last_response = DiskioSdSpi_LastResponse();

  primask = BSP_Critical_Enter();
  s_status.disk_error    = disk_error;
  s_status.card_type     = card_type;
  s_status.mount_result  = (uint8_t)mount_result;
  s_status.last_command  = last_command;
  s_status.last_response = last_response;
  BSP_Critical_Exit(primask);
}

/**
 * @brief 更新文件打开阶段诊断字段。
 */
static void Storage_SD_SetOpenDiag(Storage_OpenPhase_t open_phase, uint8_t open_result)
{
  uint32_t primask;

  primask = BSP_Critical_Enter();
  s_status.open_phase  = (uint8_t)open_phase;
  s_status.open_result = open_result;
  BSP_Critical_Exit(primask);
}

/**
 * @brief 更新 SD 挂载标志。
 */
static void Storage_SD_SetMounted(uint8_t mounted)
{
  uint32_t primask;

  primask = BSP_Critical_Enter();
  s_status.mounted = mounted;
  BSP_Critical_Exit(primask);
}

/**
 * @brief 更新文件打开标志。
 */
static void Storage_SD_SetFilesOpen(uint8_t files_open)
{
  uint32_t primask;

  primask = BSP_Critical_Enter();
  s_status.files_open = files_open;
  BSP_Critical_Exit(primask);
}

/**
 * @brief 更新最近一次 sync 结果。
 */
static void Storage_SD_SetLastSyncOk(uint8_t last_sync_ok)
{
  uint32_t primask;

  primask = BSP_Critical_Enter();
  s_status.last_sync_ok = last_sync_ok;
  BSP_Critical_Exit(primask);
}

/**
 * @brief 累加 SD 服务错误计数。
 */
static void Storage_SD_IncrementError(void)
{
  uint32_t primask;

  primask = BSP_Critical_Enter();
  s_status.error_count++;
  BSP_Critical_Exit(primask);
}

/**
 * @brief 记录一次成功写入。
 */
static void Storage_SD_NoteWriteOk(uint32_t now_ms)
{
  uint32_t primask;

  primask = BSP_Critical_Enter();
  s_status.written_count++;
  s_status.last_write_ms = now_ms;
  BSP_Critical_Exit(primask);
}

static void Storage_SD_CloseFiles(void)
{
  Storage_SdStatus_t status;

  Storage_SD_CopyStatus(&status);
  if (status.files_open != 0U) {
    (void)f_close(&s_data_file);
    (void)f_close(&s_error_file);
  }
  Storage_SD_SetFilesOpen(0U);
}

/**
 * @brief 清理 FatFS 和 diskio 旧状态，为下一次挂载尝试做准备。
 *
 * @note 热插拔或启动无卡失败后，FatFS 与 diskio 可能保留旧文件句柄、挂载态
 *       或初始化失败态。每次重新挂载前先统一清理，避免插卡后仍卡在旧红灯。
 */
static void Storage_SD_PrepareMountAttempt(void)
{
  Storage_SD_CloseFiles();
  (void)f_mount(0, "0:", 0U);
  DiskioSdSpi_Reset();
  memset(&s_fatfs, 0, sizeof(s_fatfs));
  memset(&s_data_file, 0, sizeof(s_data_file));
  memset(&s_error_file, 0, sizeof(s_error_file));
  Storage_SD_SetMounted(0U);
  Storage_SD_SetFilesOpen(0U);
  Storage_SD_SetLastSyncOk(0U);
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
    Storage_SD_SetOpenDiag(open_phase, (uint8_t)result);
    return Storage_SD_MapResult(result);
  }

  if (f_size(file) == 0U) {
    if (Storage_SD_WriteRaw(file, header) != PX4LITE_OK) {
      (void)f_close(file);
      Storage_SD_SetOpenDiag(header_phase, (uint8_t)FR_DISK_ERR);
      return PX4LITE_IO_ERROR;
    }
  } else {
    result = f_lseek(file, f_size(file));
    if (result != FR_OK) {
      (void)f_close(file);
      Storage_SD_SetOpenDiag(seek_phase, (uint8_t)result);
      return PX4LITE_IO_ERROR;
    }
  }

  return PX4LITE_OK;
}

static Px4Lite_Result_t Storage_SD_OpenFiles(void)
{
  Storage_SD_SetOpenDiag(STORAGE_OPEN_PHASE_NONE, (uint8_t)FR_OK);

  if (Storage_SD_OpenFile(&s_data_file, "0:/" STORAGE_SENSOR_DATA_FILE, StorageCsv_DataHeader(), STORAGE_OPEN_PHASE_DATA_OPEN, STORAGE_OPEN_PHASE_DATA_HEADER, STORAGE_OPEN_PHASE_DATA_SEEK) != PX4LITE_OK) { return PX4LITE_IO_ERROR; }

  if (Storage_SD_OpenFile(&s_error_file, "0:/" STORAGE_SYSTEM_ERROR_FILE, StorageCsv_ErrorHeader(), STORAGE_OPEN_PHASE_ERROR_OPEN, STORAGE_OPEN_PHASE_ERROR_HEADER, STORAGE_OPEN_PHASE_ERROR_SEEK) != PX4LITE_OK) {
    (void)f_close(&s_data_file);
    return PX4LITE_IO_ERROR;
  }

  Storage_SD_SetFilesOpen(1U);
  return PX4LITE_OK;
}

void Storage_SD_Init(void)
{
  Storage_SD_PrepareMountAttempt();
  memset(&s_fatfs, 0, sizeof(s_fatfs));
  memset(&s_data_file, 0, sizeof(s_data_file));
  memset(&s_error_file, 0, sizeof(s_error_file));
  memset(&s_status, 0, sizeof(s_status));
  s_status.state = PX4LITE_STATE_UNINITIALIZED;
}

void Storage_SD_Service(uint32_t now_ms)
{
  FRESULT result;
  Storage_SdStatus_t status;

  Storage_SD_CopyStatus(&status);
  if ((status.state == PX4LITE_STATE_ONLINE) || ((status.last_attempt_ms != 0U) && ((uint32_t)(now_ms - status.last_attempt_ms) < STORAGE_MOUNT_RETRY_MS))) { return; }

  Storage_SD_PrepareMountAttempt();
  Storage_SD_SetState(PX4LITE_STATE_STARTING, now_ms);
  result = f_mount(&s_fatfs, "0:", 1U);
  Storage_SD_UpdateDiskDiag(result);
  if (result != FR_OK) {
    Storage_SD_SetMounted(0U);
    Storage_SD_SetFilesOpen(0U);
    Storage_SD_IncrementError();
    Storage_SD_SetState(PX4LITE_STATE_DEGRADED, now_ms);
    return;
  }
  Storage_SD_SetMounted(1U);

  if (Storage_SD_OpenFiles() != PX4LITE_OK) {
    Storage_SD_UpdateDiskDiag(result);
    Storage_SD_CloseFiles();
    Storage_SD_IncrementError();
    Storage_SD_SetState(PX4LITE_STATE_DEGRADED, now_ms);
    return;
  }

  Storage_SD_SetLastSyncOk(1U);
  Storage_SD_UpdateDiskDiag(FR_OK);
  Storage_SD_SetState(PX4LITE_STATE_ONLINE, now_ms);
}

static Px4Lite_Result_t Storage_SD_WriteLine(FIL *file, const char *line, uint32_t now_ms)
{
  Px4Lite_Result_t result;

  if (Storage_SD_IsReady() == 0U) { return PX4LITE_NOT_READY; }

  result = Storage_SD_WriteRaw(file, line);
  if (result == PX4LITE_OK) {
    Storage_SD_NoteWriteOk(now_ms);
    return PX4LITE_OK;
  }

  Storage_SD_IncrementError();
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
    Storage_SD_SetLastSyncOk(1U);
    return PX4LITE_OK;
  }

  Storage_SD_SetLastSyncOk(0U);
  Storage_SD_IncrementError();
  Storage_SD_CloseFiles();
  Storage_SD_SetState(PX4LITE_STATE_DEGRADED, now_ms);
  return PX4LITE_IO_ERROR;
}

void Storage_SD_CopyStatus(Storage_SdStatus_t *out)
{
  uint32_t primask;

  if (out == 0) { return; }

  primask = BSP_Critical_Enter();
  *out = s_status;
  BSP_Critical_Exit(primask);
}

uint8_t Storage_SD_IsReady(void)
{
  Storage_SdStatus_t status;

  Storage_SD_CopyStatus(&status);
  return ((status.state == PX4LITE_STATE_ONLINE) && (status.mounted != 0U) && (status.files_open != 0U)) ? 1U : 0U;
}
