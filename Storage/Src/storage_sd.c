/**
 * @file storage_sd.c
 * @brief Storage 层 SD 日志服务；由 storage task 独占 FatFs I/O、故障清理和挂载恢复。
 * @details 失败首因在清理前保存，恢复不清零运行统计；禁止其他任务直接操作文件。
 */
#include "storage_sd.h"
#include "px4lite_platform.h"
#include <string.h>
#include "diskio_sd_spi.h"
#include "ff.h"
#include "storage_config.h"
#include "storage_csv.h"
#include "storage_file.h"

static FATFS s_fatfs;
static FIL s_data_file, s_event_file;
static Storage_SdStatus_t s_status;
static uint32_t s_data_file_date, s_event_file_date;
static uint8_t s_data_file_open, s_event_file_open;

/** @brief 故障阶段；保留原有 1..6 诊断编号。 */
typedef enum {
  STORAGE_OPEN_PHASE_NONE = 0,
  STORAGE_OPEN_PHASE_DATA_OPEN = 1, STORAGE_OPEN_PHASE_DATA_HEADER = 2,
  STORAGE_OPEN_PHASE_DATA_SEEK = 3, STORAGE_OPEN_PHASE_ERROR_OPEN = 4,
  STORAGE_OPEN_PHASE_ERROR_HEADER = 5, STORAGE_OPEN_PHASE_ERROR_SEEK = 6,
  STORAGE_OPEN_PHASE_ROTATE_SYNC = 7, STORAGE_OPEN_PHASE_ROTATE_CLOSE = 8,
  STORAGE_OPEN_PHASE_WRITE = 9, STORAGE_OPEN_PHASE_SYNC = 10,
  STORAGE_OPEN_PHASE_MOUNT = 11, STORAGE_OPEN_PHASE_RECOVER_CLOSE = 12
} Storage_OpenPhase_t;

/** @brief 在清理前复制底层诊断，避免 close/unmount 覆盖故障首因。 */
static void Storage_SD_UpdateDiskDiag(void)
{
  s_status.disk_error = (uint8_t)DiskioSdSpi_LastError();
  s_status.card_type = DiskioSdSpi_CardType();
  s_status.last_command = DiskioSdSpi_LastCommand();
  s_status.last_response = DiskioSdSpi_LastResponse();
}

/**
 * @brief 释放文件；主动恢复保存首个 close 错误，已有故障后的清理不覆盖首因。
 * @param[in] record_failure 1 表示主动恢复，可记录新故障；0 表示已保存首因。
 */
static void Storage_SD_CloseFiles(uint8_t record_failure)
{
  FRESULT result;
  uint8_t index;
  for (index = 0U; index < 2U; index++) {
    if (((index == 0U) ? s_data_file_open : s_event_file_open) == 0U) { continue; }
    result = f_close((index == 0U) ? &s_data_file : &s_event_file);
    if ((result != FR_OK) && (record_failure != 0U)) {
      s_status.open_phase = STORAGE_OPEN_PHASE_RECOVER_CLOSE;
      s_status.open_result = (uint8_t)result;
      s_status.short_write = 0U;
      Storage_SD_UpdateDiskDiag();
      s_status.error_count++;
      record_failure = 0U;
    }
  }
  s_data_file_open = 0U; s_event_file_open = 0U;
  s_data_file_date = 0U; s_event_file_date = 0U;
  s_status.files_open = 0U;
}

/** @brief 所有介质操作失败共用的清理、降级和 2 秒恢复退避出口。 */
static Px4Lite_Result_t Storage_SD_Fail(Storage_OpenPhase_t phase, FRESULT result,
                                       uint8_t short_write, uint32_t now_ms)
{
  (void)now_ms;
  s_status.open_phase = (uint8_t)phase;
  s_status.open_result = (uint8_t)result;
  s_status.short_write = short_write;
  Storage_SD_UpdateDiskDiag();
  s_status.error_count++;
  s_status.last_sync_ok = 0U;
  Storage_SD_CloseFiles(0U);
  (void)f_mount(0, "0:", 0U);
  s_status.mounted = 0U;
  s_status.state = PX4LITE_STATE_DEGRADED;
  s_status.last_attempt_ms = Px4Lite_PlatformGetMs();
  return PX4LITE_IO_ERROR;
}

/** @brief 一次完整写入；短写保留真实 FR_OK，并另设 short_write 标志。 */
static Px4Lite_Result_t Storage_SD_WriteRaw(FIL *file, const char *line,
                                           Storage_OpenPhase_t phase, uint32_t now_ms)
{
  UINT written = 0U;
  UINT length = (UINT)strlen(line);
  FRESULT result = f_write(file, line, length, &written);
  if ((result != FR_OK) || (written != length)) {
    return Storage_SD_Fail(phase, result, (uint8_t)((result == FR_OK) && (written != length)), now_ms);
  }
  return PX4LITE_OK;
}

/** @brief 按日期准备目标文件；每个失败阶段均转入同一恢复路径。 */
static Px4Lite_Result_t Storage_SD_EnsureFileForDate(FIL *file, uint8_t *is_open,
    uint32_t *open_date, Storage_RecordType_t type, uint32_t date, uint32_t now_ms)
{
  char path[24];
  const char *header;
  FRESULT result;
  Storage_OpenPhase_t phase;
  /* 参数错误不触碰已打开文件，也不当成介质故障。 */
  if (StorageFile_FormatPath(type, date, path, sizeof(path)) != PX4LITE_OK) { return PX4LITE_INVALID_PARAM; }
  if ((*is_open != 0U) && (*open_date == date)) { return PX4LITE_OK; }
  if (*is_open != 0U) {
    result = f_sync(file);
    if (result != FR_OK) { return Storage_SD_Fail(STORAGE_OPEN_PHASE_ROTATE_SYNC, result, 0U, now_ms); }
    result = f_close(file);
    if (result != FR_OK) { return Storage_SD_Fail(STORAGE_OPEN_PHASE_ROTATE_CLOSE, result, 0U, now_ms); }
    *is_open = 0U; *open_date = 0U;
  }
  phase = (type == STORAGE_RECORD_EVENT) ? STORAGE_OPEN_PHASE_ERROR_OPEN : STORAGE_OPEN_PHASE_DATA_OPEN;
  header = (type == STORAGE_RECORD_EVENT) ? StorageCsv_EventHeader() : StorageCsv_DataHeader();
  result = f_open(file, path, FA_OPEN_ALWAYS | FA_WRITE);
  if (result != FR_OK) { return Storage_SD_Fail(phase, result, 0U, now_ms); }
  /* 立即记录句柄所有权，表头/seek 失败也由统一出口关闭。 */
  *is_open = 1U;
  if (f_size(file) == 0U) {
    if (Storage_SD_WriteRaw(file, header, (Storage_OpenPhase_t)(phase + 1), now_ms) != PX4LITE_OK) { return PX4LITE_IO_ERROR; }
  } else {
    result = f_lseek(file, f_size(file));
    if (result != FR_OK) { return Storage_SD_Fail((Storage_OpenPhase_t)(phase + 2), result, 0U, now_ms); }
  }
  *open_date = date;
  s_status.files_open = 1U;
  return PX4LITE_OK;
}

/** @brief 仅启动时清零状态；运行期必须使用 Recover 保留统计和首因。 */
void Storage_SD_Init(void)
{
  memset(&s_fatfs, 0, sizeof(s_fatfs));
  memset(&s_data_file, 0, sizeof(s_data_file));
  memset(&s_event_file, 0, sizeof(s_event_file));
  memset(&s_status, 0, sizeof(s_status));
  s_data_file_date = 0U; s_event_file_date = 0U;
  s_data_file_open = 0U; s_event_file_open = 0U;
  s_status.state = PX4LITE_STATE_UNINITIALIZED;
}

/** @brief storage task 主动释放并退避；已完成故障清理时不重复 I/O 或重置退避。 */
void Storage_SD_Recover(uint32_t now_ms)
{
  (void)now_ms;
  if (s_status.state == PX4LITE_STATE_DEGRADED) { return; }
  Storage_SD_CloseFiles(1U);
  (void)f_mount(0, "0:", 0U);
  s_status.mounted = 0U;
  s_status.last_sync_ok = 0U;
  s_status.state = PX4LITE_STATE_DEGRADED;
  /* close/unmount 可阻塞，退避必须从全部清理完成的真实时间开始。 */
  s_status.last_attempt_ms = Px4Lite_PlatformGetMs();
}

/** @brief 周期挂载；包含时刻 0 和毫秒回绕在内，失败后至少等待配置退避。 */
void Storage_SD_Service(uint32_t now_ms)
{
  FRESULT result;
  if (s_status.state == PX4LITE_STATE_ONLINE) { return; }
  if ((s_status.state != PX4LITE_STATE_UNINITIALIZED) &&
      (((int32_t)(now_ms - s_status.last_attempt_ms) < 0) ||
       ((uint32_t)(now_ms - s_status.last_attempt_ms) < STORAGE_MOUNT_RETRY_MS))) { return; }
  s_status.state = PX4LITE_STATE_STARTING;
  s_status.last_attempt_ms = now_ms;
  result = f_mount(&s_fatfs, "0:", 1U);
  s_status.mount_result = (uint8_t)result;
  if (result != FR_OK) { (void)Storage_SD_Fail(STORAGE_OPEN_PHASE_MOUNT, result, 0U, now_ms); return; }
  s_status.mounted = 1U;
  s_status.last_sync_ok = 1U;
  s_status.state = PX4LITE_STATE_ONLINE;
}

/** @brief 提交一条记录；失败记录只计一次丢失，不自动重放可能部分落盘的数据。 */
static Px4Lite_Result_t Storage_SD_WriteRecord(const char *line, uint32_t date,
                                               uint32_t now_ms, Storage_RecordType_t type)
{
  Px4Lite_Result_t result;
  FIL *file = (type == STORAGE_RECORD_EVENT) ? &s_event_file : &s_data_file;
  if (line == 0) { result = PX4LITE_INVALID_PARAM; }
  else if (Storage_SD_IsReady() == 0U) { result = PX4LITE_NOT_READY; }
  else {
    result = Storage_SD_EnsureFileForDate(file,
        (type == STORAGE_RECORD_EVENT) ? &s_event_file_open : &s_data_file_open,
        (type == STORAGE_RECORD_EVENT) ? &s_event_file_date : &s_data_file_date, type, date, now_ms);
    if (result == PX4LITE_OK) { result = Storage_SD_WriteRaw(file, line, STORAGE_OPEN_PHASE_WRITE, now_ms); }
  }
  if (result != PX4LITE_OK) { s_status.write_drop_count++; return result; }
  s_status.written_count++;
  s_status.last_write_ms = now_ms;
  return PX4LITE_OK;
}
/** @brief storage task 写入常规 CSV 记录。 */
Px4Lite_Result_t Storage_SD_WriteDataLine(const char *line, uint32_t date, uint32_t now_ms)
{ return Storage_SD_WriteRecord(line, date, now_ms, STORAGE_RECORD_DATA); }
/** @brief storage task 写入事件 CSV 记录。 */
Px4Lite_Result_t Storage_SD_WriteEventLine(const char *line, uint32_t date, uint32_t now_ms)
{ return Storage_SD_WriteRecord(line, date, now_ms, STORAGE_RECORD_EVENT); }
/** @brief 兼容旧错误日志入口。 */
Px4Lite_Result_t Storage_SD_WriteErrorLine(const char *line, uint32_t date, uint32_t now_ms)
{ return Storage_SD_WriteEventLine(line, date, now_ms); }
/** @brief 顺序同步文件，首次失败立即保留诊断并退出。 */
Px4Lite_Result_t Storage_SD_Sync(uint32_t now_ms)
{
  FRESULT result = FR_OK;
  if (Storage_SD_IsReady() == 0U) { return PX4LITE_NOT_READY; }
  if (s_data_file_open != 0U) { result = f_sync(&s_data_file); }
  if ((result == FR_OK) && (s_event_file_open != 0U)) { result = f_sync(&s_event_file); }
  if (result != FR_OK) { return Storage_SD_Fail(STORAGE_OPEN_PHASE_SYNC, result, 0U, now_ms); }
  s_status.last_sync_ok = 1U;
  return PX4LITE_OK;
}
/** @brief 复制同一 storage task 使用的诊断快照，不访问介质。 */
void Storage_SD_CopyStatus(Storage_SdStatus_t *out)
{ if (out != 0) { *out = s_status; } }
/** @brief 返回挂载服务是否可接收记录，文件按日期延迟打开。 */
uint8_t Storage_SD_IsReady(void)
{ return (uint8_t)((s_status.state == PX4LITE_STATE_ONLINE) && (s_status.mounted != 0U)); }
