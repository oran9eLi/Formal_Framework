/** @file test_m2_sd_recovery.c @brief SD 文件阶段故障注入；只用于 host。 */
#include <assert.h>
#include <stdio.h>
#include "../../Storage/Src/storage_sd.c"
#include "../../Storage/Src/storage_file.c"
static FRESULT open_result, write_result, seek_result, sync_result, close_result;
static unsigned opens, closes, mounts;
static unsigned short_write, existing_file;
static uint32_t fake_clock_ms, write_delay_ms, close_delay_ms, unmount_delay_ms;
uint32_t Px4Lite_PlatformGetMs(void) { return fake_clock_ms; }
FRESULT f_mount(FATFS *f,const char *p,BYTE v) { (void)p;(void)v; if(f) mounts++; else fake_clock_ms+=unmount_delay_ms; return FR_OK; }
FRESULT f_open(FIL *f,const char *p,BYTE m) { (void)p;(void)m;opens++;f->size=existing_file;return open_result; }
FRESULT f_write(FIL *f,const void *p,UINT n,UINT *w) { (void)f;(void)p;fake_clock_ms+=write_delay_ms;*w=short_write ? n-1:n;return write_result; }
FRESULT f_close(FIL *f) { (void)f;closes++;fake_clock_ms+=close_delay_ms;return close_result; }
FRESULT f_sync(FIL *f) { (void)f;return sync_result; }
FRESULT f_lseek(FIL *f,FSIZE_t n) { (void)f;(void)n;return seek_result; }
DiskioSdSpi_Error_t DiskioSdSpi_LastError(void) { return DISKIO_SD_SPI_ERR_NONE; }
uint8_t DiskioSdSpi_CardType(void) { return 1; }
uint8_t DiskioSdSpi_LastCommand(void) { return 0; }
uint8_t DiskioSdSpi_LastResponse(void) { return 0; }
const char *StorageCsv_DataHeader(void) { return "data\n"; }
const char *StorageCsv_EventHeader(void) { return "event\n"; }
/** @brief 每次故障场景使用全新服务与可控 FatFs 桩。 */
static void Reset(void) {
  open_result=write_result=seek_result=sync_result=close_result=FR_OK;
  short_write=existing_file=opens=closes=mounts=0;
  fake_clock_ms=write_delay_ms=close_delay_ms=unmount_delay_ms=0;
  Storage_SD_Init();Storage_SD_Service(0);
}
/** @brief 核对清理、首因、计数与退避，并在重挂载后实际写一条记录。 */
static void CheckFailure(unsigned phase, FRESULT cause, unsigned expected_close) {
  Storage_SdStatus_t before,after;
  Storage_SD_CopyStatus(&before);
  assert(!Storage_SD_IsReady() && !before.mounted && !before.files_open);
  assert(before.open_phase==phase && before.open_result==cause);
  assert(before.error_count==1 && before.write_drop_count==1 && closes==expected_close);
  Storage_SD_Recover(1000);Storage_SD_CopyStatus(&after);
  assert(after.write_drop_count==before.write_drop_count && after.error_count==before.error_count);
  assert(after.open_result==cause && after.last_attempt_ms==before.last_attempt_ms);
  Storage_SD_Service(before.last_attempt_ms+1999);assert(mounts==1);
  open_result=write_result=seek_result=sync_result=close_result=FR_OK;short_write=0;
  Storage_SD_Service(before.last_attempt_ms+2000);assert(mounts==2 && Storage_SD_IsReady());
  assert(Storage_SD_WriteDataLine("recovered\n",20260929,2500)==PX4LITE_OK);
  Storage_SD_CopyStatus(&after);assert(after.open_result==cause && after.write_drop_count==1);
}
int main(void) {
  unsigned type, stage;
  for(type=0;type<2;type++) for(stage=0;stage<3;stage++) {
    Reset();
    if(stage==0) open_result=FR_DENIED;
    if(stage==1) { write_result=FR_NOT_READY;close_result=FR_DENIED; }
    if(stage==2) { existing_file=1;seek_result=FR_INT_ERR; }
    assert((type ? Storage_SD_WriteEventLine("row\n",20260928,0) :
                   Storage_SD_WriteDataLine("row\n",20260928,0))==PX4LITE_IO_ERROR);
    CheckFailure((type?4:1)+stage,stage==0?FR_DENIED:stage==1?FR_NOT_READY:FR_INT_ERR,stage==0?0:1);
  }
  Reset();short_write=1;assert(Storage_SD_WriteDataLine("row\n",0,0)==PX4LITE_IO_ERROR);
  assert(s_status.short_write==1 && s_status.open_result==FR_OK);CheckFailure(2,FR_OK,1);
  Reset();assert(Storage_SD_WriteDataLine("ok\n",20260928,0)==PX4LITE_OK);
  assert(Storage_SD_WriteEventLine("ok\n",20260928,0)==PX4LITE_OK);
  write_result=FR_DISK_ERR;assert(Storage_SD_WriteDataLine("bad\n",20260928,0)==PX4LITE_IO_ERROR);
  CheckFailure(9,FR_DISK_ERR,2);
  for(stage=0;stage<2;stage++) {
    Reset();assert(Storage_SD_WriteDataLine("ok\n",20260928,0)==PX4LITE_OK);
    if(stage==0) sync_result=FR_NOT_READY;else close_result=FR_DENIED;
    assert(Storage_SD_WriteDataLine("rotate\n",20260929,0)==PX4LITE_IO_ERROR);
    CheckFailure(stage==0?7:8,stage==0?FR_NOT_READY:FR_DENIED,stage==0?1:2);
  }
  Reset();assert(Storage_SD_WriteDataLine("ok\n",20260928,0)==PX4LITE_OK);
  assert(Storage_SD_WriteDataLine("invalid\n",20261328,1)==PX4LITE_INVALID_PARAM);
  assert(Storage_SD_IsReady() && closes==0 && opens==1);
  sync_result=FR_NOT_READY;assert(Storage_SD_Sync(2)==PX4LITE_IO_ERROR);
  assert(s_status.open_phase==10 && s_status.error_count==1 && s_status.write_drop_count==1);
  Reset();assert(Storage_SD_WriteDataLine("ok\n",0,0)==PX4LITE_OK);
  fake_clock_ms=5;Storage_SD_Recover(5);assert(closes==1 && !Storage_SD_IsReady() && s_status.written_count==1);
  Storage_SD_Service(2004);assert(mounts==1);Storage_SD_Service(2005);assert(mounts==2);
  Reset();open_result=FR_DENIED;fake_clock_ms=UINT32_MAX-99;
  assert(Storage_SD_WriteDataLine("wrap\n",0,UINT32_MAX-99)==PX4LITE_IO_ERROR);
  Storage_SD_Service(1899);assert(mounts==1);open_result=FR_OK;Storage_SD_Service(1900);assert(mounts==2);
  Reset();assert(Storage_SD_WriteDataLine("ok\n",0,0)==PX4LITE_OK);
  write_result=FR_DISK_ERR;write_delay_ms=500;close_delay_ms=300;unmount_delay_ms=200;
  assert(Storage_SD_WriteDataLine("bad\n",0,0)==PX4LITE_IO_ERROR);
  assert(s_status.last_attempt_ms==1000 && fake_clock_ms==1000);
  Storage_SD_Service(2999);assert(mounts==1);
  Storage_SD_Service(3000);assert(mounts==2);
  Reset();assert(Storage_SD_WriteDataLine("ok\n",0,0)==PX4LITE_OK);
  close_delay_ms=300;unmount_delay_ms=200;Storage_SD_Recover(0);
  assert(s_status.last_attempt_ms==500);
  Storage_SD_Service(0);assert(mounts==1); /* 同一工作拍携带旧时刻也不能下溢立即重挂载。 */
  Storage_SD_Service(2499);assert(mounts==1);Storage_SD_Service(2500);assert(mounts==2);
  Reset();assert(Storage_SD_WriteDataLine("ok\n",0,0)==PX4LITE_OK);
  assert(Storage_SD_WriteEventLine("ok\n",0,0)==PX4LITE_OK);
  close_result=FR_DENIED;close_delay_ms=300;unmount_delay_ms=200;Storage_SD_Recover(0);
  assert(s_status.error_count==1 && s_status.open_phase==12 && s_status.open_result==FR_DENIED);
  assert(s_status.last_attempt_ms==800 && closes==2 && s_status.written_count==2 && s_status.write_drop_count==0);
  Storage_SD_Recover(1000);assert(closes==2 && s_status.last_attempt_ms==800 && s_status.error_count==1);
  puts("m2_sd_recovery: PASS (all preparation phases, write/sync, first cause, counters, retry/wrap)");return 0;
}
