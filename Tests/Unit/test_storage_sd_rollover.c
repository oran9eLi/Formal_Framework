#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "diskio_sd_spi.h"
#include "ff.h"
#include "storage_sd.h"

static char s_open_paths[8][24];
static uint8_t s_open_count;
static uint8_t s_close_count;
static uint8_t s_sync_count;
static uint8_t s_write_count;

static int ExpectUint32(const char *label, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("%s: expected %lu, got %lu\n", label, (unsigned long)expected, (unsigned long)actual);
    return 0;
  }
  return 1;
}

static int ExpectString(const char *label, const char *actual, const char *expected)
{
  if (strcmp(actual, expected) != 0) {
    printf("%s: expected %s, got %s\n", label, expected, actual);
    return 0;
  }
  return 1;
}

FRESULT f_mount(FATFS *fs, const TCHAR *path, BYTE opt)
{
  (void)fs;
  (void)path;
  (void)opt;
  return FR_OK;
}

FRESULT f_open(FIL *fp, const TCHAR *path, BYTE mode)
{
  (void)mode;
  if (s_open_count < 8U) {
    (void)snprintf(s_open_paths[s_open_count], sizeof(s_open_paths[s_open_count]), "%s", path);
  }
  s_open_count++;
  fp->fsize = 0U;
  fp->fptr  = 0U;
  return FR_OK;
}

FRESULT f_close(FIL *fp)
{
  (void)fp;
  s_close_count++;
  return FR_OK;
}

FRESULT f_lseek(FIL *fp, DWORD ofs)
{
  fp->fptr = ofs;
  return FR_OK;
}

FRESULT f_write(FIL *fp, const void *buff, UINT btw, UINT *bw)
{
  (void)buff;
  fp->fptr += btw;
  fp->fsize += btw;
  *bw = btw;
  s_write_count++;
  return FR_OK;
}

FRESULT f_sync(FIL *fp)
{
  (void)fp;
  s_sync_count++;
  return FR_OK;
}

uint8_t DiskioSdSpi_IsInitialized(void) { return 1U; }
uint8_t DiskioSdSpi_CardType(void) { return 1U; }
DiskioSdSpi_Error_t DiskioSdSpi_LastError(void) { return DISKIO_SD_SPI_ERR_NONE; }
uint8_t DiskioSdSpi_LastCommand(void) { return 0U; }
uint8_t DiskioSdSpi_LastResponse(void) { return 0U; }

static void ResetFakes(void)
{
  memset(s_open_paths, 0, sizeof(s_open_paths));
  s_open_count = 0U;
  s_close_count = 0U;
  s_sync_count = 0U;
  s_write_count = 0U;
}

static int TestDataFileRollsByRecordDate(void)
{
  int ok = 1;

  ResetFakes();
  Storage_SD_Init();
  Storage_SD_Service(10U);

  ok &= (Storage_SD_WriteDataLine("d1\r\n", 20260622U, 20U) == PX4LITE_OK);
  ok &= (Storage_SD_WriteDataLine("d2\r\n", 20260623U, 30U) == PX4LITE_OK);
  ok &= ExpectUint32("open count", s_open_count, 2U);
  ok &= ExpectString("first data path", s_open_paths[0], "0:/260622_D.CSV");
  ok &= ExpectString("second data path", s_open_paths[1], "0:/260623_D.CSV");
  ok &= ExpectUint32("data switch closes old file", s_close_count, 1U);
  return ok;
}

static int TestUnsyncedDataFileRollsToDatedFile(void)
{
  int ok = 1;

  ResetFakes();
  Storage_SD_Init();
  Storage_SD_Service(10U);

  ok &= (Storage_SD_WriteDataLine("d0\r\n", 0U, 20U) == PX4LITE_OK);
  ok &= (Storage_SD_WriteDataLine("d1\r\n", 20260622U, 30U) == PX4LITE_OK);
  ok &= ExpectUint32("unsynced data open count", s_open_count, 2U);
  ok &= ExpectString("unsynced data path", s_open_paths[0], "0:/UNSYNC_D.CSV");
  ok &= ExpectString("dated data path", s_open_paths[1], "0:/260622_D.CSV");
  ok &= ExpectUint32("unsynced switch closes old file", s_close_count, 1U);
  return ok;
}

static int TestEventFileUsesEventPrefix(void)
{
  int ok = 1;

  ResetFakes();
  Storage_SD_Init();
  Storage_SD_Service(10U);

  ok &= (Storage_SD_WriteEventLine("e1\r\n", 20260622U, 20U) == PX4LITE_OK);
  ok &= ExpectUint32("event open count", s_open_count, 1U);
  ok &= ExpectString("event path", s_open_paths[0], "0:/260622_E.CSV");
  return ok;
}

static int TestUnsyncedEventFileUsesEventSuffix(void)
{
  int ok = 1;

  ResetFakes();
  Storage_SD_Init();
  Storage_SD_Service(10U);

  ok &= (Storage_SD_WriteEventLine("e0\r\n", 0U, 20U) == PX4LITE_OK);
  ok &= ExpectUint32("unsynced event open count", s_open_count, 1U);
  ok &= ExpectString("unsynced event path", s_open_paths[0], "0:/UNSYNC_E.CSV");
  return ok;
}

int main(void)
{
  int ok = 1;

  ok &= TestDataFileRollsByRecordDate();
  ok &= TestUnsyncedDataFileRollsToDatedFile();
  ok &= TestEventFileUsesEventPrefix();
  ok &= TestUnsyncedEventFileUsesEventSuffix();

  if (ok != 0) {
    printf("storage sd rollover tests passed\n");
    return 0;
  }

  return 1;
}
