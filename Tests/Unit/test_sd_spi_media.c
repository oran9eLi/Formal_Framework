/**
 * @file test_sd_spi_media.c
 * @brief 主机侧验证生产 SD SPI 初始化及身份检查，用可控卡响应替代 BSP。
 * @details 不使用真实卡，不模拟文件系统。检查无响应、瞬时读错和更换介质后的写保护。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Storage/Src/diskio_sd_spi.c"

static uint8_t reply[8], reply_count, reply_pos;
static uint8_t selected, missing, cid_faults, changed, corrupt_one_read;
static unsigned cid_reads, write_commands;
static uint32_t clock_ms;

/** @brief 为一次 SPI 命令排入响应，剩余时钟返回总线空闲值。 */
static void Reply(const uint8_t *bytes, uint8_t count)
{
  memcpy(reply, bytes, count);
  reply_count = count;
  reply_pos = 0U;
}

void BSP_SPI1_Init(void) {}
BSP_Status_t BSP_SPI1_SetSpeedLow(void) { return BSP_STATUS_OK; }
BSP_Status_t BSP_SPI1_SetSpeedHigh(void) { return BSP_STATUS_OK; }
uint32_t BSP_Time_GetTickMs(void) { return clock_ms++; }
void BSP_SPI1_SD_Select(void) { selected = 1U; }
void BSP_SPI1_SD_Deselect(void) { selected = 0U; reply_pos = reply_count = 0U; }
uint32_t Storage_TimeGetFatFsTime(void) { return 0U; }

/** @brief 模拟一张 SDHC 卡的初始化、CID 数据令牌以及缺卡时持续的 0xFF。 */
BSP_Status_t BSP_SPI1_Transmit(const uint8_t *tx, uint16_t length, uint32_t timeout_ms)
{
  static const uint8_t idle[] = {1U}, ok[] = {0U}, cid[] = {0U, 0xFEU};
  static const uint8_t r7[] = {1U, 0U, 0U, 1U, 0xAAU}, ocr[] = {0U, 0x40U, 0U, 0U, 0U};
  (void)timeout_ms;
  assert(selected && length == 6U);
  switch (tx[0] & 63U) {
    case SD_CMD0: case SD_CMD55: Reply(idle, sizeof(idle)); break;
    case SD_CMD8: Reply(r7, sizeof(r7)); break;
    case 41U: Reply(ok, sizeof(ok)); break;
    case SD_CMD58: Reply(ocr, sizeof(ocr)); break;
    case SD_CMD10:
      ++cid_reads;
      if (cid_faults) { --cid_faults; return BSP_STATUS_TIMEOUT; }
      Reply(cid, sizeof(cid));
      break;
    case SD_CMD24: ++write_commands; break;
    default: assert(0);
  }
  return BSP_STATUS_OK;
}

/** @brief 在卡返回 CID 时检查发送出的填充字节，随后写入指定的 16 字节身份。 */
BSP_Status_t BSP_SPI1_Receive(uint8_t *rx, uint16_t length, uint32_t timeout_ms)
{
  uint16_t i;
  (void)timeout_ms;
  assert(selected && !missing && length == 16U);
  for (i = 0U; i < length; ++i) { assert(rx[i] == 0xFFU); rx[i] = (uint8_t)(i + changed); }
  if (corrupt_one_read) { rx[1] ^= 1U; corrupt_one_read = 0U; }
  return BSP_STATUS_OK;
}

/** @brief 只在已选中的卡实际响应时驱动 MISO。 */
BSP_Status_t BSP_SPI1_TransmitReceive(const uint8_t *tx, uint8_t *rx, uint16_t length, uint32_t timeout_ms)
{
  (void)timeout_ms;
  assert(length == 1U && *tx == 0xFFU);
  *rx = selected && !missing && reply_pos < reply_count ? reply[reply_pos++] : 0xFFU;
  return BSP_STATUS_OK;
}

/** @brief 验证生产入口按顺序处理启动、瞬时错误、空闲拔出、重新初始化与替换卡。 */
int main(void)
{
  uint8_t sector[SD_BLOCK_SIZE] = {0};
  assert(disk_initialize(0) == 0U && DiskioSdSpi_CardType() == (SD_TYPE_SD2 | SD_TYPE_BLOCK));
  assert(DiskioSdSpi_CheckMedia() && cid_reads == 2U && !selected);
  cid_faults = 1U;
  assert(DiskioSdSpi_CheckMedia() && cid_reads == 4U && !selected);
  missing = 1U;
  assert(!DiskioSdSpi_CheckMedia() && cid_reads == 6U && !selected);
  assert(DiskioSdSpi_LastError() == DISKIO_SD_SPI_ERR_CID_READ && DiskioSdSpi_LastResponse() == 0xFFU);
  assert(disk_status(0) == STA_NOINIT && disk_write(0, sector, 0, 1) == RES_NOTRDY);
  missing = 0U;
  cid_faults = 1U;
  assert(disk_initialize(0) == STA_NOINIT && DiskioSdSpi_CardType() == 0U);
  assert(disk_initialize(0) == 0U && DiskioSdSpi_CheckMedia());
  corrupt_one_read = 1U;
  assert(DiskioSdSpi_CheckMedia() && !selected && disk_status(0) == 0U);
  changed = 1U;
  assert(!DiskioSdSpi_CheckMedia() && !selected && DiskioSdSpi_LastError() == DISKIO_SD_SPI_ERR_CID_CHANGED);
  assert(disk_write(0, sector, 0, 1) == RES_NOTRDY && write_commands == 0U);
  assert(disk_initialize(0) == 0U && DiskioSdSpi_CheckMedia());
  puts("SD SPI media PASS: init / read retry / floating MISO / removal / replacement / stale write guard");
  return 0;
}
