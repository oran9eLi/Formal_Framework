/**
 * @file diskio_sd_spi.c
 * @brief Implement FatFS diskio over SD card SPI mode.
 */

#include "diskio_sd_spi.h"

#include "diskio.h"
#include "bsp_spi.h"
#include "bsp_time.h"
#include "storage_config.h"

#define SD_DUMMY_BYTE         0xFFU
#define SD_BLOCK_SIZE         512U
#define SD_CMD_TIMEOUT_MS     500U
#define SD_INIT_TIMEOUT_MS    1000U
#define SD_TOKEN_TIMEOUT_MS   200U

#define SD_CMD0               0U
#define SD_CMD1               1U
#define SD_CMD8               8U
#define SD_CMD16              16U
#define SD_CMD17              17U
#define SD_CMD24              24U
#define SD_CMD55              55U
#define SD_CMD58              58U
#define SD_ACMD41             (0x80U + 41U)

#define SD_TOKEN_START_BLOCK  0xFEU
#define SD_DATA_ACCEPTED_MASK 0x1FU
#define SD_DATA_ACCEPTED      0x05U

#define SD_TYPE_MMC           0x01U
#define SD_TYPE_SD1           0x02U
#define SD_TYPE_SD2           0x04U
#define SD_TYPE_BLOCK         0x08U

static DSTATUS s_status = STA_NOINIT;
static uint8_t s_card_type;
static DiskioSdSpi_Error_t s_last_error;
static uint8_t s_last_command;
static uint8_t s_last_response = SD_DUMMY_BYTE;
#if STORAGE_SD_CS_PROBE_LOW_MS > 0U
static uint8_t s_cs_probe_done;
#endif

static uint8_t SdSpi_TransferByte(uint8_t value)
{
  uint8_t rx = SD_DUMMY_BYTE;
  (void)BSP_SPI1_TransmitReceive(&value, &rx, 1U, STORAGE_SPI_TIMEOUT_MS);
  return rx;
}

static void SdSpi_Deselect(void)
{
  BSP_SPI1_SD_Deselect();
  (void)SdSpi_TransferByte(SD_DUMMY_BYTE);
}

static uint8_t SdSpi_WaitReady(uint32_t timeout_ms)
{
  uint32_t start_ms = BSP_Time_GetTickMs();

  do {
    if (SdSpi_TransferByte(SD_DUMMY_BYTE) == SD_DUMMY_BYTE) { return 1U; }
  } while ((uint32_t)(BSP_Time_GetTickMs() - start_ms) < timeout_ms);

  return 0U;
}

static uint8_t SdSpi_Select(void)
{
  BSP_SPI1_SD_Select();
  if (SdSpi_WaitReady(SD_CMD_TIMEOUT_MS) != 0U) { return 1U; }
  SdSpi_Deselect();
  return 0U;
}

static uint8_t SdSpi_SendCommand(uint8_t cmd, uint32_t arg)
{
  uint8_t packet[6];
  uint8_t response;
  uint8_t n;

  if ((cmd & 0x80U) != 0U) {
    cmd &= 0x7FU;
    response = SdSpi_SendCommand(SD_CMD55, 0U);
    SdSpi_Deselect();
    if (response > 1U) { return response; }
  }

  SdSpi_Deselect();
  if (SdSpi_Select() == 0U) {
    s_last_command  = cmd;
    s_last_response = 0xFFU;
    return 0xFFU;
  }

  s_last_command = cmd;
  packet[0]      = (uint8_t)(0x40U | cmd);
  packet[1]      = (uint8_t)(arg >> 24);
  packet[2]      = (uint8_t)(arg >> 16);
  packet[3]      = (uint8_t)(arg >> 8);
  packet[4]      = (uint8_t)arg;
  packet[5]      = 0x01U;
  if (cmd == SD_CMD0) {
    packet[5] = 0x95U;
  } else if (cmd == SD_CMD8) {
    packet[5] = 0x87U;
  }

  (void)BSP_SPI1_Transmit(packet, sizeof(packet), STORAGE_SPI_TIMEOUT_MS);

  for (n = 0U; n < 10U; ++n) {
    response        = SdSpi_TransferByte(SD_DUMMY_BYTE);
    s_last_response = response;
    if ((response & 0x80U) == 0U) { return response; }
  }
  return response;
}

static uint8_t SdSpi_ReadBlock(uint8_t *buffer, uint32_t count)
{
  uint8_t token;
  uint32_t start_ms = BSP_Time_GetTickMs();

  do {
    token = SdSpi_TransferByte(SD_DUMMY_BYTE);
    if (token == SD_TOKEN_START_BLOCK) {
      if (BSP_SPI1_Receive(buffer, (uint16_t)count, STORAGE_SPI_TIMEOUT_MS) != BSP_STATUS_OK) { return 0U; }
      (void)SdSpi_TransferByte(SD_DUMMY_BYTE);
      (void)SdSpi_TransferByte(SD_DUMMY_BYTE);
      return 1U;
    }
  } while ((uint32_t)(BSP_Time_GetTickMs() - start_ms) < SD_TOKEN_TIMEOUT_MS);

  return 0U;
}

static uint8_t SdSpi_WriteBlock(const uint8_t *buffer)
{
  uint8_t token  = SD_TOKEN_START_BLOCK;
  uint8_t crc[2] = {SD_DUMMY_BYTE, SD_DUMMY_BYTE};
  uint8_t response;

  if (SdSpi_WaitReady(SD_CMD_TIMEOUT_MS) == 0U) { return 0U; }

  (void)BSP_SPI1_Transmit(&token, 1U, STORAGE_SPI_TIMEOUT_MS);
  if (BSP_SPI1_Transmit(buffer, SD_BLOCK_SIZE, STORAGE_SPI_TIMEOUT_MS) != BSP_STATUS_OK) { return 0U; }
  (void)BSP_SPI1_Transmit(crc, sizeof(crc), STORAGE_SPI_TIMEOUT_MS);

  response = SdSpi_TransferByte(SD_DUMMY_BYTE);
  if ((response & SD_DATA_ACCEPTED_MASK) != SD_DATA_ACCEPTED) { return 0U; }
  return SdSpi_WaitReady(SD_CMD_TIMEOUT_MS);
}

DSTATUS disk_initialize(BYTE pdrv)
{
  uint8_t response;
  uint8_t ocr[4];
  uint16_t i;
  uint32_t start_ms;

  if (pdrv != 0U) {
    s_last_error = DISKIO_SD_SPI_ERR_BAD_DRIVE;
    return STA_NOINIT;
  }

  s_card_type     = 0U;
  s_status        = STA_NOINIT;
  s_last_error    = DISKIO_SD_SPI_ERR_NONE;
  s_last_command  = 0xFFU;
  s_last_response = 0xFFU;
  BSP_SPI1_Init();
  (void)BSP_SPI1_SetSpeedLow();

#if STORAGE_SD_CS_PROBE_LOW_MS > 0U
  if (s_cs_probe_done == 0U) {
    s_cs_probe_done = 1U;
    BSP_SPI1_SD_Select();
    BSP_Time_DelayMs(STORAGE_SD_CS_PROBE_LOW_MS);
    BSP_SPI1_SD_Deselect();
  }
#endif

  SdSpi_Deselect();

  for (i = 0U; i < 10U; ++i) { (void)SdSpi_TransferByte(SD_DUMMY_BYTE); }

  response = SdSpi_SendCommand(SD_CMD0, 0U);
  SdSpi_Deselect();
  if (response != 1U) {
    s_last_error = DISKIO_SD_SPI_ERR_CMD0;
    return s_status;
  }

  response = SdSpi_SendCommand(SD_CMD8, 0x1AAU);
  if (response == 1U) {
    for (i = 0U; i < 4U; ++i) { ocr[i] = SdSpi_TransferByte(SD_DUMMY_BYTE); }
    SdSpi_Deselect();
    if ((ocr[2] != 0x01U) || (ocr[3] != 0xAAU)) {
      s_last_error = DISKIO_SD_SPI_ERR_CMD8_ECHO;
      return s_status;
    }

    start_ms = BSP_Time_GetTickMs();
    do {
      response = SdSpi_SendCommand(SD_ACMD41, 0x40000000UL);
      SdSpi_Deselect();
      if (response == 0U) { break; }
    } while ((uint32_t)(BSP_Time_GetTickMs() - start_ms) < SD_INIT_TIMEOUT_MS);

    if (response != 0U) {
      s_last_error = DISKIO_SD_SPI_ERR_ACMD41;
      return s_status;
    }

    response = SdSpi_SendCommand(SD_CMD58, 0U);
    if (response != 0U) {
      SdSpi_Deselect();
      s_last_error = DISKIO_SD_SPI_ERR_CMD58;
      return s_status;
    }
    for (i = 0U; i < 4U; ++i) { ocr[i] = SdSpi_TransferByte(SD_DUMMY_BYTE); }
    SdSpi_Deselect();
    s_card_type = (uint8_t)(SD_TYPE_SD2 | ((ocr[0] & 0x40U) != 0U ? SD_TYPE_BLOCK : 0U));
  } else {
    SdSpi_Deselect();
    start_ms = BSP_Time_GetTickMs();
    do {
      response = SdSpi_SendCommand(SD_ACMD41, 0U);
      SdSpi_Deselect();
      if (response == 0U) {
        s_card_type = SD_TYPE_SD1;
        break;
      }
      response = SdSpi_SendCommand(SD_CMD1, 0U);
      SdSpi_Deselect();
      if (response == 0U) {
        s_card_type = SD_TYPE_MMC;
        break;
      }
    } while ((uint32_t)(BSP_Time_GetTickMs() - start_ms) < SD_INIT_TIMEOUT_MS);

    if (s_card_type == 0U) {
      s_last_error = DISKIO_SD_SPI_ERR_ACMD41;
      return s_status;
    }
    response = SdSpi_SendCommand(SD_CMD16, SD_BLOCK_SIZE);
    SdSpi_Deselect();
    if (response != 0U) {
      s_card_type  = 0U;
      s_last_error = DISKIO_SD_SPI_ERR_CMD16;
      return s_status;
    }
  }

  (void)BSP_SPI1_SetSpeedHigh();
  s_status = 0U;
  return s_status;
}

DSTATUS disk_status(BYTE pdrv)
{
  return (pdrv == 0U) ? s_status : STA_NOINIT;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  if ((pdrv != 0U) || (buff == 0) || (count == 0U)) { return RES_PARERR; }
  if ((s_status & STA_NOINIT) != 0U) { return RES_NOTRDY; }
  if ((s_card_type & SD_TYPE_BLOCK) == 0U) { sector *= SD_BLOCK_SIZE; }

  while (count > 0U) {
    if ((SdSpi_SendCommand(SD_CMD17, sector) != 0U) || (SdSpi_ReadBlock(buff, SD_BLOCK_SIZE) == 0U)) {
      SdSpi_Deselect();
      return RES_ERROR;
    }
    SdSpi_Deselect();
    buff += SD_BLOCK_SIZE;
    sector += ((s_card_type & SD_TYPE_BLOCK) != 0U) ? 1U : SD_BLOCK_SIZE;
    count--;
  }
  return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  if ((pdrv != 0U) || (buff == 0) || (count == 0U)) { return RES_PARERR; }
  if ((s_status & STA_NOINIT) != 0U) { return RES_NOTRDY; }
  if ((s_card_type & SD_TYPE_BLOCK) == 0U) { sector *= SD_BLOCK_SIZE; }

  while (count > 0U) {
    if ((SdSpi_SendCommand(SD_CMD24, sector) != 0U) || (SdSpi_WriteBlock(buff) == 0U)) {
      SdSpi_Deselect();
      return RES_ERROR;
    }
    SdSpi_Deselect();
    buff += SD_BLOCK_SIZE;
    sector += ((s_card_type & SD_TYPE_BLOCK) != 0U) ? 1U : SD_BLOCK_SIZE;
    count--;
  }
  return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  if (pdrv != 0U) { return RES_PARERR; }
  if ((s_status & STA_NOINIT) != 0U) { return RES_NOTRDY; }

  switch (cmd) {
    case CTRL_SYNC:
      return (SdSpi_Select() != 0U) ? (SdSpi_Deselect(), RES_OK) : RES_ERROR;
    case GET_SECTOR_SIZE:
      if (buff == 0) { return RES_PARERR; }
      *(WORD *)buff = SD_BLOCK_SIZE;
      return RES_OK;
    case GET_BLOCK_SIZE:
      if (buff == 0) { return RES_PARERR; }
      *(DWORD *)buff = 1U;
      return RES_OK;
    default:
      return RES_PARERR;
  }
}

DWORD get_fattime(void)
{
  return ((DWORD)(2026U - 1980U) << 25) | ((DWORD)1U << 21) | ((DWORD)1U << 16);
}

uint8_t DiskioSdSpi_IsInitialized(void)
{
  return ((s_status & STA_NOINIT) == 0U) ? 1U : 0U;
}

uint8_t DiskioSdSpi_CardType(void)
{
  return s_card_type;
}

DiskioSdSpi_Error_t DiskioSdSpi_LastError(void)
{
  return s_last_error;
}

uint8_t DiskioSdSpi_LastCommand(void)
{
  return s_last_command;
}

uint8_t DiskioSdSpi_LastResponse(void)
{
  return s_last_response;
}
