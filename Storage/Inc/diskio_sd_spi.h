/**
 * @file diskio_sd_spi.h
 * @brief Storage 层 FatFs 磁盘适配，通过 BSP 操作 SPI，不判定页面或告警状态。
 */

#ifndef DISKIO_SD_SPI_H
#define DISKIO_SD_SPI_H

#include <stdint.h>

/** @brief SPI 初始化及运行期介质检查的诊断原因，已有编号保持兼容。 */
typedef enum {
  DISKIO_SD_SPI_ERR_NONE = 0,
  DISKIO_SD_SPI_ERR_BAD_DRIVE,
  DISKIO_SD_SPI_ERR_CMD0,
  DISKIO_SD_SPI_ERR_CMD8_ECHO,
  DISKIO_SD_SPI_ERR_ACMD41,
  DISKIO_SD_SPI_ERR_CMD58,
  DISKIO_SD_SPI_ERR_CMD16,
  DISKIO_SD_SPI_ERR_CID_READ,    /**< 卡身份寄存器未能完整读取。 */
  DISKIO_SD_SPI_ERR_CID_CHANGED  /**< 当前卡与本次初始化确认的卡不同。 */
} DiskioSdSpi_Error_t;

uint8_t DiskioSdSpi_IsInitialized(void);
/** @brief storage 任务只读核对卡身份，返回 1 表示仍为当前卡，0 表示必须重新初始化。 */
uint8_t DiskioSdSpi_CheckMedia(void);
uint8_t DiskioSdSpi_CardType(void);
DiskioSdSpi_Error_t DiskioSdSpi_LastError(void);
uint8_t DiskioSdSpi_LastCommand(void);
uint8_t DiskioSdSpi_LastResponse(void);

#endif
