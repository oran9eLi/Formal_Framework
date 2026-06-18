/**
 * @file diskio_sd_spi.h
 * @brief Declare SD SPI diskio helpers.
 */

#ifndef DISKIO_SD_SPI_H
#define DISKIO_SD_SPI_H

#include <stdint.h>

typedef enum {
  DISKIO_SD_SPI_ERR_NONE = 0,
  DISKIO_SD_SPI_ERR_BAD_DRIVE,
  DISKIO_SD_SPI_ERR_CMD0,
  DISKIO_SD_SPI_ERR_CMD8_ECHO,
  DISKIO_SD_SPI_ERR_ACMD41,
  DISKIO_SD_SPI_ERR_CMD58,
  DISKIO_SD_SPI_ERR_CMD16
} DiskioSdSpi_Error_t;

uint8_t DiskioSdSpi_IsInitialized(void);
uint8_t DiskioSdSpi_CardType(void);
DiskioSdSpi_Error_t DiskioSdSpi_LastError(void);
uint8_t DiskioSdSpi_LastCommand(void);
uint8_t DiskioSdSpi_LastResponse(void);

#endif
