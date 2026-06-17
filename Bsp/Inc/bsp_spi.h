/**
 * @file bsp_spi.h
 * @brief Declare SPI SD access helpers for SD card logging.
 */

#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stdint.h>
#include "bsp_status.h"

void BSP_SPI1_Init(void);
BSP_Status_t BSP_SPI1_SetSpeedLow(void);
BSP_Status_t BSP_SPI1_SetSpeedHigh(void);
BSP_Status_t BSP_SPI1_Transmit(const uint8_t *tx,
                               uint16_t length,
                               uint32_t timeout_ms);
BSP_Status_t BSP_SPI1_Receive(uint8_t *rx,
                              uint16_t length,
                              uint32_t timeout_ms);
BSP_Status_t BSP_SPI1_TransmitReceive(const uint8_t *tx,
                                      uint8_t *rx,
                                      uint16_t length,
                                      uint32_t timeout_ms);
void BSP_SPI1_SD_Select(void);
void BSP_SPI1_SD_Deselect(void);

#endif
