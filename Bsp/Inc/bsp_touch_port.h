/**
 * @file bsp_touch_port.h
 * @brief Declare the software I2C GPIO port used by the GT911 touch driver.
 */

#ifndef BSP_TOUCH_PORT_H
#define BSP_TOUCH_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BSP_TOUCH_PORT_OK = 0,
    BSP_TOUCH_PORT_ERROR
} BSP_TouchPortResult_t;

/**
 * @brief Initialize the GPIO resources used by the software I2C touch port.
 */
BSP_TouchPortResult_t BSP_TouchPort_Init(void);
/**
 * @brief Drive the touch reset line high or low.
 */
void BSP_TouchPort_WriteReset(uint8_t level);
/**
 * @brief Drive the touch INT line during GT911 address selection.
 */
void BSP_TouchPort_SetIntOutput(uint8_t level);
/**
 * @brief Restore the touch INT line to input mode.
 */
void BSP_TouchPort_SetIntInput(void);
/**
 * @brief Read the current touch INT line level.
 */
uint8_t BSP_TouchPort_ReadInt(void);
/**
 * @brief Read the current touch SCL line level.
 */
uint8_t BSP_TouchPort_ReadScl(void);
/**
 * @brief Read the current touch SDA line level.
 */
uint8_t BSP_TouchPort_ReadSda(void);
/**
 * @brief Recover a stuck software I2C bus by clocking SCL.
 */
void BSP_TouchPort_Recover(void);
/**
 * @brief Write bytes to an 8-bit addressed touch register.
 */
BSP_TouchPortResult_t BSP_TouchPort_WriteReg8(
    uint8_t addr,
    uint8_t reg,
    const uint8_t *buf,
    uint8_t len);
/**
 * @brief Read bytes from an 8-bit addressed touch register.
 */
BSP_TouchPortResult_t BSP_TouchPort_ReadReg8(
    uint8_t addr,
    uint8_t reg,
    uint8_t *buf,
    uint8_t len);
/**
 * @brief Write bytes to a 16-bit addressed touch register.
 */
BSP_TouchPortResult_t BSP_TouchPort_WriteReg16(
    uint8_t addr,
    uint16_t reg,
    const uint8_t *buf,
    uint8_t len);
/**
 * @brief Read bytes from a 16-bit addressed touch register.
 */
BSP_TouchPortResult_t BSP_TouchPort_ReadReg16(
    uint8_t addr,
    uint16_t reg,
    uint8_t *buf,
    uint8_t len);
/**
 * @brief Return the last low-level software I2C diagnostic error code.
 */
uint8_t BSP_TouchPort_GetLastError(void);

#ifdef __cplusplus
}
#endif

#endif
