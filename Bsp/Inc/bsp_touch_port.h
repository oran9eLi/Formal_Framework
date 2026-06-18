/**
 * @file bsp_touch_port.h
 * @brief 声明 GT911 触摸驱动使用的软件 I2C GPIO 端口接口。
 */

#ifndef BSP_TOUCH_PORT_H
#define BSP_TOUCH_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 触摸端口返回码。
 */
typedef enum {
  BSP_TOUCH_PORT_OK = 0,
  BSP_TOUCH_PORT_ERROR
} BSP_TouchPortResult_t;

/**
 * @brief 初始化软件 I2C 触摸端口使用的 GPIO 资源。
 */
BSP_TouchPortResult_t BSP_TouchPort_Init(void);
/**
 * @brief 驱动触摸 reset 引脚高低电平。
 */
void BSP_TouchPort_WriteReset(uint8_t level);
/**
 * @brief 在 GT911 地址选择阶段驱动 INT 引脚。
 */
void BSP_TouchPort_SetIntOutput(uint8_t level);
/**
 * @brief 将触摸 INT 引脚恢复为输入模式。
 */
void BSP_TouchPort_SetIntInput(void);
/**
 * @brief 读取当前触摸 INT 引脚电平。
 */
uint8_t BSP_TouchPort_ReadInt(void);
/**
 * @brief 读取当前触摸 SCL 引脚电平。
 */
uint8_t BSP_TouchPort_ReadScl(void);
/**
 * @brief 读取当前触摸 SDA 引脚电平。
 */
uint8_t BSP_TouchPort_ReadSda(void);
/**
 * @brief 通过 SCL 时钟脉冲恢复卡死的软件 I2C 总线。
 */
void BSP_TouchPort_Recover(void);
/**
 * @brief 向 8-bit 地址触摸寄存器写入字节。
 */
BSP_TouchPortResult_t BSP_TouchPort_WriteReg8(uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len);
/**
 * @brief 从 8-bit 地址触摸寄存器读取字节。
 */
BSP_TouchPortResult_t BSP_TouchPort_ReadReg8(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
/**
 * @brief 向 16-bit 地址触摸寄存器写入字节。
 */
BSP_TouchPortResult_t BSP_TouchPort_WriteReg16(uint8_t addr, uint16_t reg, const uint8_t *buf, uint8_t len);
/**
 * @brief 从 16-bit 地址触摸寄存器读取字节。
 */
BSP_TouchPortResult_t BSP_TouchPort_ReadReg16(uint8_t addr, uint16_t reg, uint8_t *buf, uint8_t len);
/**
 * @brief 返回最近一次底层软件 I2C 诊断错误码。
 */
uint8_t BSP_TouchPort_GetLastError(void);

#ifdef __cplusplus
}
#endif

#endif
