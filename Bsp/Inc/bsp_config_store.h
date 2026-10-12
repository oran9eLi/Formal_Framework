/**
 * @file bsp_config_store.h
 * @brief BSP 层内部 Flash 配置保留区原始读写边界。
 * @details 仅处理 sector 11 的字节读取与单字编程；不解析记录、不擦除、不判断供电或电机状态。
 */
#ifndef BSP_CONFIG_STORE_H
#define BSP_CONFIG_STORE_H
#include <stdint.h>
#define BSP_CONFIG_STORE_BASE 0x080E0000UL /**< 配置区起始地址，单位 byte。 */
#define BSP_CONFIG_STORE_SIZE 0x00020000UL /**< 保留容量 128 KiB。 */
/** @brief 读取保留区内有界字节；成功返回 0，越界返回 -1。 */
int32_t BSP_ConfigStore_Read(uint32_t offset, uint8_t *out, uint16_t length);
/** @brief 在空白对齐字编程；0 为完成，1 为忙，-1 为错误。仅由 CommTask 调用，不擦除。 */
int32_t BSP_ConfigStore_ProgramWord(uint32_t offset, uint32_t value);
/** @brief 收取已发起编程并重新锁定控制器；0 完成、1 忙、-1 失败，无 pending 返回 0。 */
int32_t BSP_ConfigStore_FinishWord(void);
#endif
