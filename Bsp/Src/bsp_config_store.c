/**
 * @file bsp_config_store.c
 * @brief BSP 层 sector 11 原始读取与单字 Flash 编程。
 * @details 依赖 STM32 HAL/CMSIS Flash 寄存器；不解析配置、不擦扇区、不关闭中断。
 * 同片 Flash 编程可能暂停取指，单步实际耗时须由实板测量，不能以非阻塞接口保证无暂停。
 */
#include "bsp_config_store.h"
#include "stm32f4xx_hal.h"

static uint8_t s_program_pending;
static uint32_t s_pending_offset;
static uint32_t s_pending_value;
#define CONFIG_FLASH_ERRORS (FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR)

/** @brief 有界读取配置区；不等待写入完成，忙时返回 1。 */
int32_t BSP_ConfigStore_Read(uint32_t offset, uint8_t *out, uint16_t length)
{
  uint16_t i;
  const volatile uint8_t *source;
  if ((out == 0) || (offset > BSP_CONFIG_STORE_SIZE) || ((uint32_t)length > BSP_CONFIG_STORE_SIZE - offset)) { return -1; }
  if ((FLASH->SR & FLASH_SR_BSY) != 0U) { return 1; }
  source = (const volatile uint8_t *)(BSP_CONFIG_STORE_BASE + offset);
  for (i = 0U; i < length; i++) { out[i] = source[i]; }
  return 0;
}

/** @brief 即使上层门禁撤销也收取已启动硬件操作，避免 PG 或解锁状态遗留。 */
int32_t BSP_ConfigStore_FinishWord(void)
{
  uint32_t errors;
  uint32_t actual;
  if (s_program_pending == 0U) { return 0; }
  if ((FLASH->SR & FLASH_SR_BSY) != 0U) { return 1; }
  errors = FLASH->SR & CONFIG_FLASH_ERRORS;
  actual = *(volatile uint32_t *)(BSP_CONFIG_STORE_BASE + s_pending_offset);
  FLASH->CR &= ~FLASH_CR_PG;
  (void)HAL_FLASH_Lock();
  s_program_pending = 0U;
  if (errors != 0U) { FLASH->SR = errors; return -1; }
  return (actual == s_pending_value) ? 0 : -1;
}

/** @brief 每次最多发起一个 32 位编程；完成由 FinishWord 显式收取。 */
int32_t BSP_ConfigStore_ProgramWord(uint32_t offset, uint32_t value)
{
  volatile uint32_t *target;
  if ((offset > BSP_CONFIG_STORE_SIZE - 4U) || ((offset & 3U) != 0U)) { return -1; }
  target = (volatile uint32_t *)(BSP_CONFIG_STORE_BASE + offset);
  if (s_program_pending != 0U) {
    if ((offset != s_pending_offset) || (value != s_pending_value)) { return -1; }
    return BSP_ConfigStore_FinishWord();
  }
  /* 不把其他 Flash 使用者的忙状态误报为本模块已启动写入。 */
  if ((FLASH->SR & FLASH_SR_BSY) != 0U) { return -1; }
  if (*target != UINT32_MAX) { return -1; }
  if (HAL_FLASH_Unlock() != HAL_OK) { return -1; }
  FLASH->SR = CONFIG_FLASH_ERRORS | FLASH_SR_EOP;
  FLASH->CR = (FLASH->CR & ~(FLASH_CR_PSIZE | FLASH_CR_SER | FLASH_CR_MER)) | FLASH_PSIZE_WORD | FLASH_CR_PG;
  s_pending_offset = offset;
  s_pending_value = value;
  s_program_pending = 1U;
  *target = value;
  __DSB();
  return 1;
}
