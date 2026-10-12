/**
 * @file core_fatal.h
 * @brief Core 不可恢复错误入口；只依赖标准整数类型，禁止引入 HAL/RTOS/业务层。
 */
#ifndef CORE_FATAL_H
#define CORE_FATAL_H
#include <stdint.h>
/** @brief 禁止中断并立即撤收 PWM；可重复调用，不恢复正常运行。 */
void Core_FatalQuiesce(void);
/** @brief 撤收输出、清理 DMA 后永久停机，等待独立看门狗或外部复位。 */
void Core_FatalHalt(void);
/**
 * @brief 异常现场复制前检查连续 RAM 范围，避免解引用明显损坏的栈指针。
 * @param[in] address 首地址；仅支持本板 SRAM/CCM。
 * @param[in] bytes 所需字节数，必须大于 0。
 * @return 1 表示范围合法；不能证明 RAM 内容正确或硬件无总线故障。
 */
static inline uint8_t Core_FatalRamRangeValid(uintptr_t address, uint32_t bytes)
{
  if ((bytes == 0U) || (bytes > 0x20000UL)) { return 0U; }
  return (((address >= 0x20000000UL) && (address <= (0x20020000UL - bytes))) ||
          ((bytes <= 0x10000UL) && (address >= 0x10000000UL) && (address <= (0x10010000UL - bytes)))) ? 1U : 0U;
}
#endif
