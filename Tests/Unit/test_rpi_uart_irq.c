/**
 * @file test_rpi_uart_irq.c
 * @brief 主机侧验证真实 USART6 BSP 启动与重配的 IRQ 顺序。
 * @details HAL 替身在首次挂接接收时置位 RXNE，开放 NVIC 时立即执行真实 BSP ISR。
 *          实板串口波形和异常标志的清除仍需在 STM32 上验证。
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t SR; } UartRegisters;
typedef struct {
  UartRegisters *Instance;
  struct {
    uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling;
  } Init;
  uint32_t gState;
} UART_HandleTypeDef;

static UartRegisters s_dbg_uart;
static UartRegisters s_rpi_uart;
static uint8_t *s_rx_buffer;
static uint8_t s_irq_enabled;
static unsigned s_failed_arms;
static unsigned s_irq_opens;

#define BSP_CRITICAL_H
#define USART1 (&s_dbg_uart)
#define USART6 (&s_rpi_uart)
#define USART6_IRQn 71
#define USART_SR_PE 0x01U
#define USART_SR_FE 0x02U
#define USART_SR_NE 0x04U
#define USART_SR_ORE 0x08U
#define USART_SR_RXNE 0x20U
#define UART_WORDLENGTH_8B 0U
#define UART_STOPBITS_1 0U
#define UART_PARITY_NONE 0U
#define UART_MODE_TX_RX 0x0CU
#define UART_HWCONTROL_NONE 0U
#define UART_OVERSAMPLING_16 0U
#define HAL_UART_STATE_RESET 0U
#define __HAL_UART_CLEAR_OREFLAG(uart) FakeClearOverrun(uart)

#include "bsp_uart.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart);

static uint32_t BSP_Critical_Enter(void) { return 0U; }
static void BSP_Critical_Exit(uint32_t primask) { (void)primask; }

uint32_t BSP_Time_GetTickMs(void) { return 13U; }

void HAL_NVIC_DisableIRQ(int irq)
{
  assert(irq == BSP_RPI_IRQn);
  s_irq_enabled = 0U;
}

void HAL_NVIC_EnableIRQ(int irq)
{
  assert(irq == BSP_RPI_IRQn);
  assert(s_rx_buffer != NULL);
  assert(!s_irq_enabled);
  s_irq_enabled = 1U;
  s_irq_opens++;
  if ((s_rpi_uart.SR & USART_SR_RXNE) != 0U) {
    BSP_RpiUART_IrqHandler();
    assert((s_rpi_uart.SR & USART_SR_RXNE) == 0U);
  }
}

void HAL_NVIC_ClearPendingIRQ(int irq)
{
  assert(irq == BSP_RPI_IRQn);
  assert(!s_irq_enabled);
}

static HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *uart)
{
  assert(!s_irq_enabled);
  uart->gState = 1U;
  return HAL_OK;
}

static HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
  assert(!s_irq_enabled);
  s_rx_buffer = NULL;
  uart->Instance->SR = 0U;
  return HAL_OK;
}

static HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *uart)
{
  assert(!s_irq_enabled);
  s_rx_buffer = NULL;
  uart->gState = HAL_UART_STATE_RESET;
  return HAL_OK;
}

static HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *dst, uint16_t length)
{
  assert(uart->Instance == BSP_RPI_UART && length == 1U);
  if (s_failed_arms != 0U) {
    assert(!s_irq_enabled);
    s_failed_arms--;
    return HAL_ERROR;
  }
  s_rx_buffer = dst;
  if (!s_irq_enabled) { uart->Instance->SR |= USART_SR_RXNE; }
  return HAL_OK;
}

static HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data,
                                         uint16_t length, uint32_t timeout_ms)
{
  (void)uart;
  (void)data;
  (void)length;
  (void)timeout_ms;
  return HAL_OK;
}

/** @brief 模拟 RM0090 所述的 SR 和 DR 清除序列。 */
void FakeClearOverrun(UART_HandleTypeDef *uart)
{
  assert((uart->Instance->SR & USART_SR_RXNE) == 0U);
  uart->Instance->SR &= ~USART_SR_ORE;
}

/** @brief 交付已经置位的 RXNE，模拟 HAL 先确认中断再调用接收完成回调的顺序。 */
static void HAL_UART_IRQHandler(UART_HandleTypeDef *uart)
{
  assert(s_irq_enabled);
  if (uart->Instance->SR == USART_SR_ORE) {
    /* ORE 单独置位时，HAL 结束接收后直接回调，未读 DR。 */
    s_rx_buffer = NULL;
    HAL_UART_ErrorCallback(uart);
    return;
  }
  assert((uart->Instance->SR & USART_SR_RXNE) != 0U);
  uart->Instance->SR &= ~(USART_SR_RXNE | USART_SR_ORE);
  *s_rx_buffer = 0xA5U;
  HAL_UART_RxCpltCallback(uart);
}

#include "../../Bsp/Src/bsp_uart.c"

/** @brief 使用 BSP 接口确认接收环中的首字节和累计计数。 */
static void CheckReceive(uint32_t baud_bps, uint32_t rx_count)
{
  BSP_RpiUART_Diagnostics_t diagnostics;
  uint8_t byte;

  BSP_RpiUART_CopyDiagnostics(&diagnostics);
  assert(diagnostics.config_valid == 1U && diagnostics.baud_rate == baud_bps);
  assert(diagnostics.rx_byte_count == rx_count);
  assert(diagnostics.rx_seen == 1U && diagnostics.last_rx_ms == 13U);
  assert(BSP_RpiUART_Read(&byte, 1U) == 1U && byte == 0xA5U);
  assert(BSP_RpiUART_Read(&byte, 1U) == 0U);
}

/** @brief 验证启动、切速、失败回滚及过载错误后的接收。 */
int main(void)
{
  BSP_RpiUART_Diagnostics_t diagnostics;
  uint32_t baud;

  s_failed_arms = 1U;
  assert(BSP_RpiUART_Init() == BSP_STATUS_ERROR);
  assert(!s_irq_enabled && s_irq_opens == 0U);

  assert(BSP_RpiUART_Init() == BSP_STATUS_OK);
  assert(s_irq_enabled && s_irq_opens == 1U);
  CheckReceive(BSP_RPI_UART_BAUD, 1U);

  assert(BSP_RpiUART_SetBaudRate(230400U) == BSP_STATUS_OK);
  assert(s_irq_enabled && s_irq_opens == 2U);
  CheckReceive(230400U, 2U);

  s_failed_arms = 1U;
  assert(BSP_RpiUART_SetBaudRate(460800U) == BSP_STATUS_ERROR);
  assert(s_irq_enabled && s_irq_opens == 3U);
  CheckReceive(230400U, 3U);

  s_rpi_uart.SR = USART_SR_ORE;
  BSP_RpiUART_IrqHandler();
  assert(s_rpi_uart.SR == 0U && s_rx_buffer != NULL);
  BSP_RpiUART_CopyDiagnostics(&diagnostics);
  assert(diagnostics.rx_uart_error_count == 1U && diagnostics.rx_byte_count == 3U);

  s_rpi_uart.SR = USART_SR_ORE | USART_SR_RXNE;
  BSP_RpiUART_IrqHandler();
  CheckReceive(230400U, 4U);
  BSP_RpiUART_CopyDiagnostics(&diagnostics);
  assert(diagnostics.rx_uart_error_count == 2U);

  s_failed_arms = 2U;
  assert(BSP_RpiUART_SetBaudRate(460800U) == BSP_STATUS_ERROR);
  assert(!s_irq_enabled && s_irq_opens == 3U);
  assert(BSP_RpiUART_GetBaudRate(&baud) == BSP_STATUS_BUSY);

  puts("Raspberry Pi UART IRQ: initial RX, baud switch, rollback and overrun recovery PASS");
  return 0;
}
