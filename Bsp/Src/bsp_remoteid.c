/**
 * @file bsp_remoteid.c
 * @brief 实现 ESP32-S3 RemoteID 的 UART4 DMA 原始发送通道。
 *
 * @details
 * 本文件属于 BSP 层，只管理 UART4、DMA1 Stream4 和静态发送缓冲区。MAVLink/OpenDroneID
 * 编码、发送周期、身份字段和 GNSS 数据读取均由 Framework 层完成。
 */

#include "bsp_remoteid.h"
#include "bsp_config.h"
#include <string.h>

#if BSP_REMOTEID_ENABLE

static UART_HandleTypeDef s_remoteid_uart;
static DMA_HandleTypeDef s_remoteid_tx_dma;
static uint8_t s_remoteid_tx_buf[BSP_REMOTEID_TX_BUF_SIZE];
static volatile uint8_t s_remoteid_tx_busy;
static volatile uint8_t s_remoteid_initialized;

/**
 * @brief 返回 RemoteID UART4 句柄。
 */
UART_HandleTypeDef *BSP_RemoteId_GetUartHandle(void)
{
  return &s_remoteid_uart;
}

/**
 * @brief 初始化 RemoteID 使用的 UART4 和 TX DMA。
 */
int32_t BSP_RemoteId_Init(void)
{
  if (s_remoteid_initialized != 0U) {
    BSP_RemoteId_AbortTx();
    (void)HAL_UART_DeInit(&s_remoteid_uart);
    (void)HAL_DMA_DeInit(&s_remoteid_tx_dma);
    s_remoteid_initialized = 0U;
  }

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_UART4_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* PC8 ESP32 在位检测：下拉输入，ESP32 上电时其 3.3V 把该脚拉高。 */
  {
    GPIO_InitTypeDef detect_gpio;
    detect_gpio.Pin   = BSP_ESP_DETECT_PIN;
    detect_gpio.Mode  = GPIO_MODE_INPUT;
    detect_gpio.Pull  = GPIO_PULLDOWN;
    detect_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BSP_ESP_DETECT_PORT, &detect_gpio);
  }

  s_remoteid_uart.Instance          = BSP_REMOTEID_UART;
  s_remoteid_uart.Init.BaudRate     = BSP_REMOTEID_UART_BAUD;
  s_remoteid_uart.Init.WordLength   = BSP_REMOTEID_UART_WORD;
  s_remoteid_uart.Init.StopBits     = BSP_REMOTEID_UART_STOP;
  s_remoteid_uart.Init.Parity       = BSP_REMOTEID_UART_PARITY;
  s_remoteid_uart.Init.Mode         = BSP_REMOTEID_UART_MODE;
  s_remoteid_uart.Init.HwFlowCtl    = BSP_REMOTEID_UART_HWCTL;
  s_remoteid_uart.Init.OverSampling = BSP_REMOTEID_UART_OVERSAMP;
  if (HAL_UART_Init(&s_remoteid_uart) != HAL_OK) {
    s_remoteid_tx_busy = 0U;
    return -1;
  }

  s_remoteid_tx_dma.Instance                 = BSP_REMOTEID_TX_DMA_STREAM;
  s_remoteid_tx_dma.Init.Channel             = BSP_REMOTEID_TX_DMA_CHANNEL;
  s_remoteid_tx_dma.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  s_remoteid_tx_dma.Init.PeriphInc           = DMA_PINC_DISABLE;
  s_remoteid_tx_dma.Init.MemInc              = DMA_MINC_ENABLE;
  s_remoteid_tx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  s_remoteid_tx_dma.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  s_remoteid_tx_dma.Init.Mode                = DMA_NORMAL;
  s_remoteid_tx_dma.Init.Priority            = DMA_PRIORITY_MEDIUM;
  s_remoteid_tx_dma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&s_remoteid_tx_dma) != HAL_OK) {
    (void)HAL_UART_DeInit(&s_remoteid_uart);
    s_remoteid_tx_busy = 0U;
    return -1;
  }

  __HAL_LINKDMA(&s_remoteid_uart, hdmatx, s_remoteid_tx_dma);
  s_remoteid_tx_busy = 0U;
  s_remoteid_initialized = 1U;
  return 0;
}

/**
 * @brief 启动一次 RemoteID 非阻塞 DMA 发送。
 */
int32_t BSP_RemoteId_StartSend(const uint8_t *data, uint16_t len)
{
  if ((data == 0) || (len == 0U) || (len > BSP_REMOTEID_TX_BUF_SIZE)) { return -1; }
  if (s_remoteid_initialized == 0U) { return -1; }
  if (s_remoteid_tx_busy != 0U) { return 1; }

  memcpy(s_remoteid_tx_buf, data, len);
  s_remoteid_tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(&s_remoteid_uart, s_remoteid_tx_buf, len) != HAL_OK) {
    s_remoteid_tx_busy = 0U;
    return -1;
  }
  return 0;
}

/**
 * @brief 查询 RemoteID TX DMA 是否忙。
 */
uint8_t BSP_RemoteId_IsTxBusy(void)
{
  return s_remoteid_tx_busy;
}

uint8_t BSP_RemoteId_IsReady(void)
{
  return s_remoteid_initialized;
}

uint8_t BSP_RemoteId_IsPresent(void)
{
  return (HAL_GPIO_ReadPin(BSP_ESP_DETECT_PORT, BSP_ESP_DETECT_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

/**
 * @brief 中止当前 RemoteID TX DMA。
 */
void BSP_RemoteId_AbortTx(void)
{
  if (s_remoteid_initialized != 0U) { (void)HAL_UART_AbortTransmit(&s_remoteid_uart); }
  s_remoteid_tx_busy = 0U;
}

/**
 * @brief 处理 RemoteID TX DMA 中断。
 */
void BSP_RemoteId_TxDmaIrqHandler(void)
{
  HAL_DMA_IRQHandler(&s_remoteid_tx_dma);
}

/**
 * @brief 处理 UART TX 完成回调。
 */
void BSP_RemoteId_TxCompleteCallback(UART_HandleTypeDef *huart)
{
  if ((huart != 0) && (huart->Instance == BSP_REMOTEID_UART)) { s_remoteid_tx_busy = 0U; }
}

#else

UART_HandleTypeDef *BSP_RemoteId_GetUartHandle(void) { return 0; }
int32_t BSP_RemoteId_Init(void) { return 0; }
int32_t BSP_RemoteId_StartSend(const uint8_t *data, uint16_t len) { (void)data; (void)len; return -1; }
uint8_t BSP_RemoteId_IsTxBusy(void) { return 0U; }
uint8_t BSP_RemoteId_IsReady(void) { return 0U; }
uint8_t BSP_RemoteId_IsPresent(void) { return 0U; }
void BSP_RemoteId_AbortTx(void) {}
void BSP_RemoteId_TxDmaIrqHandler(void) {}
void BSP_RemoteId_TxCompleteCallback(UART_HandleTypeDef *huart) { (void)huart; }

#endif
