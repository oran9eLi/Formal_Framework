/**
 * @file bsp_lora.c
 * @brief Implement USART3 DMA circular reception and E22 control pins for LoRa.
 */

#include "bsp_lora.h"
#include "bsp_config.h"
#include "stm32f4xx_hal.h"
#include <string.h>

static UART_HandleTypeDef s_lora_uart;
static DMA_HandleTypeDef hdma_lora_rx;
static DMA_HandleTypeDef hdma_lora_tx;

/**
 * @brief Return the private LoRa UART handle for interrupt and MSP use only.
 */
UART_HandleTypeDef *BSP_LoRa_GetUartHandle(void)
{
    return &s_lora_uart;
}

static uint8_t s_rx_buf[BSP_LORA_RX_BUF_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint32_t s_rx_overflow_count;

static uint8_t s_tx_buf[BSP_LORA_TX_BUF_SIZE];
static volatile uint8_t s_tx_busy;

/**
 * @brief Publish the current circular-DMA write position to the software ring.
 */
static void BSP_LoRa_UpdateRxPosition(void)
{
    uint16_t position;
    uint16_t produced;
    uint16_t unread;
    uint16_t free_space;

    position = (uint16_t)(BSP_LORA_RX_BUF_SIZE -
                          (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_lora_rx));
    if (position >= BSP_LORA_RX_BUF_SIZE) {
        position = 0U;
    }
    if (position == s_rx_head) {
        return;
    }

    produced = (position >= s_rx_head)
                   ? (uint16_t)(position - s_rx_head)
                   : (uint16_t)(BSP_LORA_RX_BUF_SIZE -
                                s_rx_head + position);
    unread = (s_rx_head >= s_rx_tail)
                 ? (uint16_t)(s_rx_head - s_rx_tail)
                 : (uint16_t)(BSP_LORA_RX_BUF_SIZE -
                              s_rx_tail + s_rx_head);
    free_space = (uint16_t)(BSP_LORA_RX_BUF_SIZE - 1U - unread);

    if (produced > free_space) {
        s_rx_overflow_count++;
        s_rx_tail = (uint16_t)((position + 1U) %
                               BSP_LORA_RX_BUF_SIZE);
    }
    s_rx_head = position;
}

int32_t BSP_LoRa_Init(void)
{
    GPIO_InitTypeDef gpio;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* The HAL_UART_MspInit(UART3) callback handles PB10/PB11 AF7, DMA, NVIC
       and is already registered in stm32f4xx_hal_msp.c.  This function only
       fills runtime parameters that neither the .h constants nor the MSP can
       provide (baud rate, word length, etc.). */

    s_lora_uart.Instance          = BSP_LORA_UART;
    s_lora_uart.Init.BaudRate     = BSP_LORA_UART_BAUD;
    s_lora_uart.Init.WordLength   = BSP_LORA_UART_WORD;
    s_lora_uart.Init.StopBits     = BSP_LORA_UART_STOP;
    s_lora_uart.Init.Parity       = BSP_LORA_UART_PARITY;
    s_lora_uart.Init.Mode         = BSP_LORA_UART_MODE;
    s_lora_uart.Init.HwFlowCtl    = BSP_LORA_UART_HWCTL;
    s_lora_uart.Init.OverSampling = BSP_LORA_UART_OVERSAMP;
    if (HAL_UART_Init(&s_lora_uart) != HAL_OK) {
        return -1;
    }

    hdma_lora_rx.Instance                 = BSP_LORA_RX_DMA_STREAM;
    hdma_lora_rx.Init.Channel             = BSP_LORA_RX_DMA_CHANNEL;
    hdma_lora_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_lora_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_lora_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_lora_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_lora_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_lora_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_lora_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;
    hdma_lora_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_lora_rx) != HAL_OK) {
        return -1;
    }

    __HAL_LINKDMA(&s_lora_uart, hdmarx, hdma_lora_rx);

    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_overflow_count = 0U;

    if (HAL_UART_Receive_DMA(&s_lora_uart,
                            s_rx_buf,
                            BSP_LORA_RX_BUF_SIZE) != HAL_OK) {
        return -1;
    }
    __HAL_UART_ENABLE_IT(&s_lora_uart, UART_IT_IDLE);

    /* USART3 TX DMA: memory-to-peripheral, single-shot (normal mode). The
       transfer is started on demand by BSP_LoRa_StartSend(); it must not run
       in circular mode. */
    hdma_lora_tx.Instance                 = BSP_LORA_TX_DMA_STREAM;
    hdma_lora_tx.Init.Channel             = BSP_LORA_TX_DMA_CHANNEL;
    hdma_lora_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_lora_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_lora_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_lora_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_lora_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_lora_tx.Init.Mode                = DMA_NORMAL;
    hdma_lora_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
    hdma_lora_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_lora_tx) != HAL_OK) {
        return -1;
    }
    __HAL_LINKDMA(&s_lora_uart, hdmatx, hdma_lora_tx);
    s_tx_busy = 0U;

    /* E22 mode pins: M0=PF1, M1=PF2 */
    __HAL_RCC_GPIOF_CLK_ENABLE();

    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = BSP_LORA_M0_PIN;
    HAL_GPIO_Init(BSP_LORA_M0_PORT, &gpio);

    gpio.Pin = BSP_LORA_M1_PIN;
    HAL_GPIO_Init(BSP_LORA_M1_PORT, &gpio);

    /* AUX pin: PF0, input */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin  = BSP_LORA_AUX_PIN;
    HAL_GPIO_Init(BSP_LORA_AUX_PORT, &gpio);

    return 0;
}

void BSP_LoRa_SetMode(uint8_t m)
{
    HAL_GPIO_WritePin(BSP_LORA_M0_PORT, BSP_LORA_M0_PIN,
                      (m & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BSP_LORA_M1_PORT, BSP_LORA_M1_PIN,
                      (m & 2U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t BSP_LoRa_IsReady(void)
{
    /* E22 AUX is high when the module is ready for a new operation. */
    return (HAL_GPIO_ReadPin(BSP_LORA_AUX_PORT,
                             BSP_LORA_AUX_PIN) != GPIO_PIN_RESET) ? 1U : 0U;
}

uint8_t BSP_LoRa_IsBusy(void)
{
    return (BSP_LoRa_IsReady() != 0U) ? 0U : 1U;
}

uint16_t BSP_LoRa_GetRxData(uint8_t *dst, uint16_t max_len)
{
    uint16_t count = 0U;
    uint32_t primask;

    if (dst == 0 || max_len == 0U) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    BSP_LoRa_UpdateRxPosition();

    while ((s_rx_tail != s_rx_head) && (count < max_len)) {
        dst[count] = s_rx_buf[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % BSP_LORA_RX_BUF_SIZE);
        count++;
    }

    if (primask == 0U) {
        __enable_irq();
    }

    return count;
}

uint16_t BSP_LoRa_GetRxCount(void)
{
    uint16_t count;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    BSP_LoRa_UpdateRxPosition();

    count = (s_rx_head >= s_rx_tail)
                ? (uint16_t)(s_rx_head - s_rx_tail)
                : (uint16_t)(BSP_LORA_RX_BUF_SIZE - s_rx_tail + s_rx_head);

    if (primask == 0U) {
        __enable_irq();
    }

    return count;
}

uint32_t BSP_LoRa_GetRxOverflowCount(void)
{
    return s_rx_overflow_count;
}

int32_t BSP_LoRa_StartSend(const uint8_t *data, uint16_t len)
{
    if (data == 0 || len == 0U || len > BSP_LORA_TX_BUF_SIZE) {
        return -1;
    }
    if (s_tx_busy != 0U) {
        return 1;
    }

    /* The DMA reads from s_tx_buf for the whole transfer, so the frame must be
       copied out of the caller's buffer, which it is free to reuse on return. */
    memcpy(s_tx_buf, data, len);
    s_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(&s_lora_uart, s_tx_buf, len) != HAL_OK) {
        s_tx_busy = 0U;
        return -1;
    }
    return 0;
}

uint8_t BSP_LoRa_IsTxBusy(void)
{
    return s_tx_busy;
}

void BSP_LoRa_AbortTx(void)
{
    (void)HAL_UART_AbortTransmit(&s_lora_uart);
    s_tx_busy = 0U;
}

void BSP_LoRa_TxDmaIrqHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_lora_tx);
}

/* Invoked from the USART3 IRQ once the final byte has shifted out (UART TC).
   Only USART3 transmits via DMA in this project; the debug UART uses blocking
   transmit and never reaches this callback. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == BSP_LORA_UART) {
        s_tx_busy = 0U;
    }
}

void BSP_LoRa_RxIdleCallback(uint16_t dummy)
{
    (void)dummy;
    BSP_LoRa_UpdateRxPosition();
}

void BSP_LoRa_DmaIrqHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_lora_rx);
    BSP_LoRa_UpdateRxPosition();
}

void BSP_LoRa_RecoverRx(void)
{
    volatile uint32_t tmp;

    __HAL_UART_DISABLE_IT(&s_lora_uart, UART_IT_IDLE);

    tmp = s_lora_uart.Instance->SR;
    tmp = s_lora_uart.Instance->DR;
    (void)tmp;

    s_lora_uart.ErrorCode = HAL_UART_ERROR_NONE;

    (void)HAL_UART_DMAStop(&s_lora_uart);

    s_rx_head = 0U;
    s_rx_tail = 0U;

    (void)HAL_UART_Receive_DMA(&s_lora_uart,
                                s_rx_buf,
                                BSP_LORA_RX_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&s_lora_uart, UART_IT_IDLE);
}
