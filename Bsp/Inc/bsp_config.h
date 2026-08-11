/**
 * @file bsp_config.h
 * @brief 定义板级引脚、外设实例、DMA 流和中断资源。
 *
 * @details
 * 本文件只描述板级硬件事实。Framework 模块开关和任务参数不得反向影响 BSP 宏，
 * 以保持 Platform Adapter -> Driver -> BSP -> HAL 的单向依赖。
 */

#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

#include "stm32f4xx_hal.h"

/*
 * BSP 模块编译开关。关闭的外设不会由 BSP_Init() 初始化；源文件被门控后也不占用
 * Flash/RAM。这些宏归 BSP 层所有，不依赖 Framework 的 PX4LITE_* 宏。
 */
#define BSP_ENABLE_I2C    1U
#define BSP_ENABLE_ADC    1U
#define BSP_ENABLE_ADC_CURRENT 1U
#define BSP_ENABLE_GNSS   1U
#define BSP_ENABLE_RTC    1U
#define BSP_ENABLE_PWM    1U
#define BSP_ENABLE_BUTTON 1U
#define BSP_ENABLE_REMOTEID 1U
#define BSP_ENABLE_RPI_UART 1U

/*
 * 独立看门狗(IWDG)。IWDG 由 LSI 驱动，STM32F407 的 LSI 标称 32 kHz，数据手册给出
 * 的实际范围是 17~47 kHz，因此超时选型必须按最快的 47 kHz 校核"最短超时"，否则
 * 会出现偶发误复位。
 *
 * 当前取值：PR=4(64 分频)、RLR=1249，超时 = 64 * (RLR + 1) / f_LSI
 *   f_LSI = 47 kHz(最快) -> 1.70 s  <- 安全下限
 *   f_LSI = 32 kHz(标称) -> 2.50 s
 *   f_LSI = 17 kHz(最慢) -> 4.71 s
 *
 * Health 任务每 100 ms 喂一次，最短超时仍有 17 倍余量。任务卡死到复位的总时延约为
 * 500 ms(心跳超时) + 100 ms(health 周期) + IWDG 超时。
 */
#define BSP_WATCHDOG_PRESCALER_CODE 4U    /**< IWDG_PR：4 表示 64 分频。 */
#define BSP_WATCHDOG_RELOAD         1249U /**< IWDG_RLR：12 位重装值，上限 4095。 */
#define BSP_WATCHDOG_FREEZE_ON_DEBUG 1U   /**< 调试器挂起内核时冻结 IWDG，便于单步。 */

/* 调试 UART：USART1，PA9/PA10，115200 8N1。仅供 DebugConsole 使用，不再复用给树莓派。 */
#define BSP_DBG_UART          USART1
#define BSP_DBG_UART_BAUD     115200U
#define BSP_DBG_UART_WORD     UART_WORDLENGTH_8B
#define BSP_DBG_UART_STOP     UART_STOPBITS_1
#define BSP_DBG_UART_PARITY   UART_PARITY_NONE
#define BSP_DBG_UART_MODE     UART_MODE_TX_RX
#define BSP_DBG_UART_HWCTL    UART_HWCONTROL_NONE
#define BSP_DBG_UART_OVERSAMP UART_OVERSAMPLING_16
#define BSP_DBG_TX_PORT       GPIOA
#define BSP_DBG_TX_PIN        GPIO_PIN_9
#define BSP_DBG_TX_AF         GPIO_AF7_USART1
#define BSP_DBG_RX_PORT       GPIOA
#define BSP_DBG_RX_PIN        GPIO_PIN_10
#define BSP_DBG_RX_AF         GPIO_AF7_USART1

/*
 * 树莓派 MAVLink UART：USART6，PC6(TX)/PC7(RX)，115200 8N1，经 3.3V USB-TTL 连接。
 * TX 仍使用阻塞发送；RX 使用 USART6 中断收字节环形缓冲，不占用 DMA；与调试口 USART1 物理分离。
 */
#define BSP_RPI_UART          USART6
#define BSP_RPI_UART_BAUD     115200U
#define BSP_RPI_UART_WORD     UART_WORDLENGTH_8B
#define BSP_RPI_UART_STOP     UART_STOPBITS_1
#define BSP_RPI_UART_PARITY   UART_PARITY_NONE
#define BSP_RPI_UART_MODE     UART_MODE_TX_RX
#define BSP_RPI_UART_HWCTL    UART_HWCONTROL_NONE
#define BSP_RPI_UART_OVERSAMP UART_OVERSAMPLING_16
#define BSP_RPI_RX_BUF_SIZE   128U
#define BSP_RPI_TX_PORT       GPIOC
#define BSP_RPI_TX_PIN        GPIO_PIN_6
#define BSP_RPI_TX_AF         GPIO_AF8_USART6
#define BSP_RPI_RX_PORT       GPIOC
#define BSP_RPI_RX_PIN        GPIO_PIN_7
#define BSP_RPI_RX_AF         GPIO_AF8_USART6
#define BSP_RPI_IRQn          USART6_IRQn
#define BSP_RPI_IRQ_PRIORITY  6U

/* ATGM336H GNSS：USART2，PA2/PA3，9600 8N1。 */
#define BSP_GNSS_UART           USART2
#define BSP_GNSS_UART_BAUD      9600U
#define BSP_GNSS_UART_WORD      UART_WORDLENGTH_8B
#define BSP_GNSS_UART_STOP      UART_STOPBITS_1
#define BSP_GNSS_UART_PARITY    UART_PARITY_NONE
#define BSP_GNSS_UART_MODE      UART_MODE_TX_RX
#define BSP_GNSS_UART_HWCTL     UART_HWCONTROL_NONE
#define BSP_GNSS_UART_OVERSAMP  UART_OVERSAMPLING_16
#define BSP_GNSS_TX_PORT        GPIOA
#define BSP_GNSS_TX_PIN         GPIO_PIN_2
#define BSP_GNSS_TX_AF          GPIO_AF7_USART2
#define BSP_GNSS_RX_PORT        GPIOA
#define BSP_GNSS_RX_PIN         GPIO_PIN_3
#define BSP_GNSS_RX_AF          GPIO_AF7_USART2

#define BSP_GNSS_RX_DMA_STREAM  DMA1_Stream5
#define BSP_GNSS_RX_DMA_CHANNEL DMA_CHANNEL_4
#define BSP_GNSS_RX_DMA_IRQn    DMA1_Stream5_IRQn
#define BSP_GNSS_RX_BUF_SIZE    512U
#define BSP_GNSS_IRQn           USART2_IRQn
#define BSP_GNSS_IRQ_PRIORITY   6U

/* BME280 I2C 总线：I2C1，PB6/PB7。 */
#define BSP_I2C_INS      I2C1
#define BSP_I2C_SPEED    100000U
#define BSP_I2C_SCL_PORT GPIOB
#define BSP_I2C_SCL_PIN  GPIO_PIN_6
#define BSP_I2C_SCL_AF   GPIO_AF4_I2C1
#define BSP_I2C_SDA_PORT GPIOB
#define BSP_I2C_SDA_PIN  GPIO_PIN_7
#define BSP_I2C_SDA_AF   GPIO_AF4_I2C1

/* 电源采样 ADC：ADC1 IN5，PA5。 */
#define BSP_ADC_INS         ADC1
#define BSP_ADC_CH          ADC_CHANNEL_5
#define BSP_ADC_PORT        GPIOA
#define BSP_ADC_PIN         GPIO_PIN_5
/* 分压系数 = NUM/DEN，用于把引脚电压还原成电池电压。
   2026-07-24 地面站电源模块标定实测值 10.17793941，按 1e-4 精度取整为 101779/10000。
   上一版为标称值 10.080。修改后必须同步 Tests/Unit/test_power_adc_calibration.c 的断言。 */
#define BSP_ADC_DIVIDER_NUM 101779U
#define BSP_ADC_DIVIDER_DEN 10000U

/* 第二块电池电源采样 ADC2：ADC2 IN4，PA4。与电池 1 同规格，分压系数相同。 */
#define BSP_ADC2_INS         ADC2
#define BSP_ADC2_CH          ADC_CHANNEL_4
#define BSP_ADC2_PORT        GPIOA
#define BSP_ADC2_PIN         GPIO_PIN_4
#define BSP_ADC2_DIVIDER_NUM 101779U
#define BSP_ADC2_DIVIDER_DEN 10000U

/* 两路电流计采样 ADC：ADC3 IN10/IN11，PC0/PC1。与电池电压 ADC1/ADC2 分属独立外设，互不冲突。 */
#define BSP_ADC_CURRENT_INS           ADC3
#define BSP_ADC_CURRENT1_CH           ADC_CHANNEL_10
#define BSP_ADC_CURRENT1_PORT         GPIOC
#define BSP_ADC_CURRENT1_PIN          GPIO_PIN_0
#define BSP_ADC_CURRENT2_CH           ADC_CHANNEL_11
#define BSP_ADC_CURRENT2_PORT         GPIOC
#define BSP_ADC_CURRENT2_PIN          GPIO_PIN_1
#define BSP_ADC_CURRENT_SAMPLE_TIME   ADC_SAMPLETIME_480CYCLES
#define BSP_ADC_CURRENT_AVERAGE_COUNT 16U

/* 电机 PWM 输出：TIM3 PA6/PA7、TIM4 PD12/PD13，50 Hz 对应标准 20 ms ESC 周期。 */
#define BSP_PWM_FREQ        50U
#define BSP_PWM_MOTOR1_TIM  TIM3
#define BSP_PWM_MOTOR1_CH   TIM_CHANNEL_1
#define BSP_PWM_MOTOR1_PORT GPIOA
#define BSP_PWM_MOTOR1_PIN  GPIO_PIN_6
#define BSP_PWM_MOTOR1_AF   GPIO_AF2_TIM3
#define BSP_PWM_MOTOR2_TIM  TIM3
#define BSP_PWM_MOTOR2_CH   TIM_CHANNEL_2
#define BSP_PWM_MOTOR2_PORT GPIOA
#define BSP_PWM_MOTOR2_PIN  GPIO_PIN_7
#define BSP_PWM_MOTOR2_AF   GPIO_AF2_TIM3
#define BSP_PWM_MOTOR3_TIM  TIM4
#define BSP_PWM_MOTOR3_CH   TIM_CHANNEL_1
#define BSP_PWM_MOTOR3_PORT GPIOD
#define BSP_PWM_MOTOR3_PIN  GPIO_PIN_12
#define BSP_PWM_MOTOR3_AF   GPIO_AF2_TIM4
#define BSP_PWM_MOTOR4_TIM  TIM4
#define BSP_PWM_MOTOR4_CH   TIM_CHANNEL_2
#define BSP_PWM_MOTOR4_PORT GPIOD
#define BSP_PWM_MOTOR4_PIN  GPIO_PIN_13
#define BSP_PWM_MOTOR4_AF   GPIO_AF2_TIM4

/* 板载按键：WKUP 高电平按下，KEY0/KEY1/KEY2 低电平按下。 */
#define BSP_BTN_WKUP_PORT GPIOA
#define BSP_BTN_WKUP_PIN  GPIO_PIN_0
#define BSP_BTN_KEY0_PORT GPIOE
#define BSP_BTN_KEY0_PIN  GPIO_PIN_4
#define BSP_BTN_KEY1_PORT GPIOE
#define BSP_BTN_KEY1_PIN  GPIO_PIN_3
#define BSP_BTN_KEY2_PORT GPIOE
#define BSP_BTN_KEY2_PIN  GPIO_PIN_2

/* ATK-MD0700 显示屏：SSD1963 类控制器，FSMC 8080 16-bit 总线。 */
#define BSP_DISPLAY_ENABLE              1U
#define BSP_DISPLAY_MODEL_ATK_MD0700    1U
#define BSP_DISPLAY_CONTROLLER_SSD1963  1U
#define BSP_DISPLAY_USE_FSMC_8080       1U
#define BSP_DISPLAY_BUS_WIDTH_BITS      16U
#define BSP_DISPLAY_WIDTH               800U
#define BSP_DISPLAY_HEIGHT              480U
#define BSP_DISPLAY_PIXEL_FORMAT_RGB565 1U
#define BSP_DISPLAY_EXPECTED_PID        0x61U

/* 兼容移植显示代码使用的历史模块开关。 */
#define BSP_DISPLAY_ATK_MD0700_ENABLE BSP_DISPLAY_MODEL_ATK_MD0700

/* FSMC Bank1 NOR/SRAM4，NE4 + A6 用于命令/数据地址选择。 */
#define BSP_LCD_FSMC_BANK_ADDR           0x6C000000UL
#define BSP_LCD_FSMC_REG_SEL             6U
#define BSP_LCD_FSMC_CMD_ADDR            (BSP_LCD_FSMC_BANK_ADDR | (((1UL << BSP_LCD_FSMC_REG_SEL) - 1UL) << 1))
#define BSP_LCD_FSMC_DAT_ADDR            (BSP_LCD_FSMC_BANK_ADDR | ((1UL << BSP_LCD_FSMC_REG_SEL) << 1))
#define BSP_LCD_FSMC_BCR_INDEX           6U
#define BSP_LCD_FSMC_BTR_INDEX           7U
#define BSP_LCD_FSMC_BWTR_INDEX          6U
#define BSP_LCD_FSMC_ADDRESS_SETUP       15U
#define BSP_LCD_FSMC_ADDRESS_SETUP_WRITE 9U
#define BSP_LCD_FSMC_DATA_SETUP_READ     60U
#define BSP_LCD_FSMC_DATA_SETUP_WRITE    8U

#define BSP_LCD_FSMC_GPIO_CLK_ENABLE()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 \
  do {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 \
    __HAL_RCC_GPIOD_CLK_ENABLE();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      \
    __HAL_RCC_GPIOE_CLK_ENABLE();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      \
    __HAL_RCC_GPIOF_CLK_ENABLE();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      \
    __HAL_RCC_GPIOG_CLK_ENABLE();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      \
  } while (0)

/* FSMC 8080 控制线。 */
#define BSP_LCD_FSMC_CS_PORT GPIOG
#define BSP_LCD_FSMC_CS_PIN  GPIO_PIN_12
#define BSP_LCD_FSMC_CS_AF   GPIO_AF12_FSMC
#define BSP_LCD_FSMC_RS_PORT GPIOF
#define BSP_LCD_FSMC_RS_PIN  GPIO_PIN_12
#define BSP_LCD_FSMC_RS_AF   GPIO_AF12_FSMC
#define BSP_LCD_FSMC_RD_PORT GPIOD
#define BSP_LCD_FSMC_RD_PIN  GPIO_PIN_4
#define BSP_LCD_FSMC_RD_AF   GPIO_AF12_FSMC
#define BSP_LCD_FSMC_WR_PORT GPIOD
#define BSP_LCD_FSMC_WR_PIN  GPIO_PIN_5
#define BSP_LCD_FSMC_WR_AF   GPIO_AF12_FSMC

/* FSMC 16-bit 数据总线。 */
#define BSP_LCD_FSMC_D0_PORT  GPIOD
#define BSP_LCD_FSMC_D0_PIN   GPIO_PIN_14
#define BSP_LCD_FSMC_D0_AF    GPIO_AF12_FSMC
#define BSP_LCD_FSMC_D1_PORT  GPIOD
#define BSP_LCD_FSMC_D1_PIN   GPIO_PIN_15
#define BSP_LCD_FSMC_D1_AF    GPIO_AF12_FSMC
#define BSP_LCD_FSMC_D2_PORT  GPIOD
#define BSP_LCD_FSMC_D2_PIN   GPIO_PIN_0
#define BSP_LCD_FSMC_D2_AF    GPIO_AF12_FSMC
#define BSP_LCD_FSMC_D3_PORT  GPIOD
#define BSP_LCD_FSMC_D3_PIN   GPIO_PIN_1
#define BSP_LCD_FSMC_D3_AF    GPIO_AF12_FSMC
#define BSP_LCD_FSMC_D4_PORT  GPIOE
#define BSP_LCD_FSMC_D4_PIN   GPIO_PIN_7
#define BSP_LCD_FSMC_D4_AF    GPIO_AF12_FSMC
#define BSP_LCD_FSMC_D5_PORT  GPIOE
#define BSP_LCD_FSMC_D5_PIN   GPIO_PIN_8
#define BSP_LCD_FSMC_D5_AF    GPIO_AF12_FSMC
#define BSP_LCD_FSMC_D6_PORT  GPIOE
#define BSP_LCD_FSMC_D6_PIN   GPIO_PIN_9
#define BSP_LCD_FSMC_D6_AF    GPIO_AF12_FSMC
#define BSP_LCD_FSMC_D7_PORT  GPIOE
#define BSP_LCD_FSMC_D7_PIN   GPIO_PIN_10
#define BSP_LCD_FSMC_D7_AF    GPIO_AF12_FSMC
#define BSP_LCD_FSMC_D8_PORT  GPIOE
#define BSP_LCD_FSMC_D8_PIN   GPIO_PIN_11
#define BSP_LCD_FSMC_D8_AF    GPIO_AF12_FSMC
#define BSP_LCD_FSMC_D9_PORT  GPIOE
#define BSP_LCD_FSMC_D9_PIN   GPIO_PIN_12
#define BSP_LCD_FSMC_D9_AF    GPIO_AF12_FSMC
#define BSP_LCD_FSMC_D10_PORT GPIOE
#define BSP_LCD_FSMC_D10_PIN  GPIO_PIN_13
#define BSP_LCD_FSMC_D10_AF   GPIO_AF12_FSMC
#define BSP_LCD_FSMC_D11_PORT GPIOE
#define BSP_LCD_FSMC_D11_PIN  GPIO_PIN_14
#define BSP_LCD_FSMC_D11_AF   GPIO_AF12_FSMC
#define BSP_LCD_FSMC_D12_PORT GPIOE
#define BSP_LCD_FSMC_D12_PIN  GPIO_PIN_15
#define BSP_LCD_FSMC_D12_AF   GPIO_AF12_FSMC
#define BSP_LCD_FSMC_D13_PORT GPIOD
#define BSP_LCD_FSMC_D13_PIN  GPIO_PIN_8
#define BSP_LCD_FSMC_D13_AF   GPIO_AF12_FSMC
#define BSP_LCD_FSMC_D14_PORT GPIOD
#define BSP_LCD_FSMC_D14_PIN  GPIO_PIN_9
#define BSP_LCD_FSMC_D14_AF   GPIO_AF12_FSMC
#define BSP_LCD_FSMC_D15_PORT GPIOD
#define BSP_LCD_FSMC_D15_PIN  GPIO_PIN_10
#define BSP_LCD_FSMC_D15_AF   GPIO_AF12_FSMC

/* 显示背光，当前使用普通 GPIO 输出驱动。 */
#define BSP_LCD_BL_PORT              GPIOB
#define BSP_LCD_BL_PIN               GPIO_PIN_15
#define BSP_LCD_BL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BSP_LCD_BL_ACTIVE_LEVEL      GPIO_PIN_SET

/* GT911 电容触摸，使用软件 I2C。 */
#define BSP_TOUCH_ENABLE           1U
#define BSP_TOUCH_CONTROLLER_GT911 1U
#define BSP_TOUCH_USE_SOFT_I2C     1U
#define BSP_TOUCH_SCL_PORT         GPIOB
#define BSP_TOUCH_SCL_PIN          GPIO_PIN_0
#define BSP_TOUCH_SDA_PORT         GPIOF
#define BSP_TOUCH_SDA_PIN          GPIO_PIN_11
#define BSP_TOUCH_INT_PORT         GPIOB
#define BSP_TOUCH_INT_PIN          GPIO_PIN_1
#define BSP_TOUCH_RST_PORT         GPIOC
#define BSP_TOUCH_RST_PIN          GPIO_PIN_13
#define BSP_TOUCH_GPIO_CLK_ENABLE()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    \
  do {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 \
    __HAL_RCC_GPIOB_CLK_ENABLE();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      \
    __HAL_RCC_GPIOC_CLK_ENABLE();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      \
    __HAL_RCC_GPIOF_CLK_ENABLE();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      \
  } while (0)

/* LoRa E22-400T30D：USART3，PB10(TX)/PB11(RX)，9600 8N1；M0=PF1，M1=PF2，AUX=PF0。 */
#define BSP_LORA_ENABLE         1U
#define BSP_LORA_UART           USART3
#define BSP_LORA_UART_BAUD      9600U
#define BSP_LORA_AIR_BPS        9600U
#define BSP_LORA_UART_WORD      UART_WORDLENGTH_8B
#define BSP_LORA_UART_STOP      UART_STOPBITS_1
#define BSP_LORA_UART_PARITY    UART_PARITY_NONE
#define BSP_LORA_UART_MODE      UART_MODE_TX_RX
#define BSP_LORA_UART_HWCTL     UART_HWCONTROL_NONE
#define BSP_LORA_UART_OVERSAMP  UART_OVERSAMPLING_16
#define BSP_LORA_TX_PORT        GPIOB
#define BSP_LORA_TX_PIN         GPIO_PIN_10
#define BSP_LORA_TX_AF          GPIO_AF7_USART3
#define BSP_LORA_RX_PORT        GPIOB
#define BSP_LORA_RX_PIN         GPIO_PIN_11
#define BSP_LORA_RX_AF          GPIO_AF7_USART3

#define BSP_LORA_RX_DMA_STREAM  DMA1_Stream1
#define BSP_LORA_RX_DMA_CHANNEL DMA_CHANNEL_4
#define BSP_LORA_RX_DMA_IRQn    DMA1_Stream1_IRQn
#define BSP_LORA_RX_BUF_SIZE    512U

#define BSP_LORA_TX_DMA_STREAM  DMA1_Stream3
#define BSP_LORA_TX_DMA_CHANNEL DMA_CHANNEL_4
#define BSP_LORA_TX_DMA_IRQn    DMA1_Stream3_IRQn
#define BSP_LORA_TX_BUF_SIZE    280U
#define BSP_LORA_IRQn           USART3_IRQn
#define BSP_LORA_IRQ_PRIORITY   6U

#define BSP_LORA_M0_PORT        GPIOF
#define BSP_LORA_M0_PIN         GPIO_PIN_1
#define BSP_LORA_M1_PORT        GPIOF
#define BSP_LORA_M1_PIN         GPIO_PIN_2
#define BSP_LORA_AUX_PORT       GPIOF
#define BSP_LORA_AUX_PIN        GPIO_PIN_0

/*
 * SD 卡 SPI 总线：SPI3，PB3(SCK)/PB4(MISO)/PB5(MOSI)，PA15 CS。这些引脚在板上
 * 未被其他功能占用；PA15/PB3/PB4 属于 JTAG 引脚，因此片上调试必须只使用 SWD
 * (PA13/PA14)。
 */
#define BSP_SD_SPI_INS          SPI3
#define BSP_SD_SPI_CLK_ENABLE() __HAL_RCC_SPI3_CLK_ENABLE()
#define BSP_SD_SPI_SCK_PORT     GPIOB
#define BSP_SD_SPI_SCK_PIN      GPIO_PIN_3
#define BSP_SD_SPI_SCK_AF       GPIO_AF6_SPI3
#define BSP_SD_SPI_MISO_PORT    GPIOB
#define BSP_SD_SPI_MISO_PIN     GPIO_PIN_4
#define BSP_SD_SPI_MISO_AF      GPIO_AF6_SPI3
#define BSP_SD_SPI_MOSI_PORT    GPIOB
#define BSP_SD_SPI_MOSI_PIN     GPIO_PIN_5
#define BSP_SD_SPI_MOSI_AF      GPIO_AF6_SPI3
#define BSP_SD_CS_PORT          GPIOA
#define BSP_SD_CS_PIN           GPIO_PIN_15

/* ESP32-S3 RemoteID：UART4，PC10(TX)/PC11(RX)，57600 8N1，仅由 comm 任务经平台适配层发送。 */
#define BSP_REMOTEID_ENABLE         BSP_ENABLE_REMOTEID
#define BSP_REMOTEID_UART           UART4
#define BSP_REMOTEID_UART_BAUD      57600U
#define BSP_REMOTEID_UART_WORD      UART_WORDLENGTH_8B
#define BSP_REMOTEID_UART_STOP      UART_STOPBITS_1
#define BSP_REMOTEID_UART_PARITY    UART_PARITY_NONE
#define BSP_REMOTEID_UART_MODE      UART_MODE_TX_RX
#define BSP_REMOTEID_UART_HWCTL     UART_HWCONTROL_NONE
#define BSP_REMOTEID_UART_OVERSAMP  UART_OVERSAMPLING_16
#define BSP_REMOTEID_TX_PORT        GPIOC
#define BSP_REMOTEID_TX_PIN         GPIO_PIN_10
#define BSP_REMOTEID_TX_AF          GPIO_AF8_UART4
#define BSP_REMOTEID_RX_PORT        GPIOC
#define BSP_REMOTEID_RX_PIN         GPIO_PIN_11
#define BSP_REMOTEID_RX_AF          GPIO_AF8_UART4
#define BSP_REMOTEID_TX_DMA_STREAM  DMA1_Stream4
#define BSP_REMOTEID_TX_DMA_CHANNEL DMA_CHANNEL_4
#define BSP_REMOTEID_TX_DMA_IRQn    DMA1_Stream4_IRQn
#define BSP_REMOTEID_TX_BUF_SIZE    300U
#define BSP_REMOTEID_IRQn           UART4_IRQn
#define BSP_REMOTEID_IRQ_PRIORITY   6U

/* ESP32 在位检测：PC8 下拉输入接 ESP32 的 3.3V，高=已插并上电，低=未插/未上电。 */
#define BSP_ESP_DETECT_PORT         GPIOC
#define BSP_ESP_DETECT_PIN          GPIO_PIN_8

/* PC11 is the UART4 RX pin; RemoteID status is based on the local TX channel, not a reply packet. */

#endif
