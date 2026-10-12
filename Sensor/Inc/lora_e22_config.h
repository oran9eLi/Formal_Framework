/**
 * @file lora_e22_config.h
 * @brief Driver 层 E22 寄存器事务接口；由 CommTask 独占调用。
 * @details 不包含 Framework/HAL；正常发送与配置互斥，只有真实读回才报告信道。
 */
#ifndef LORA_E22_CONFIG_H
#define LORA_E22_CONFIG_H
#include <stdint.h>
/** @brief 配置事务结果；PROFILE 表示需通过厂商工具修正课堂基础参数。 */
typedef enum { LORA_CFG_UNKNOWN=0, LORA_CFG_BUSY, LORA_CFG_READY, LORA_CFG_PROFILE,
  LORA_CFG_TIMEOUT, LORA_CFG_IO, LORA_CFG_VERIFY } Lora_ConfigResult_t;
/** @brief 只读 E22 配置事实，不缓存 MCU 推测的信道。 */
typedef struct {
  uint32_t generation; /**< 完成序号，按事务完成递增。 */
  uint32_t request_id; /**< 当前事务号；启动读和显式写各自递增。 */
  uint32_t completed_request_id; /**< 最近结束的事务号，包括失败/会话中止。 */
  Lora_ConfigResult_t completed_result; /**< 最近结束事务结果，不被启动读取覆盖。 */
  uint8_t completed_write; /**< 1 表示最近结束事务为显式 CH 写请求。 */
  uint8_t completed_channel; /**< 最近结束写事务的目标 CH。 */
  uint32_t uart_done; /**< 本次上电 UART TC 次数，包括基础/实验/配置命令。 */
  uint32_t module_idle; /**< 正常发送后 AUX 空闲计数，不证明对端收到。 */
  uint8_t busy; /**< 1 表示正在配置。 */
  uint8_t ready; /**< 1 表示课堂参数有效且已回到正常模式。 */
  uint8_t channel_known; /**< 1 表示当前信道经过寄存器读回。 */
  uint8_t channel; /**< 模块实际信道，范围 0..83；未知时不展示数值。 */
  Lora_ConfigResult_t result; /**< 最近事务结果。 */
} Lora_ConfigSnapshot_t;
/** @brief 每次硬件初始化后启动只读校验，不自动改写参数。 */
void Lora_E22_ConfigInit(uint32_t now_ms);
/** @brief 硬件会话失效时撤销事实，结束旧事务，不保留无界 Busy。 */
void Lora_E22_ConfigInvalidate(void);
/** @brief 请求只写 CH 寄存器；0 接受，1 忙，-1 参数或基础配置不满足。 */
int32_t Lora_E22_ConfigRequest(uint8_t channel,uint32_t now_ms);
/** @brief 有界事务推进；1 忙、0 空闲、-1 正常模式不可确认，需恢复。 */
int32_t Lora_E22_ConfigService(uint32_t now_ms);
/** @brief 获取配置忙标志。 */
uint8_t Lora_E22_ConfigBusy(void);
/** @brief 正常发送准入，配置期间或校验失败为 0。 */
uint8_t Lora_E22_ConfigReady(void);
/** @brief 复制配置事实，调用者不得修改驱动状态。 */
void Lora_E22_CopyConfig(Lora_ConfigSnapshot_t *out);
#endif
