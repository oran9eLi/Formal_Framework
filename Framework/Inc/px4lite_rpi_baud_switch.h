/**
 * @file px4lite_rpi_baud_switch.h
 * @brief 声明 USART6 上受控的单次临时波特率切换协议。
 *
 * @details
 * 本模块仅处理 MAVLink 2 TUNNEL 0x8003 的类型 3/4。当前版本只接受本机
 * 115200 bit/s 到 57600 bit/s 的切换；切换值只保存在 UART HAL 句柄中，复位后
 * BSP 重新按 115200 bit/s 初始化。CommTask 独占协议状态和串口重初始化。
 */

#ifndef PX4LITE_RPI_BAUD_SWITCH_H
#define PX4LITE_RPI_BAUD_SWITCH_H

#include <stdint.h>

#include "px4lite_types.h"

#define PX4LITE_RPI_BAUD_TUNNEL_TYPE       0x8003U /**< UART6 实验专用 TUNNEL 类型。 */
#define PX4LITE_RPI_BAUD_VERSION           1U      /**< 临时改参载荷版本。 */
#define PX4LITE_RPI_BAUD_REQUEST_TYPE      3U      /**< F407 波特率切换请求。 */
#define PX4LITE_RPI_BAUD_ACCEPT_TYPE       4U      /**< 旧速率下发送的已接受响应。 */
#define PX4LITE_RPI_BAUD_PAYLOAD_LENGTH    14U     /**< 版本、类型、nonce 和波特率总字节数。 */
#define PX4LITE_RPI_BAUD_NONCE_LENGTH      8U      /**< nonce 字节数。 */

/** @brief 切换事务的单次启动状态。 */
typedef enum {
  PX4LITE_RPI_BAUD_STATE_IDLE = 0,       /**< 当前启动周期尚未接受切换。 */
  PX4LITE_RPI_BAUD_STATE_PENDING_TX,     /**< 已排队接受响应，等待旧速率整帧发送完成。 */
  PX4LITE_RPI_BAUD_STATE_APPLIED,        /**< 已发送接受响应并应用 57600 bit/s。 */
  PX4LITE_RPI_BAUD_STATE_APPLY_FAILED    /**< 接受响应已发出，但串口重配置失败。 */
} Px4Lite_RpiBaudSwitchState_t;

/** @brief 待发接受响应的只读视图；指针在发送成功通知前有效。 */
typedef struct {
  const uint8_t *payload; /**< 逐字节编码的 TUNNEL 数据区。 */
  uint16_t payload_type;  /**< 固定为 0x8003。 */
  uint8_t payload_length; /**< 固定为 14 字节。 */
  uint8_t target_system;  /**< 响应目标系统号，取请求来源。 */
  uint8_t target_component; /**< 响应目标组件号，取请求来源。 */
} Px4Lite_RpiBaudSwitchResponseView_t;

/** @brief 波特率切换诊断快照；计数只表示本机处理结果。 */
typedef struct {
  uint32_t accepted_count; /**< 合法切换请求数。 */
  uint32_t rejected_count; /**< 目标、版本、长度、波特率或状态非法数。 */
  uint32_t duplicate_count; /**< 待发期间相同来源和 nonce 的重复请求数。 */
  uint32_t busy_count; /**< 待发期间其他请求数。 */
  uint32_t response_sent_count; /**< 旧速率接受响应整帧发送成功数。 */
  uint32_t apply_success_count; /**< UART6 成功切至 B1 的次数。 */
  uint32_t apply_failure_count; /**< UART6 重配置失败数。 */
  uint32_t current_baud_bps; /**< 最近读取的已应用波特率，单位 bit/s；0 表示不可读。 */
  Px4Lite_RpiBaudSwitchState_t state; /**< 当前切换状态。 */
} Px4Lite_RpiBaudSwitchStats_t;

/** @brief 清空本次启动的切换状态，并确认当前 UART6 处于 B0。 */
Px4Lite_Result_t Px4Lite_RpiBaudSwitchInit(void);

/**
 * @brief 校验并接收一条 UART6 TUNNEL 波特率切换请求。
 * @param[in] source_system 请求来源系统号，不得为 0。
 * @param[in] source_component 请求来源组件号，不得为 0。
 * @param[in] target_system TUNNEL 目标系统号，必须为本机系统号。
 * @param[in] target_component TUNNEL 目标组件号，必须为 UART6 组件号 193。
 * @param[in] local_system 本机系统号。
 * @param[in] payload_type TUNNEL payload_type。
 * @param[in] payload TUNNEL 数据区。
 * @param[in] payload_length TUNNEL 声明的数据区长度，必须恰为 14 字节。
 * @param[in] wire_payload_length MAVLink 实际携带的 TUNNEL 数据字节数；尾随零可被 MAVLink 2 裁剪。
 * @return OK 表示接受响应已排队或待发重复已去重；IDLE 表示其他 TUNNEL 类型。
 */
Px4Lite_Result_t Px4Lite_RpiBaudSwitchHandleTunnel(uint8_t source_system,
                                                   uint8_t source_component,
                                                   uint8_t target_system,
                                                   uint8_t target_component,
                                                   uint8_t local_system,
                                                   uint16_t payload_type,
                                                   const uint8_t *payload,
                                                   uint8_t payload_length,
                                                   uint8_t wire_payload_length);

/** @brief 检查切换接受响应是否正在等待发送。 */
uint8_t Px4Lite_RpiBaudSwitchIsPending(void);

/** @brief 获取待发接受响应，不转移静态缓冲所有权。 */
Px4Lite_Result_t Px4Lite_RpiBaudSwitchPeekResponse(Px4Lite_RpiBaudSwitchResponseView_t *out);

/**
 * @brief 通知旧速率响应整帧发送成功，并在安全点应用 B1。
 * @note 仅由 CommTask 在阻塞 UART 发送返回成功后调用。
 */
Px4Lite_Result_t Px4Lite_RpiBaudSwitchResponseSent(void);

/** @brief 复制当前切换诊断快照并刷新已应用波特率。 */
void Px4Lite_RpiBaudSwitchGetStats(Px4Lite_RpiBaudSwitchStats_t *out);

#endif
