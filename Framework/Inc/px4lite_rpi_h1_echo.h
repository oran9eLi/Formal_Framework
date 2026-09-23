/**
 * @file px4lite_rpi_h1_echo.h
 * @brief 声明 USART6 上的阶段一 H1 最小受控回显协议。
 *
 * @details
 * 仅处理独立的 MAVLink 2 TUNNEL 0x8003 请求/响应；文本作为不透明字节复制。
 * 本模块不解析 UTF-8、不修改串口参数、不写 Flash、不执行飞控命令。
 * CommTask 是静态响应槽和统计状态的唯一所有者。
 */

#ifndef PX4LITE_RPI_H1_ECHO_H
#define PX4LITE_RPI_H1_ECHO_H

#include <stdint.h>

#include "px4lite_types.h"

#define PX4LITE_RPI_H1_TUNNEL_TYPE       0x8003U /**< 与业务 0x8001/0x8002 区分的实验类型。 */
#define PX4LITE_RPI_H1_VERSION           1U      /**< H1 payload 版本。 */
#define PX4LITE_RPI_H1_REQUEST_TYPE      1U      /**< 受控回显请求类型。 */
#define PX4LITE_RPI_H1_RESPONSE_TYPE     2U      /**< 受控回显响应类型。 */
#define PX4LITE_RPI_H1_REQUEST_PREFIX    11U     /**< 版本、类型、nonce 和 N 的字节数。 */
#define PX4LITE_RPI_H1_BAUD_BYTES        4U      /**< 响应尾部实际波特率字节数。 */
#define PX4LITE_RPI_H1_TEXT_MAX_BYTES    48U     /**< 文本最大字节数，不是字符数。 */
#define PX4LITE_RPI_H1_RESPONSE_MAX      (PX4LITE_RPI_H1_REQUEST_PREFIX + PX4LITE_RPI_H1_TEXT_MAX_BYTES + PX4LITE_RPI_H1_BAUD_BYTES)

/** @brief CommTask 待发响应的只读视图；指针在通知发送成功前有效。 */
typedef struct {
  const uint8_t *payload; /**< 已逐字段编码的数据区。 */
  uint16_t payload_type;  /**< 固定为 0x8003。 */
  uint8_t payload_length; /**< 数据区真实长度。 */
  uint8_t target_system;  /**< 回复请求帧的来源 system id。 */
  uint8_t target_component; /**< 回复请求帧的来源 component id。 */
} Px4Lite_RpiH1EchoResponseView_t;

/** @brief H1 诊断计数，不代表 Pi 或 Server 动作成功。 */
typedef struct {
  uint32_t handled_request_count;  /**< 已接受且排队的合法请求数。 */
  uint32_t rejected_request_count; /**< 目标、格式、长度或波特率读取失败数。 */
  uint32_t duplicate_count;        /**< 同一请求在响应待发期间的重复数。 */
  uint32_t busy_count;             /**< 响应槽被其他请求占用次数。 */
  uint32_t response_sent_count;    /**< UART6 阻塞发送成功后的响应数。 */
  uint32_t current_baud_bps;       /**< 最近一次成功读取的已应用配置；0 表示当前不可读。 */
} Px4Lite_RpiH1EchoStats_t;

/** @brief 清空 H1 静态状态并核验 UART6 已应用波特率可读。 */
Px4Lite_Result_t Px4Lite_RpiH1EchoInit(void);

/**
 * @brief 校验并处理一条已通过 MAVLink CRC 的 UART6 TUNNEL 数据区。
 * @param[in] source_system 来源 system id；可与本机 system id 相同，但不得为 0。
 * @param[in] source_component 来源 component id，不得为 0。
 * @param[in] target_system TUNNEL 精确目标 system id，禁止广播。
 * @param[in] target_component TUNNEL 精确目标 component id，禁止广播。
 * @param[in] local_system 本机当前 system id。
 * @param[in] payload_type TUNNEL payload_type。
 * @param[in] payload TUNNEL 数据区只读字节。
 * @param[in] payload_length TUNNEL 声明的数据区字节数。
 * @param[in] wire_payload_length MAVLink 帧中实际携带的数据区字节数，不计 TUNNEL 固定头。
 * @return OK 表示合法响应已排队或相同待发请求已去重；IDLE 表示非 H1 类型。
 */
Px4Lite_Result_t Px4Lite_RpiH1EchoHandleTunnel(uint8_t source_system,
                                               uint8_t source_component,
                                               uint8_t target_system,
                                               uint8_t target_component,
                                               uint8_t local_system,
                                               uint16_t payload_type,
                                               const uint8_t *payload,
                                               uint8_t payload_length,
                                               uint8_t wire_payload_length);

/** @brief 获取待发响应，不转移静态缓冲所有权。 */
Px4Lite_Result_t Px4Lite_RpiH1EchoPeekResponse(Px4Lite_RpiH1EchoResponseView_t *out);

/** @brief 仅在 UART6 完整发送成功后释放响应槽。 */
void Px4Lite_RpiH1EchoResponseSent(void);

/** @brief 复制诊断计数，并尝试刷新 UART6 已应用波特率。 */
void Px4Lite_RpiH1EchoGetStats(Px4Lite_RpiH1EchoStats_t *out);

#endif
