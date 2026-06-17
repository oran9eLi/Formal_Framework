/**
 * @file lora_e22.h
 * @brief E22-400T30D LoRa 驱动接口和接收帧类型。
 *
 * @details
 * 本驱动负责 LoRa UART 收发、MAVLink 帧边界识别、发送忙状态和通信统计。
 * 驱动不解释业务命令，不直接修改 Business 状态。发送为异步 copy 语义。
 */

#ifndef LORA_E22_H
#define LORA_E22_H

#include <stdint.h>

#define LORA_E22_RX_PAYLOAD_MAX   255U /**< MAVLink 最大 payload 长度，单位：byte。 */
#define LORA_E22_TX_BUF_SIZE      280U /**< MAVLink2 帧含签名的发送缓冲长度，单位：byte。 */

/**
 * @brief LoRa 驱动返回值。
 */
typedef enum {
    LORA_RESULT_OK = 0,       /**< 操作成功或发送已入队。 */
    LORA_RESULT_NO_DATA,      /**< 当前无可用接收帧。 */
    LORA_RESULT_BUSY,         /**< 发送通道忙，上一帧尚未完成。 */
    LORA_RESULT_IO_ERROR,     /**< UART/DMA 或 BSP 访问失败。 */
    LORA_RESULT_INVALID_PARAM /**< 参数非法。 */
} Lora_Result_t;

/**
 * @brief LoRa 驱动公开状态。
 */
typedef enum {
    LORA_STATE_NOT_READY = 0, /**< 尚未初始化或尚无有效活动。 */
    LORA_STATE_ONLINE,        /**< 在线。 */
    LORA_STATE_OFFLINE,       /**< 超时离线。 */
    LORA_STATE_FAILED         /**< 驱动失败。 */
} Lora_State_t;

/**
 * @brief LoRa 接收到的一帧 MAVLink 数据。
 */
typedef struct {
    uint16_t frame_len;                    /**< 完整 MAVLink 帧长度，单位：byte。 */
    uint8_t  system_id;                    /**< MAVLink system id。 */
    uint8_t  component_id;                 /**< MAVLink component id。 */
    uint8_t  sequence;                     /**< MAVLink packet sequence。 */
    uint8_t  payload_len;                  /**< payload 长度，单位：byte。 */
    uint32_t msg_id;                       /**< MAVLink message id。 */
    uint8_t  data[LORA_E22_RX_PAYLOAD_MAX];/**< MAVLink payload 副本。 */
} Lora_RxFrame_t;

/**
 * @brief LoRa 驱动调试统计。
 */
typedef struct {
    uint32_t rx_frame_count;    /**< 已接收完整帧数量。 */
    uint32_t tx_frame_count;    /**< UART DMA 发送完成帧数量。 */
    uint32_t tx_busy_count;     /**< 因发送忙拒绝新帧次数。 */
    uint32_t crc_error_count;   /**< MAVLink CRC 错误次数。 */
    uint32_t send_error_count;  /**< 发送提交错误次数。 */
    uint32_t parse_error_count; /**< 接收解析错误次数。 */
    uint32_t rx_byte_count;     /**< 接收字节累计数。 */
    uint32_t rx_overflow_count; /**< 接收缓冲溢出次数。 */
    uint32_t rx_drop_count;     /**< 接收丢弃次数。 */
    uint32_t last_rx_ms;        /**< 最近接收完整帧时间，单位：ms。 */
    uint32_t last_tx_ms;        /**< 最近发送完成时间，单位：ms。 */
    uint32_t last_msg_id;       /**< 最近接收的 MAVLink message id。 */
} Lora_DebugInfo_t;

/**
 * @brief 初始化 E22 LoRa 驱动和底层 UART 收发资源。
 *
 * @return 初始化结果。
 */
Lora_Result_t Lora_E22_Init(void);

/**
 * @brief 执行 LoRa 周期服务，处理 RX、TX 完成和延迟 reinit。
 *
 * @param[in] now_ms 当前 comm 周期时间，单位：ms。
 *
 * @return 服务结果。
 *
 * @note 本函数由 comm 任务调用，不在 ISR 中调用。
 */
Lora_Result_t Lora_E22_Service(uint32_t now_ms);

/**
 * @brief 复制一帧已解析的 LoRa/MAVLink 接收帧。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 * @retval LORA_RESULT_OK 复制成功。
 * @retval LORA_RESULT_NO_DATA 当前无完整接收帧。
 * @retval LORA_RESULT_INVALID_PARAM 参数为空。
 */
Lora_Result_t Lora_E22_CopyRxFrame(Lora_RxFrame_t *out);

/**
 * @brief 获取 LoRa 当前状态。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 * @param[in] offline_timeout_ms 离线判定超时时间，单位：ms。
 *
 * @return LoRa 状态。
 */
Lora_State_t  Lora_E22_GetState(uint32_t now_ms,
                                uint32_t offline_timeout_ms);

/**
 * @brief 异步发送一帧 LoRa 数据。
 *
 * @param[in] data 待发送字节缓冲区，不能为 NULL。
 * @param[in] len 待发送长度，单位：byte。
 *
 * @return 发送提交结果。
 * @retval LORA_RESULT_OK 数据已复制到驱动发送缓冲并提交发送流程。
 * @retval LORA_RESULT_BUSY 上一帧仍在发送中。
 * @retval LORA_RESULT_INVALID_PARAM 参数为空或长度非法。
 * @retval LORA_RESULT_IO_ERROR 底层发送提交失败。
 *
 * @note `LORA_RESULT_OK` 不表示空口发送完成；发送完成由 UART DMA TC 回调更新
 * `tx_frame_count` 和 `last_tx_ms`。调用方在返回 OK 后可以立即复用输入缓冲区。
 */
Lora_Result_t Lora_E22_Send(const uint8_t *data, uint16_t len);

/**
 * @brief 复制 LoRa 驱动调试统计。
 *
 * @param[out] info 输出缓冲区，不能为 NULL。
 *
 * @note 本函数只复制内存统计，不访问 UART 或 LoRa 模块。
 */
void          Lora_E22_GetDebugInfo(Lora_DebugInfo_t *info);

/**
 * @brief 请求 LoRa 在下一次 Service 中重新初始化。
 *
 * @note 本函数只置位请求标志，适合 recovery 回调调用。
 */
void          Lora_E22_RequestReinit(void);

#endif
