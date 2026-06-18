/**
 * @file lora_e22.h
 * @brief Define the E22-400T30D LoRa driver interface and typed frame.
 */

#ifndef LORA_E22_H
#define LORA_E22_H

#include <stdint.h>

#define LORA_E22_RX_PAYLOAD_MAX   255U   /* MAVLink max payload */
#define LORA_E22_TX_BUF_SIZE      280U   /* MAVLink 2 frame including signature */

typedef enum {
    LORA_RESULT_OK = 0,
    LORA_RESULT_NO_DATA,
    LORA_RESULT_BUSY,
    LORA_RESULT_IO_ERROR,
    LORA_RESULT_INVALID_PARAM
} Lora_Result_t;

typedef enum {
    LORA_STATE_NOT_READY = 0,
    LORA_STATE_ONLINE,
    LORA_STATE_OFFLINE,
    LORA_STATE_FAILED
} Lora_State_t;

typedef struct {
    uint16_t frame_len;
    uint8_t  system_id;
    uint8_t  component_id;
    uint8_t  sequence;
    uint8_t  payload_len;
    uint32_t msg_id;
    uint8_t  data[LORA_E22_RX_PAYLOAD_MAX];
} Lora_RxFrame_t;

typedef struct {
    uint32_t rx_frame_count;
    uint32_t tx_frame_count;
    uint32_t tx_busy_count;
    uint32_t crc_error_count;
    uint32_t send_error_count;
    uint32_t parse_error_count;
    uint32_t rx_byte_count;
    uint32_t rx_overflow_count;
    uint32_t rx_drop_count;
    uint32_t last_rx_ms;
    uint32_t last_tx_ms;
    uint32_t last_msg_id;
} Lora_DebugInfo_t;

Lora_Result_t Lora_E22_Init(void);
Lora_Result_t Lora_E22_Service(uint32_t now_ms);
Lora_Result_t Lora_E22_CopyRxFrame(Lora_RxFrame_t *out);
Lora_State_t  Lora_E22_GetState(uint32_t now_ms,
                                uint32_t offline_timeout_ms);
/*
 * Async-copy send. On LORA_RESULT_OK the frame has been copied into the driver
 * and staged for transmission (one frame in flight); the caller may reuse its
 * buffer immediately. OK does NOT mean the frame has left the air interface --
 * air-side completion is reflected by tx_frame_count / last_tx_ms in
 * Lora_DebugInfo_t. Returns LORA_RESULT_BUSY while a frame is still in flight.
 */
Lora_Result_t Lora_E22_Send(const uint8_t *data, uint16_t len);
void          Lora_E22_GetDebugInfo(Lora_DebugInfo_t *info);

/* Request a re-init (cheap; performed on the next Service call). */
void          Lora_E22_RequestReinit(void);

#endif
