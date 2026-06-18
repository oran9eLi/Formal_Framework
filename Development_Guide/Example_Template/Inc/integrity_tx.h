#ifndef INTEGRITY_TX_H
#define INTEGRITY_TX_H

#include "integrity_types.h"

typedef enum
{
    INTEGRITY_TX_IDLE = 0,
    INTEGRITY_TX_SENDING,
    INTEGRITY_TX_WAIT_COMPLETE,
    INTEGRITY_TX_COMPLETE,
    INTEGRITY_TX_FAILED
} Integrity_TxState_t;

typedef int32_t (*Integrity_TransportWriteFn)(
    const uint8_t *data,
    uint16_t length);

typedef struct
{
    const uint8_t *frame;
    uint16_t frame_length;
    uint16_t offset;
    uint32_t deadline_ms;
    Integrity_TxState_t state;
    uint32_t complete_count;
    uint32_t timeout_count;
    uint32_t error_count;
} Integrity_TxContext_t;

void Integrity_TxInit(Integrity_TxContext_t *context);
Integrity_Result_t Integrity_TxStart(
    Integrity_TxContext_t *context,
    const uint8_t *frame,
    uint16_t frame_length,
    uint32_t now_ms,
    uint32_t timeout_ms);
Integrity_Result_t Integrity_TxStep(
    Integrity_TxContext_t *context,
    Integrity_TransportWriteFn write_fn,
    uint32_t now_ms);
void Integrity_TxOnComplete(Integrity_TxContext_t *context,
                            uint8_t success);

#endif
