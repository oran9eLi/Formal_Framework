#include "integrity_tx.h"
#include <string.h>

void Integrity_TxInit(Integrity_TxContext_t *context)
{
  if (context != 0) { memset(context, 0, sizeof(*context)); }
}

Integrity_Result_t Integrity_TxStart(Integrity_TxContext_t *context, const uint8_t *frame, uint16_t frame_length, uint32_t now_ms, uint32_t timeout_ms)
{
  if ((context == 0) || (frame == 0) || (frame_length == 0U)) { return INTEGRITY_INVALID_PARAM; }

  if ((context->state == INTEGRITY_TX_SENDING) || (context->state == INTEGRITY_TX_WAIT_COMPLETE)) { return INTEGRITY_BUSY; }

  context->frame        = frame;
  context->frame_length = frame_length;
  context->offset       = 0U;
  context->deadline_ms  = now_ms + timeout_ms;
  context->state        = INTEGRITY_TX_SENDING;
  return INTEGRITY_OK;
}

Integrity_Result_t Integrity_TxStep(Integrity_TxContext_t *context, Integrity_TransportWriteFn write_fn, uint32_t now_ms)
{
  int32_t written;
  uint16_t remaining;

  if ((context == 0) || (write_fn == 0)) { return INTEGRITY_INVALID_PARAM; }

  if ((context->state != INTEGRITY_TX_SENDING) && (context->state != INTEGRITY_TX_WAIT_COMPLETE)) { return INTEGRITY_IDLE; }

  if ((int32_t)(now_ms - context->deadline_ms) >= 0) {
    context->state = INTEGRITY_TX_FAILED;
    context->timeout_count++;
    return INTEGRITY_TIMEOUT;
  }

  if (context->state == INTEGRITY_TX_WAIT_COMPLETE) { return INTEGRITY_BUSY; }

  remaining = (uint16_t)(context->frame_length - context->offset);
  written   = write_fn(&context->frame[context->offset], remaining);

  if (written < 0) {
    context->state = INTEGRITY_TX_FAILED;
    context->error_count++;
    return INTEGRITY_IO_ERROR;
  }

  if (written == 0) { return INTEGRITY_BUSY; }

  if ((uint32_t)written > remaining) {
    context->state = INTEGRITY_TX_FAILED;
    context->error_count++;
    return INTEGRITY_IO_ERROR;
  }

  context->offset = (uint16_t)(context->offset + (uint16_t)written);

  if (context->offset < context->frame_length) { return INTEGRITY_BUSY; }

  /*
   * All bytes were accepted by the driver. The buffer still belongs to
   * this context until the DMA/transport complete callback is received.
   */
  context->state = INTEGRITY_TX_WAIT_COMPLETE;
  return INTEGRITY_BUSY;
}

void Integrity_TxOnComplete(Integrity_TxContext_t *context, uint8_t success)
{
  if ((context == 0) || (context->state != INTEGRITY_TX_WAIT_COMPLETE)) { return; }

  if (success != 0U) {
    context->state = INTEGRITY_TX_COMPLETE;
    context->complete_count++;
  } else {
    context->state = INTEGRITY_TX_FAILED;
    context->error_count++;
  }
}
