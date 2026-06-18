#include "integrity_fifo.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

typedef struct {
  Integrity_HighRateSample_t samples[INTEGRITY_FIFO_CAPACITY];
  uint16_t head;
  uint16_t tail;
  uint16_t count;
  Integrity_BufferStats_t stats;
} Integrity_FifoContext_t;

static Integrity_FifoContext_t s_fifo;

void Integrity_FifoInit(void)
{
  taskENTER_CRITICAL();
  memset(&s_fifo, 0, sizeof(s_fifo));
  taskEXIT_CRITICAL();
}

Integrity_Result_t Integrity_FifoPush(const Integrity_HighRateSample_t *sample)
{
  Integrity_Result_t result = INTEGRITY_OK;

  if (sample == 0) { return INTEGRITY_INVALID_PARAM; }

  taskENTER_CRITICAL();

  if (s_fifo.count >= INTEGRITY_FIFO_CAPACITY) {
    s_fifo.tail = (uint16_t)((s_fifo.tail + 1U) % INTEGRITY_FIFO_CAPACITY);
    s_fifo.count--;
    s_fifo.stats.drop_count++;
    result = INTEGRITY_OVERFLOW;
  }

  s_fifo.samples[s_fifo.head] = *sample;
  s_fifo.head                 = (uint16_t)((s_fifo.head + 1U) % INTEGRITY_FIFO_CAPACITY);
  s_fifo.count++;
  s_fifo.stats.push_count++;
  s_fifo.stats.current_count = s_fifo.count;

  if (s_fifo.count > s_fifo.stats.peak_count) { s_fifo.stats.peak_count = s_fifo.count; }

  taskEXIT_CRITICAL();
  return result;
}

Integrity_Result_t Integrity_FifoPop(Integrity_HighRateSample_t *sample)
{
  if (sample == 0) { return INTEGRITY_INVALID_PARAM; }

  taskENTER_CRITICAL();
  if (s_fifo.count == 0U) {
    taskEXIT_CRITICAL();
    return INTEGRITY_NOT_READY;
  }

  *sample     = s_fifo.samples[s_fifo.tail];
  s_fifo.tail = (uint16_t)((s_fifo.tail + 1U) % INTEGRITY_FIFO_CAPACITY);
  s_fifo.count--;
  s_fifo.stats.pop_count++;
  s_fifo.stats.current_count = s_fifo.count;
  taskEXIT_CRITICAL();
  return INTEGRITY_OK;
}

void Integrity_FifoGetStats(Integrity_BufferStats_t *stats)
{
  if (stats == 0) { return; }

  taskENTER_CRITICAL();
  *stats = s_fifo.stats;
  taskEXIT_CRITICAL();
}
