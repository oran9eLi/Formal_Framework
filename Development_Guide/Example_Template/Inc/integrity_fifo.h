#ifndef INTEGRITY_FIFO_H
#define INTEGRITY_FIFO_H

#include "integrity_types.h"

#define INTEGRITY_FIFO_CAPACITY 32U

typedef struct
{
    Integrity_MeasurementHeader_t header;
    int32_t axis[3];
} Integrity_HighRateSample_t;

void Integrity_FifoInit(void);
Integrity_Result_t Integrity_FifoPush(
    const Integrity_HighRateSample_t *sample);
Integrity_Result_t Integrity_FifoPop(
    Integrity_HighRateSample_t *sample);
void Integrity_FifoGetStats(Integrity_BufferStats_t *stats);

#endif
