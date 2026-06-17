#ifndef INTEGRITY_SNAPSHOT_H
#define INTEGRITY_SNAPSHOT_H

#include "integrity_types.h"

/*
 * Example payload. Replace fields, but keep the measurement header.
 * This pattern is intended for small, low-rate snapshots.
 */
typedef struct
{
    Integrity_MeasurementHeader_t header;
    int32_t value_a;
    int32_t value_b;
    uint16_t status_bits;
    uint16_t reserved;
} Integrity_ExampleSnapshot_t;

void Integrity_SnapshotInit(void);
Integrity_Result_t Integrity_SnapshotPublish(
    const Integrity_ExampleSnapshot_t *source);
Integrity_Result_t Integrity_SnapshotCopy(
    Integrity_ExampleSnapshot_t *destination);

#endif
