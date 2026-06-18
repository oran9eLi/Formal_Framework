#ifndef INTEGRITY_TYPES_H
#define INTEGRITY_TYPES_H

#include <stdint.h>

typedef enum
{
    INTEGRITY_OK = 0,
    INTEGRITY_IDLE,
    INTEGRITY_BUSY,
    INTEGRITY_INVALID_PARAM,
    INTEGRITY_NOT_READY,
    INTEGRITY_TIMEOUT,
    INTEGRITY_CRC_ERROR,
    INTEGRITY_OVERFLOW,
    INTEGRITY_IO_ERROR
} Integrity_Result_t;

typedef enum
{
    COMPONENT_UNINITIALIZED = 0,
    COMPONENT_STARTING,
    COMPONENT_ONLINE,
    COMPONENT_DEGRADED,
    COMPONENT_OFFLINE,
    COMPONENT_FAILED
} Integrity_ComponentState_t;

typedef struct
{
    uint16_t device_id;
    uint16_t flags;
    uint32_t sequence;
    uint32_t sample_time_ms;
    uint32_t publish_time_ms;
    uint8_t valid;
    uint8_t quality;
    uint16_t reserved;
} Integrity_MeasurementHeader_t;

typedef struct
{
    uint32_t push_count;
    uint32_t pop_count;
    uint32_t drop_count;
    uint32_t error_count;
    uint16_t current_count;
    uint16_t peak_count;
} Integrity_BufferStats_t;

static __inline uint8_t Integrity_IsFresh(
    const Integrity_MeasurementHeader_t *header,
    uint32_t now_ms,
    uint32_t max_age_ms)
{
    if ((header == 0) || (header->valid == 0U))
    {
        return 0U;
    }

    return ((uint32_t)(now_ms - header->publish_time_ms) <= max_age_ms)
               ? 1U
               : 0U;
}

#endif
