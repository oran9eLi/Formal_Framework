#include "integrity_snapshot.h"
#include "stm32f4xx.h"
#include <string.h>

static Integrity_ExampleSnapshot_t s_snapshot;
static uint8_t s_ready;

static uint32_t Integrity_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Integrity_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Integrity_SnapshotInit(void)
{
    uint32_t primask = Integrity_EnterCritical();
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_ready = 0U;
    Integrity_ExitCritical(primask);
}

Integrity_Result_t Integrity_SnapshotPublish(
    const Integrity_ExampleSnapshot_t *source)
{
    uint32_t primask;

    if (source == 0)
    {
        return INTEGRITY_INVALID_PARAM;
    }

    /*
     * The producer prepares source completely before entering this function.
     * The critical section contains only a bounded structure copy.
     */
    primask = Integrity_EnterCritical();
    s_snapshot = *source;
    s_ready = 1U;
    Integrity_ExitCritical(primask);
    return INTEGRITY_OK;
}

Integrity_Result_t Integrity_SnapshotCopy(
    Integrity_ExampleSnapshot_t *destination)
{
    uint32_t primask;

    if (destination == 0)
    {
        return INTEGRITY_INVALID_PARAM;
    }

    primask = Integrity_EnterCritical();
    if (s_ready == 0U)
    {
        Integrity_ExitCritical(primask);
        return INTEGRITY_NOT_READY;
    }

    *destination = s_snapshot;
    Integrity_ExitCritical(primask);
    return INTEGRITY_OK;
}
