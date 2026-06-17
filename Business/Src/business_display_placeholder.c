/**
 * @file business_display_placeholder.c
 * @brief Adapt the real Display module to the business display task hooks.
 */

#include "business_task_template.h"

#include "display.h"

/**
 * @brief Convert a display-layer result into a business service result.
 */
static Business_ServiceResult_t Business_DisplayMapResult(
    Display_Result_t result)
{
    if (result == DISPLAY_OK)
    {
        return BUSINESS_SERVICE_OK;
    }

    if (result == DISPLAY_NOT_READY)
    {
        return BUSINESS_SERVICE_NOT_READY;
    }

    return BUSINESS_SERVICE_ERROR;
}

/**
 * @brief Poll touch input through the real Display module.
 */
Business_ServiceResult_t Business_DisplayPollTouch(uint32_t now_ms)
{
    (void)now_ms;
    return Business_DisplayMapResult(Display_PollTouch());
}

/**
 * @brief Prepare one coherent display snapshot from application read-only APIs.
 */
Business_ServiceResult_t Business_DisplayPrepareSnapshot(uint32_t now_ms)
{
    return Business_DisplayMapResult(Display_PrepareSnapshot(now_ms));
}

/**
 * @brief Advance one budget-limited display refresh step.
 */
Business_ServiceResult_t Business_DisplayRefreshStep(uint32_t now_ms,
                                                     uint32_t budget_us)
{
    Display_Result_t result;

    result = Display_RefreshStep(now_ms, budget_us);
    if (result == DISPLAY_NOT_READY)
    {
        return BUSINESS_SERVICE_BUSY;
    }

    return Business_DisplayMapResult(result);
}
