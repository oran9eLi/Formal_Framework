/**
 * @file display_gt911.c
 * @brief Implement the GT911 capacitive touch driver.
 */

#include "display_gt911.h"

#include "bsp_config.h"
#include "bsp_touch_port.h"
#include "bsp_time.h"

#include <string.h>

#define DISPLAY_GT911_ADDR_14     0x14U
#define DISPLAY_GT911_ADDR_5D     0x5DU
#define DISPLAY_GT911_REG_PID     0x8140U
#define DISPLAY_GT911_REG_CTRL    0x8040U
#define DISPLAY_GT911_REG_CFG     0x8047U
#define DISPLAY_GT911_REG_TP_INFO 0x814EU
#define DISPLAY_GT911_REG_TP1     0x8150U
#define DISPLAY_GT911_RECOVER_RETRY_MS 1000U

static uint8_t s_gt911_addr;
static uint8_t s_gt911_last_info;
static Display_Gt911Result_t s_gt911_last_result = DISPLAY_GT911_NOT_READY;
static uint32_t s_gt911_next_recover_ms;
static uint8_t s_gt911_cfg_version;
static uint8_t s_gt911_cfg_verified;
static uint8_t s_gt911_init_error;
static uint8_t s_gt911_orig_version;
static uint8_t s_gt911_cfg_bytes[4];

/**
 * @brief Read a GT911 16-bit register with short bus recovery retries.
 */
static uint8_t Display_Gt911_ReadRegRetry(uint8_t addr, uint16_t reg, uint8_t *buf, uint8_t len)
{
  uint8_t retry;

  for (retry = 0U; retry < 3U; retry++) {
    if (BSP_TouchPort_ReadReg16(addr, reg, buf, len) == BSP_TOUCH_PORT_OK) { return 0U; }
    BSP_TouchPort_Recover();
    BSP_Time_DelayMs(2U);
  }

  return 1U;
}

/**
 * @brief Check whether a 4-byte product ID belongs to a supported GT chip.
 */
static uint8_t Display_Gt911_PidIsSupported(const uint8_t *pid)
{
  return (uint8_t)((memcmp(pid, "911", 4U) == 0) || (memcmp(pid, "9147", 4U) == 0) || (memcmp(pid, "9271", 4U) == 0) || (memcmp(pid, "1158", 4U) == 0));
}

/**
 * @brief Run the GT911 reset and INT address selection sequence.
 */
static void Display_Gt911_ResetForAddress(uint8_t addr)
{
  BSP_TouchPort_WriteReset(0U);

  if (addr == DISPLAY_GT911_ADDR_14) {
    BSP_TouchPort_SetIntOutput(0U);
    BSP_TouchPort_SetIntOutput(1U);
    BSP_Time_DelayMs(1U);
    BSP_TouchPort_WriteReset(1U);
    BSP_Time_DelayMs(10U);
  } else {
    BSP_TouchPort_SetIntOutput(0U);
    BSP_Time_DelayMs(1U);
    BSP_TouchPort_WriteReset(1U);
    BSP_Time_DelayMs(10U);
  }

  BSP_TouchPort_SetIntInput();
  BSP_Time_DelayMs(100U);
}

/**
 * @brief Read GT911 configuration diagnostics after product detection.
 */
static uint8_t Display_Gt911_ReadConfigDebug(uint8_t addr)
{
  s_gt911_cfg_verified = 0U;
  s_gt911_cfg_version  = 0U;
  s_gt911_orig_version = 0U;
  s_gt911_init_error   = 0U;

  if (BSP_TouchPort_ReadReg16(addr, DISPLAY_GT911_REG_CTRL, &s_gt911_orig_version, 1U) != BSP_TOUCH_PORT_OK) {
    s_gt911_init_error = 0xA1U;
    return 1U;
  }

  if (BSP_TouchPort_ReadReg16(addr, DISPLAY_GT911_REG_CFG, s_gt911_cfg_bytes, 4U) == BSP_TOUCH_PORT_OK) { s_gt911_cfg_verified = 1U; }

  if (BSP_TouchPort_ReadReg16(addr, DISPLAY_GT911_REG_CTRL, &s_gt911_cfg_version, 1U) != BSP_TOUCH_PORT_OK) { s_gt911_cfg_version = 0xFFU; }

  return 0U;
}

/**
 * @brief Probe one GT911 I2C address.
 */
static uint8_t Display_Gt911_TryAddress(uint8_t addr)
{
  uint8_t pid[4];

  Display_Gt911_ResetForAddress(addr);
  BSP_TouchPort_Recover();
  if (BSP_TouchPort_ReadReg16(addr, DISPLAY_GT911_REG_PID, pid, sizeof(pid)) != BSP_TOUCH_PORT_OK) { return 1U; }

  if (Display_Gt911_PidIsSupported(pid) != 0U) {
    s_gt911_addr = addr;
    (void)Display_Gt911_ReadConfigDebug(addr);
    return 0U;
  }

  return 1U;
}

/**
 * @brief 在 LVGL 输入扫描路径中限频重新探测 GT911。
 *
 * @param[in] now_ms 当前时间，单位：ms。
 *
 * @note 本函数只在 DisplayTask 上下文执行，避免在 ISR 或其他层做 I2C 恢复。
 */
static void Display_Gt911_RecoverIfDue(uint32_t now_ms)
{
  if ((s_gt911_addr != 0U) || ((int32_t)(now_ms - s_gt911_next_recover_ms) < 0)) {
    return;
  }

  (void)Display_Gt911_Init();
}

/**
 * @brief 标记 GT911 不可用并安排下一次限频重探测。
 *
 * @param[in] now_ms 当前时间，单位：ms。
 */
static void Display_Gt911_MarkNotReady(uint32_t now_ms)
{
  s_gt911_addr            = 0U;
  s_gt911_last_result     = DISPLAY_GT911_NOT_READY;
  s_gt911_next_recover_ms = now_ms + DISPLAY_GT911_RECOVER_RETRY_MS;
}

/**
 * @brief Clamp mapped coordinates to the 800x480 logical display area.
 */
static void Display_Gt911_Clamp(uint16_t *x, uint16_t *y)
{
  if (*x >= BSP_DISPLAY_WIDTH) { *x = (uint16_t)(BSP_DISPLAY_WIDTH - 1U); }
  if (*y >= BSP_DISPLAY_HEIGHT) { *y = (uint16_t)(BSP_DISPLAY_HEIGHT - 1U); }
}

/**
 * @brief Map GT911 coordinates to the 800x480 display space.
 */
static void Display_Gt911_MapToDisplay(uint16_t raw_x, uint16_t raw_y, uint16_t *x, uint16_t *y)
{
  *x = raw_x;
  *y = raw_y;

  Display_Gt911_Clamp(x, y);
}

/**
 * @brief Initialize the GT911 touch controller.
 */
Display_Gt911Result_t Display_Gt911_Init(void)
{
  (void)BSP_TouchPort_Init();
  s_gt911_addr        = 0U;
  s_gt911_last_info   = 0U;
  s_gt911_last_result = DISPLAY_GT911_NOT_READY;

  if (Display_Gt911_TryAddress(DISPLAY_GT911_ADDR_14) == 0U) {
    s_gt911_last_result = DISPLAY_GT911_OK;
    s_gt911_next_recover_ms = 0U;
    return DISPLAY_GT911_OK;
  }

  if (Display_Gt911_TryAddress(DISPLAY_GT911_ADDR_5D) == 0U) {
    s_gt911_last_result = DISPLAY_GT911_OK;
    s_gt911_next_recover_ms = 0U;
    return DISPLAY_GT911_OK;
  }

  s_gt911_last_result = DISPLAY_GT911_NOT_READY;
  s_gt911_next_recover_ms = BSP_Time_GetTickMs() + DISPLAY_GT911_RECOVER_RETRY_MS;
  return DISPLAY_GT911_NOT_READY;
}

/**
 * @brief Scan one touch point and return landscape display coordinates.
 */
Display_Gt911Result_t Display_Gt911_Scan(uint16_t *x, uint16_t *y)
{
  uint8_t info;
  uint8_t data[8];
  uint16_t raw_x;
  uint16_t raw_y;
  uint8_t clear = 0U;
  uint32_t now_ms = BSP_Time_GetTickMs();

  if ((x == 0) || (y == 0)) {
    s_gt911_last_result = DISPLAY_GT911_NOT_READY;
    return DISPLAY_GT911_NOT_READY;
  }

  if (s_gt911_addr == 0U) {
    Display_Gt911_RecoverIfDue(now_ms);
    if (s_gt911_addr == 0U) {
      s_gt911_last_result = DISPLAY_GT911_NOT_READY;
      return DISPLAY_GT911_NOT_READY;
    }
  }

  if (Display_Gt911_ReadRegRetry(s_gt911_addr, DISPLAY_GT911_REG_TP_INFO, &info, 1U) != 0U) {
    Display_Gt911_MarkNotReady(now_ms);
    return DISPLAY_GT911_NOT_READY;
  }

  s_gt911_last_info = info;
  if ((info & 0x80U) == 0U) {
    s_gt911_last_result = DISPLAY_GT911_NO_DATA;
    return DISPLAY_GT911_NO_DATA;
  }
  if ((info & 0x0FU) == 0U) {
    (void)BSP_TouchPort_WriteReg16(s_gt911_addr, DISPLAY_GT911_REG_TP_INFO, &clear, 1U);
    s_gt911_last_result = DISPLAY_GT911_NO_POINT;
    return DISPLAY_GT911_NO_POINT;
  }
  if (Display_Gt911_ReadRegRetry(s_gt911_addr, DISPLAY_GT911_REG_TP1, data, 6U) != 0U) {
    Display_Gt911_MarkNotReady(now_ms);
    return DISPLAY_GT911_NOT_READY;
  }

  raw_x = (uint16_t)(((uint16_t)data[1] << 8) | data[0]);
  raw_y = (uint16_t)(((uint16_t)data[3] << 8) | data[2]);
  Display_Gt911_MapToDisplay(raw_x, raw_y, x, y);
  (void)BSP_TouchPort_WriteReg16(s_gt911_addr, DISPLAY_GT911_REG_TP_INFO, &clear, 1U);

  s_gt911_last_result = DISPLAY_GT911_OK;
  return DISPLAY_GT911_OK;
}

/**
 * @brief Copy touch diagnostic state.
 */
void Display_Gt911_GetDebug(Display_Gt911Debug_t *debug)
{
  if (debug == 0) { return; }

  debug->addr         = s_gt911_addr;
  debug->last_info    = s_gt911_last_info;
  debug->int_level    = BSP_TouchPort_ReadInt();
  debug->scl_level    = BSP_TouchPort_ReadScl();
  debug->sda_level    = BSP_TouchPort_ReadSda();
  debug->last_result  = (uint8_t)s_gt911_last_result;
  debug->last_error   = BSP_TouchPort_GetLastError();
  debug->cfg_version  = s_gt911_cfg_version;
  debug->cfg_verified = s_gt911_cfg_verified;
  debug->init_error   = s_gt911_init_error;
  debug->touch_status = s_gt911_last_info;
  debug->orig_ver     = s_gt911_orig_version;
  (void)memcpy(debug->cfg_bytes, s_gt911_cfg_bytes, 4U);
}
