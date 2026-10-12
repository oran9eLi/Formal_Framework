/**
 * @file display_gt911.c
 * @brief Display 层 GT911 驱动；显示任务独占 BSP 触摸端口，阶段式恢复不阻塞等待。
 */

#include "display_gt911.h"

#include "bsp_config.h"
#include "bsp_touch_port.h"


#include <string.h>

#define DISPLAY_GT911_ADDR_14     0x14U
#define DISPLAY_GT911_ADDR_5D     0x5DU
#define DISPLAY_GT911_REG_PID     0x8140U
#define DISPLAY_GT911_REG_CTRL    0x8040U
#define DISPLAY_GT911_REG_CFG     0x8047U
#define DISPLAY_GT911_REG_TP_INFO 0x814EU
#define DISPLAY_GT911_REG_TP1     0x8150U

static uint8_t s_gt911_addr;
static uint8_t s_gt911_last_info;
static Display_Gt911Result_t s_gt911_last_result = DISPLAY_GT911_NOT_READY;
static uint8_t s_gt911_cfg_version;
static uint8_t s_gt911_cfg_verified;
static uint8_t s_gt911_init_error;
static uint8_t s_gt911_orig_version;
static uint8_t s_gt911_cfg_bytes[4];

/** @brief 显示任务内推进的初始化阶段；各延迟由毫秒截止时间表示。 */
typedef enum {
  GT911_STOPPED, GT911_START, GT911_RESET_WAIT, GT911_BOOT_WAIT,
  GT911_SETTLE_WAIT, GT911_PROBE, GT911_CFG_ORIG, GT911_CFG_BYTES,
  GT911_CFG_VERSION, GT911_BACKOFF, GT911_READY
} Display_Gt911Phase_t;
static Display_Gt911Phase_t s_gt911_phase;
static uint32_t s_gt911_deadline_ms, s_gt911_now_ms, s_gt911_recovery_count;
static uint8_t s_gt911_probe_addr, s_gt911_error;

/** @brief 保存故障并释放按键状态，至少退避 2 秒后重试。 */
static void Display_Gt911_Fail(void)
{
  s_gt911_error = BSP_TouchPort_GetLastError();
  if (s_gt911_error == 0U) { s_gt911_error = 1U; }
  s_gt911_addr = 0U;
  s_gt911_last_result = DISPLAY_GT911_NOT_READY;
  s_gt911_phase = GT911_BACKOFF;
  s_gt911_deadline_ms = s_gt911_now_ms + 2000U;
}

/**
 * @brief Check whether a 4-byte product ID belongs to a supported GT chip.
 */
static uint8_t Display_Gt911_PidIsSupported(const uint8_t *pid)
{
  return (uint8_t)((memcmp(pid, "911", 4U) == 0) || (memcmp(pid, "9147", 4U) == 0) || (memcmp(pid, "9271", 4U) == 0) || (memcmp(pid, "1158", 4U) == 0));
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

/** @brief 启动触摸阶段机，不阻塞等待硬件；重复调用不重置运行状态。 */
Display_Gt911Result_t Display_Gt911_Init(void)
{
  if (s_gt911_phase == GT911_STOPPED) { s_gt911_phase = GT911_START; }
  return (s_gt911_phase == GT911_READY) ? DISPLAY_GT911_OK : DISPLAY_GT911_NOT_READY;
}

/** @brief 每显示周期推进最多一个阶段；不调用 HAL_Delay，不分配内存。 */
void Display_Gt911_Service(uint32_t now_ms)
{
  uint8_t pid[4];
  s_gt911_now_ms = now_ms;
  switch (s_gt911_phase) {
    case GT911_BACKOFF:
      if ((int32_t)(now_ms - s_gt911_deadline_ms) < 0) { return; }
      s_gt911_phase = GT911_START;
      break;
    case GT911_START:
      s_gt911_recovery_count++;
      if (BSP_TouchPort_Init() != BSP_TOUCH_PORT_OK) { Display_Gt911_Fail(); break; }
      s_gt911_cfg_verified = 0U;
      s_gt911_probe_addr = DISPLAY_GT911_ADDR_14;
      BSP_TouchPort_WriteReset(0U);
      BSP_TouchPort_SetIntOutput(1U);
      s_gt911_deadline_ms = now_ms + 1U;
      s_gt911_phase = GT911_RESET_WAIT;
      break;
    case GT911_RESET_WAIT:
      if ((int32_t)(now_ms - s_gt911_deadline_ms) < 0) { return; }
      BSP_TouchPort_WriteReset(1U);
      s_gt911_deadline_ms = now_ms + 10U;
      s_gt911_phase = GT911_BOOT_WAIT;
      break;
    case GT911_BOOT_WAIT:
      if ((int32_t)(now_ms - s_gt911_deadline_ms) < 0) { return; }
      BSP_TouchPort_SetIntInput();
      s_gt911_deadline_ms = now_ms + 100U;
      s_gt911_phase = GT911_SETTLE_WAIT;
      break;
    case GT911_SETTLE_WAIT:
      if ((int32_t)(now_ms - s_gt911_deadline_ms) < 0) { return; }
      BSP_TouchPort_Recover();
      s_gt911_phase = GT911_PROBE;
      break;
    case GT911_PROBE:
      if ((BSP_TouchPort_ReadReg16(s_gt911_probe_addr, DISPLAY_GT911_REG_PID, pid, 4U) == BSP_TOUCH_PORT_OK) &&
          (Display_Gt911_PidIsSupported(pid) != 0U)) {
        s_gt911_phase = GT911_CFG_ORIG;
      } else if (s_gt911_probe_addr == DISPLAY_GT911_ADDR_14) {
        s_gt911_probe_addr = DISPLAY_GT911_ADDR_5D;
        BSP_TouchPort_WriteReset(0U);
        BSP_TouchPort_SetIntOutput(0U);
        s_gt911_deadline_ms = now_ms + 1U;
        s_gt911_phase = GT911_RESET_WAIT;
      } else { Display_Gt911_Fail(); }
      break;
    case GT911_CFG_ORIG:
      if (BSP_TouchPort_ReadReg16(s_gt911_probe_addr, DISPLAY_GT911_REG_CTRL, &s_gt911_orig_version, 1U) != BSP_TOUCH_PORT_OK) {
        s_gt911_init_error = 0xA1U; Display_Gt911_Fail(); break;
      }
      s_gt911_phase = GT911_CFG_BYTES;
      break;
    case GT911_CFG_BYTES:
      if (BSP_TouchPort_ReadReg16(s_gt911_probe_addr, DISPLAY_GT911_REG_CFG, s_gt911_cfg_bytes, 4U) != BSP_TOUCH_PORT_OK) {
        s_gt911_init_error = 0xA2U; Display_Gt911_Fail(); break;
      }
      s_gt911_cfg_verified = 1U;
      s_gt911_phase = GT911_CFG_VERSION;
      break;
    case GT911_CFG_VERSION:
      if (BSP_TouchPort_ReadReg16(s_gt911_probe_addr, DISPLAY_GT911_REG_CTRL, &s_gt911_cfg_version, 1U) != BSP_TOUCH_PORT_OK) {
        s_gt911_init_error = 0xA3U; Display_Gt911_Fail(); break;
      }
      s_gt911_addr = s_gt911_probe_addr;
      s_gt911_error = 0U; s_gt911_init_error = 0U;
      s_gt911_last_result = DISPLAY_GT911_OK;
      s_gt911_phase = GT911_READY;
      break;
    default: break;
  }
}

/** @brief 复制纯软件触摸状态，供显示门面读取，不执行 GPIO/I2C。 */
void Display_Gt911_GetStatus(Display_Gt911Status_t *out)
{
  if (out == 0) { return; }
  out->ready = (uint8_t)(s_gt911_phase == GT911_READY);
  out->recovering = (uint8_t)((s_gt911_phase != GT911_READY) && (s_gt911_phase != GT911_STOPPED));
  out->recovery_count = s_gt911_recovery_count;
  out->last_error = s_gt911_error;
}

/**
 * @brief 单次读取触摸点；任一 I/O 失败进入恢复退避，调用者须报告按键释放。
 */
Display_Gt911Result_t Display_Gt911_Scan(uint16_t *x, uint16_t *y)
{
  uint8_t info;
  uint8_t data[8];
  uint16_t raw_x;
  uint16_t raw_y;
  uint8_t clear = 0U;

  if ((x == 0) || (y == 0) || (s_gt911_addr == 0U)) {
    s_gt911_last_result = DISPLAY_GT911_NOT_READY;
    return DISPLAY_GT911_NOT_READY;
  }

  if (BSP_TouchPort_ReadReg16(s_gt911_addr, DISPLAY_GT911_REG_TP_INFO, &info, 1U) != BSP_TOUCH_PORT_OK) {
    Display_Gt911_Fail();
    return DISPLAY_GT911_NOT_READY;
  }

  s_gt911_last_info = info;
  if ((info & 0x80U) == 0U) {
    s_gt911_last_result = DISPLAY_GT911_NO_DATA;
    return DISPLAY_GT911_NO_DATA;
  }
  if ((info & 0x0FU) == 0U) {
    if (BSP_TouchPort_WriteReg16(s_gt911_addr, DISPLAY_GT911_REG_TP_INFO, &clear, 1U) != BSP_TOUCH_PORT_OK) {
      Display_Gt911_Fail(); return DISPLAY_GT911_NOT_READY;
    }
    s_gt911_last_result = DISPLAY_GT911_NO_POINT;
    return DISPLAY_GT911_NO_POINT;
  }
  if (BSP_TouchPort_ReadReg16(s_gt911_addr, DISPLAY_GT911_REG_TP1, data, 6U) != BSP_TOUCH_PORT_OK) {
    Display_Gt911_Fail();
    return DISPLAY_GT911_NOT_READY;
  }

  raw_x = (uint16_t)(((uint16_t)data[1] << 8) | data[0]);
  raw_y = (uint16_t)(((uint16_t)data[3] << 8) | data[2]);
  Display_Gt911_MapToDisplay(raw_x, raw_y, x, y);
  if (BSP_TouchPort_WriteReg16(s_gt911_addr, DISPLAY_GT911_REG_TP_INFO, &clear, 1U) != BSP_TOUCH_PORT_OK) {
      Display_Gt911_Fail(); return DISPLAY_GT911_NOT_READY;
    }

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
