/**
 * @file test_display_motor_lock.c
 * @brief 使用真实Display门面与LVGL锁投影验证人工锁动作、实际状态和禁用权限。
 * @details App替身只模拟已确认状态及发布结果；LVGL替身记录现有对象状态，不创建窗口或访问BSP。
 */
#include <assert.h>
#include <stdio.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
#include "../../Display/Src/display.c"
#include "../../Display/Src/display_lvgl.c"

static uint8_t actual_locked = 1U, last_request = 255U;
static Px4Lite_Result_t setter_result = PX4LITE_OK, getter_result = PX4LITE_OK;
static Px4Lite_Result_t motor_result = PX4LITE_OK;
static App_MotorSnapshot_t published_motor;
static unsigned lock_requests, throttle_requests;
static uint8_t last_motor, last_percent;
static lv_obj_t fake_objects[8], fake_lock_label;
static uint8_t disabled[8];
static const char *button_text = "";

/** @brief 模拟Control确认的最小输出，不在显示层依据请求自行捏造PWM。 */
static void PublishMinimum(uint8_t locked)
{
  unsigned i;
  memset(&published_motor, 0, sizeof(published_motor));
  published_motor.control_mode_valid = published_motor.operation_state_valid = 1U;
  published_motor.operation_state = locked ? PX4LITE_MOTOR_OP_LOCKED : PX4LITE_MOTOR_OP_ESC_PREPARING;
  for (i = 0U; i < 4U; i++) { published_motor.pulse_us[i] = 1000U; }
}

Px4Lite_Result_t App_SetMotorLocked(uint8_t locked)
{
  lock_requests++; last_request = locked;
  if (locked > 1U) { return PX4LITE_INVALID_PARAM; }
  if (setter_result == PX4LITE_OK) { actual_locked = locked; PublishMinimum(locked); }
  else if (setter_result == PX4LITE_IO_ERROR && locked) { actual_locked = 1U; }
  return setter_result;
}
Px4Lite_Result_t App_GetMotorLocked(uint8_t *locked)
{ if (getter_result != PX4LITE_OK) { return getter_result; } *locked = actual_locked; return PX4LITE_OK; }
Px4Lite_Result_t App_CopyMotor(App_MotorSnapshot_t *out, uint32_t now_ms)
{ (void)now_ms; *out = published_motor; return motor_result; }
Px4Lite_Result_t App_SetMotorThrottlePercent(uint8_t motor, uint8_t percent)
{
  throttle_requests++; last_motor = motor; last_percent = percent;
  if (actual_locked && percent) { return PX4LITE_NOT_READY; }
  published_motor.base_percent[motor] = percent;
  return PX4LITE_OK;
}
uint32_t Px4Lite_PlatformGetMs(void) { return 1000U; }

/** @brief 映射固定对象替身，任何越界访问都立即失败。 */
static unsigned ObjectIndex(lv_obj_t *obj)
{
  unsigned i;
  for (i = 0U; i < 8U; i++) { if (obj == &fake_objects[i]) { return i; } }
  assert(0); return 0U;
}
void lv_obj_add_state(lv_obj_t *obj, lv_state_t state)
{ assert(state == LV_STATE_DISABLED); disabled[ObjectIndex(obj)] = 1U; }
void lv_obj_clear_state(lv_obj_t *obj, lv_state_t state)
{ assert(state == LV_STATE_DISABLED); disabled[ObjectIndex(obj)] = 0U; }
char *lv_label_get_text(const lv_obj_t *obj)
{ assert(obj == &fake_lock_label); return (char *)button_text; }
void lv_label_set_text_static(lv_obj_t *obj, const char *text)
{ assert(obj == &fake_lock_label); button_text = text; }

/** @brief 仅配置已存在对象池的绑定，保持急停和缓降始终可操作。 */
static void BindControls(void)
{
  unsigned i;
  memset(s_buttons, 0, sizeof(s_buttons));
  s_button_used = 4U;
  s_buttons[0].obj = &fake_objects[0]; s_buttons[0].label = &fake_lock_label; s_buttons[0].action = UI_ACTION_MOTOR_LOCK;
  s_buttons[1].obj = &fake_objects[1]; s_buttons[1].action = UI_ACTION_RAMP;
  s_buttons[2].obj = &fake_objects[2]; s_buttons[2].action = UI_ACTION_ESTOP;
  s_buttons[3].obj = &fake_objects[3]; s_buttons[3].action = UI_ACTION_DOWN;
  for (i = 0U; i < 4U; i++) { s_motor_pwm_bars[i] = &fake_objects[i + 4U]; }
  s_current_lvgl_page = DISPLAY_HMI_PAGE_MOTOR;
}

/** @brief 状态标题、按钮动作和输入权限必须来自同一份实际锁状态。 */
static void AssertProjection(const char *state, const char *action, uint8_t blocked)
{
  unsigned i;
  unsigned before = lock_requests;
  unsigned before_throttle = throttle_requests;
  Display_LvglFormatValue(DISPLAY_HMI_VAR_MOTOR_LOCKED, s_values[DISPLAY_HMI_VAR_MOTOR_LOCKED]);
  assert(strcmp(s_value_slots[DISPLAY_HMI_VAR_MOTOR_LOCKED].text, state) == 0);
  UiMotorLockControls();
  assert(strcmp(button_text, action) == 0);
  assert(disabled[1] == blocked);
  assert(disabled[2] == 0U && disabled[3] == 0U);
  for (i = 4U; i < 8U; i++) { assert(disabled[i] == blocked); }
  assert(lock_requests == before && throttle_requests == before_throttle); /* 刷新不能产生控制。 */
}

/** @brief 初始上锁、显式解锁/上锁、失败真实状态及未知状态回归。 */
int main(void)
{
  uint8_t target;
  BindControls(); PublishMinimum(1U);
  Display_LoadMotorLockState();
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_LOCKED].valid);
  AssertProjection("LOCKED", "Unlock", 1U);
  assert(!disabled[0] && UiMotorLockTarget(&target) && target == 0U);

  assert(Display_RequestMotorLocked(target) == DISPLAY_OK);
  assert(last_request == 0U && lock_requests == 1U && throttle_requests == 0U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_TARGET_1].value == 0U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_PWM_1].value == 0U && s_motor_pulse_us[0] == 1000U);
  AssertProjection("UNLOCKED", "Lock", 0U);
  assert(UiMotorLockTarget(&target) && target == 1U);

  published_motor.base_percent[0] = published_motor.duty_percent[0] = 35U; published_motor.pulse_us[0] = 1350U;
  Display_LoadMotorSnapshot(&published_motor); Display_LoadMotorPulseSnapshot(&published_motor);
  assert(Display_RequestMotorLocked(target) == DISPLAY_OK);
  assert(last_request == 1U && actual_locked == 1U && lock_requests == 2U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_TARGET_1].value == 0U && s_motor_pulse_us[0] == 1000U);
  AssertProjection("LOCKED", "Unlock", 1U);

  setter_result = PX4LITE_NOT_READY;
  assert(Display_RequestMotorLocked(0U) == DISPLAY_NOT_READY);
  AssertProjection("LOCKED", "Unlock", 1U);
  setter_result = PX4LITE_OK; assert(Display_RequestMotorLocked(0U) == DISPLAY_OK);
  published_motor.duty_percent[0] = 35U; Display_LoadMotorSnapshot(&published_motor);
  setter_result = PX4LITE_IO_ERROR;
  assert(Display_RequestMotorLocked(1U) == DISPLAY_ERROR);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_PWM_1].value == 35U); /* 失败不能伪造最小输出成功。 */
  AssertProjection("LOCKED", "Unlock", 1U);

  getter_result = PX4LITE_NOT_READY; Display_LoadMotorLockState(); target = 255U;
  assert(!s_hmi_values[DISPLAY_HMI_VAR_MOTOR_LOCKED].valid && !UiMotorLockTarget(&target) && target == 255U);
  AssertProjection("Lock N/A", "Lock N/A", 1U); assert(disabled[0]);
  getter_result = PX4LITE_OK; actual_locked = 2U; Display_LoadMotorLockState();
  assert(!s_hmi_values[DISPLAY_HMI_VAR_MOTOR_LOCKED].valid && !UiMotorLockTarget(&target));
  assert(throttle_requests == 0U);

  /* Details页沿真实入口增减目标，命令编号必须映射到选中的电机。 */
  actual_locked = 0U; setter_result = PX4LITE_OK;
  published_motor.base_percent[2] = 35U;
  assert(UiMotorStepRequest(2U, 1U) == DISPLAY_OK && last_motor == 2U && last_percent == 36U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_TARGET_3].value == 36U);
  assert(UiMotorStepRequest(2U, 0U) == DISPLAY_OK && last_motor == 2U && last_percent == 35U);
  published_motor.base_percent[2] = 100U;
  assert(UiMotorStepRequest(2U, 1U) == DISPLAY_OK && last_percent == 100U);
  published_motor.base_percent[2] = 0U;
  assert(UiMotorStepRequest(2U, 0U) == DISPLAY_OK && last_percent == 0U);
  motor_result = PX4LITE_NOT_READY;
  assert(UiMotorStepRequest(2U, 1U) == DISPLAY_NOT_READY && throttle_requests == 4U);
  motor_result = PX4LITE_OK; actual_locked = 1U;
  assert(UiMotorStepRequest(2U, 1U) == DISPLAY_NOT_READY && throttle_requests == 5U);
  assert(published_motor.base_percent[2] == 0U);
  assert(UiMotorStepRequest(4U, 1U) == DISPLAY_ERROR && throttle_requests == 5U);
  puts("display motor lock: PASS (actual toggle, no implicit throttle, failure/N-A, fixed controls, real +/- motor route)");
  return 0;
}
