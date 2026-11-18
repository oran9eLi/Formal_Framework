/**
 * @file test_display_supply_details.c
 * @brief 使用真实LVGL详情格式化函数验证有效低压隐藏、逐字段缺测及同值恢复。
 * @details 只调用生产的三行格式化helper，不创建控件或读取硬件；原始App快照不得被修改。
 */
#include <assert.h>
#include <stdio.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
#include "../../Display/Src/display_lvgl.c"

/** @brief 本夹具不绑定LVGL标签；若格式化越界访问控件即失败。 */
char *lv_label_get_text(const lv_obj_t *obj)
{
  (void)obj;
  assert(0);
  return 0;
}

/** @brief 生成所选逻辑组的环境快照，另一组填入相反状态，避免误用旧组数据。 */
static void Prepare(App_EnvironmentSnapshot_t *env, uint32_t mv, int32_t ma,
                    uint8_t voltage_valid, uint8_t current_valid)
{
  memset(env, 0, sizeof(*env));
  env->valid_mask = APP_ENV_VALID_PRESSURE_TEMPERATURE | APP_ENV_VALID_HUMIDITY;
  env->pressure_pa = 100123.0f; env->temperature_c = 23.5f; env->relative_humidity_pct = 45.0f;
#if PX4LITE_ENABLE_BATTERY2
  env->voltage2_mv = mv; env->current2_ma = ma; env->power2_measurement_ms = 1000U;
  if (voltage_valid) { env->valid_mask |= APP_ENV_VALID_VOLTAGE2; }
  if (current_valid) { env->valid_mask |= APP_ENV_VALID_CURRENT2; }
  env->voltage_mv = 4999U; env->current_ma = 9999;
  env->valid_mask |= APP_ENV_VALID_VOLTAGE1 | APP_ENV_VALID_CURRENT1;
#else
  env->voltage_mv = mv; env->current_ma = ma; env->power1_measurement_ms = 1000U;
  if (voltage_valid) { env->valid_mask |= APP_ENV_VALID_VOLTAGE1; }
  if (current_valid) { env->valid_mask |= APP_ENV_VALID_CURRENT1; }
  env->voltage2_mv = 4999U; env->current2_ma = 9999;
  env->valid_mask |= APP_ENV_VALID_VOLTAGE2 | APP_ENV_VALID_CURRENT2;
#endif
}

/** @brief 断言真实文本及三行更新掩码，确保不改变温湿压行、其它文本或源快照。 */
static void Check(App_EnvironmentSnapshot_t *env, const char *voltage, const char *current, const char *power)
{
  App_EnvironmentSnapshot_t before;
  memcpy(&before, env, sizeof(before));
  (void)strcpy(s_row_text[0], "PRESSURE/TEMPERATURE SENTINEL");
  (void)strcpy(s_row_text[1], "HUMIDITY SENTINEL");
  (void)strcpy(s_row_text[5], "INPUT/TIME SENTINEL");
  s_row_written = 0U;
  Display_LvglFormatSupplyDetails(env, 1100U);
  assert(!strcmp(s_row_text[2], voltage));
  assert(!strcmp(s_row_text[3], current));
  assert(!strcmp(s_row_text[4], power));
  assert(s_row_written == ((1U << 2) | (1U << 3) | (1U << 4)));
  assert(!strcmp(s_row_text[0], "PRESSURE/TEMPERATURE SENTINEL"));
  assert(!strcmp(s_row_text[1], "HUMIDITY SENTINEL"));
  assert(!strcmp(s_row_text[5], "INPUT/TIME SENTINEL"));
  assert(!memcmp(&before, env, sizeof(before)));
}

/** @brief 5000mV闭区间隐藏；5001mV必须立即恢复，不受主页面百分位舍入影响。 */
static void TestThresholdAndRecovery(void)
{
  static const uint32_t hidden[] = {0U, 4999U, 5000U};
  App_EnvironmentSnapshot_t env;
  unsigned i;
  assert(DISPLAY_SUPPLY_HIDE_AT_OR_BELOW_MV == 5000U);
  Prepare(&env, 12000U, 800, 1U, 1U);
  Check(&env, "Voltage 12.000V / age 100ms", "Current 0.800A", "Power 9.600W");
  for (i = 0U; i < sizeof(hidden) / sizeof(hidden[0]); ++i) {
    Prepare(&env, hidden[i], 800, 1U, 1U);
    Check(&env, "Voltage N/A", "Current N/A", "Power N/A");
  }
  Prepare(&env, 5001U, 800, 1U, 1U);
  Check(&env, "Voltage 5.001V / age 100ms", "Current 0.800A", "Power 4.001W");
  Prepare(&env, 5000U, 800, 1U, 1U);
  Check(&env, "Voltage N/A", "Current N/A", "Power N/A");
  Prepare(&env, 12000U, 800, 1U, 1U);
  Check(&env, "Voltage 12.000V / age 100ms", "Current 0.800A", "Power 9.600W");
}

/** @brief 正常电压下真实0A仍显示；缺测保持原有独立字段语义，不让残留值变有效。 */
static void TestZeroAndMissingFields(void)
{
  App_EnvironmentSnapshot_t env;
  Prepare(&env, 12000U, 0, 1U, 1U);
  Check(&env, "Voltage 12.000V / age 100ms", "Current 0.000A", "Power 0.000W");
  Prepare(&env, 12000U, 800, 1U, 0U);
  Check(&env, "Voltage 12.000V / age 100ms", "Current N/A", "Power N/A");
  /* 未授权的低电压占位不触发有效低压规则；原有current-only显示保留。 */
  Prepare(&env, 4000U, 800, 0U, 1U);
  Check(&env, "Voltage N/A", "Current 0.800A", "Power N/A");
  Prepare(&env, 12000U, 800, 0U, 0U);
  Check(&env, "Voltage N/A", "Current N/A", "Power N/A");
  Prepare(&env, 5000U, 800, 1U, 0U);
  Check(&env, "Voltage N/A", "Current N/A", "Power N/A");
}

/** @brief 主机入口直接执行生产helper，不替换门槛、格式化或供电选组逻辑。 */
int main(void)
{
  TestThresholdAndRecovery();
  TestZeroAndMissingFields();
  puts("Power Details: real LVGL formatter / 0+4999+5000 hidden / 5001+12000 restored / valid zero A / missing fields / selected source / rows and raw snapshot untouched PASS");
  return 0;
}
