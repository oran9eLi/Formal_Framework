/** @file test_display_lvgl_invalidation.c @brief 实际 LVGL 缓存失效逻辑，标签原语采用计数桩。 */
#include <assert.h>
#include <stdio.h>
#include "../../Display/Src/display_lvgl.c"
Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void) { return PX4LITE_REMOTE_MODE_LOCAL; }
static unsigned label_updates, hidden_calls, restored_calls, restoring;
void lv_label_set_text_static(lv_obj_t *obj,const char *text) { assert(obj!=0);if (!restoring) assert(strcmp(text,"--")==0); else assert(strcmp(text,"--")!=0);label_updates++; }
void lv_obj_add_flag(lv_obj_t *obj,lv_obj_flag_t flag) { assert(obj!=0 && flag==LV_OBJ_FLAG_HIDDEN);hidden_calls++; }
void lv_obj_clear_flag(lv_obj_t *obj,lv_obj_flag_t flag) { assert(obj!=0 && flag==LV_OBJ_FLAG_HIDDEN);restored_calls++; }
void lv_obj_invalidate(const lv_obj_t *obj) { assert(obj!=0); }
void lv_obj_set_style_bg_color(lv_obj_t *obj,lv_color_t value,lv_style_selector_t sel) { (void)obj;(void)value;(void)sel; }
int main(void) {
  unsigned i;
  Display_HmiVariableId_t id=DISPLAY_HMI_VAR_ROLL;
  s_value_valid[id]=1;s_values[id]=123;
  s_value_slots[id].label=(lv_obj_t*)1;
  for(i=0;i<DISPLAY_LVGL_VALUE_LABEL_EXTRA_COUNT;i++) s_value_extra_labels[id][i]=(lv_obj_t*)(uintptr_t)(i+2);
  s_attitude_obj=(lv_obj_t*)100;
  assert(Display_LvglInvalidateValue(id)==DISPLAY_OK);
  assert(!s_value_valid[id] && s_values[id]==123);
  assert(label_updates==1+DISPLAY_LVGL_VALUE_LABEL_EXTRA_COUNT && hidden_calls==1);
  assert(Display_LvglInvalidateValue(DISPLAY_HMI_VAR_COUNT)==DISPLAY_ERROR);
  s_value_valid[DISPLAY_HMI_VAR_PITCH]=1;s_value_valid[DISPLAY_HMI_VAR_YAW]=1;
  restoring=1;assert(Display_LvglSetValue(id,123)==DISPLAY_OK);
  assert(s_value_valid[id] && restored_calls==1);
  assert(label_updates==2*(1+DISPLAY_LVGL_VALUE_LABEL_EXTRA_COUNT));
  puts("display LVGL invalidation passed");return 0;
}
