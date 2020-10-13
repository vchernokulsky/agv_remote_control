//
// Created by Maxim Dobryakov on 09/10/2020.
//

#include "lvgl/lvgl.h"

const uint32_t GAUGE_ANIMATION_DURATION = 50; // ms

void gaugeAnimationHandler(lv_obj_t *gauge, lv_anim_value_t value);

void setGaugeValueWithAnimation(lv_obj_t *gauge, int32_t newValue) {
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, gauge);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)gaugeAnimationHandler);
    lv_anim_set_values(&a, lv_gauge_get_value(gauge, 0), newValue);
    lv_anim_set_time(&a, GAUGE_ANIMATION_DURATION);
    lv_anim_set_repeat_count(&a, 0);
    lv_anim_start(&a);
}

void gaugeAnimationHandler(lv_obj_t *gauge, lv_anim_value_t value) {
    lv_gauge_set_value(gauge, 0, value);
}
