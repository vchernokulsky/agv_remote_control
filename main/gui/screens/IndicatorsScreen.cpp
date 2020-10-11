//
// Created by Maxim Dobryakov on 06/10/2020.
//

#include "IndicatorsScreen.h"


#include <cstdlib>
#include "assets/Background.h"
#include "assets/MaterialDesignIconsFont.h"
#include "utils/MapValue.h"
#include "utils/GaugeExt.h"

using namespace std;

IndicatorsScreen::IndicatorsScreen()
    : NEEDLE_COLORS{ lv_color_make(255, 0, 0) }
{
}

void IndicatorsScreen::initializeGui() {
    screen = lv_scr_act();

    imgBackground = lv_img_create(screen, nullptr);
    lv_img_set_src(imgBackground, &Background);

    lblPlatformName = lv_label_create(screen, nullptr);
    lv_label_set_text(lblPlatformName, "Платформа №42");
    lv_label_set_long_mode(lblPlatformName, LV_LABEL_LONG_SROLL_CIRC);
    lv_obj_set_size(lblPlatformName, 180, 20);
    lv_obj_align(lblPlatformName, nullptr, LV_ALIGN_IN_TOP_LEFT, 5, 5);

    lblWiFiStatus = lv_label_create(screen, nullptr);
    lv_label_set_text(lblWiFiStatus, MDI_SYMBOL_WIFI_NOT_FOUND);
    lv_label_set_align(lblWiFiStatus, LV_LABEL_ALIGN_CENTER);
    lv_obj_set_style_local_text_font(lblWiFiStatus, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &MaterialDesignIconsFont);
    lv_obj_set_size(lblWiFiStatus, 20, 20);
    lv_obj_align(lblWiFiStatus, nullptr, LV_ALIGN_IN_TOP_RIGHT, -25, 5);

    lblBatteryStatus = lv_label_create(screen, nullptr);
    lv_label_set_text(lblBatteryStatus, MDI_SYMBOL_BATTERY_NOT_FOUND);
    lv_label_set_align(lblBatteryStatus, LV_LABEL_ALIGN_CENTER);
    lv_obj_set_style_local_text_font(lblBatteryStatus, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &MaterialDesignIconsFont);
    lv_obj_set_size(lblBatteryStatus, 20, 20);
    lv_obj_align(lblBatteryStatus, nullptr, LV_ALIGN_IN_TOP_RIGHT, -5, 5);

    ggLinearSpeed = lv_gauge_create(screen, nullptr);
    lv_obj_set_size(ggLinearSpeed, 160, 160);
    lv_gauge_set_range(ggLinearSpeed, -100, 100);
    lv_gauge_set_scale(ggLinearSpeed, 270, 9, 5);
    lv_gauge_set_needle_count(ggLinearSpeed, 1, NEEDLE_COLORS);
    lv_gauge_set_formatter_cb(ggLinearSpeed, LinearSpeedFormatterCallback);
    lv_obj_set_style_local_pad_inner(ggLinearSpeed, LV_GAUGE_PART_MAIN, LV_STATE_DEFAULT, 15);
    lv_obj_align(ggLinearSpeed, nullptr, LV_ALIGN_IN_LEFT_MID, 15, 0);

    lblLinearSpeed = lv_label_create(ggLinearSpeed, nullptr);
    lv_label_set_text_fmt(lblLinearSpeed, "%.2f м/сек", 0.0f);
    lv_obj_set_style_local_text_font(lblLinearSpeed, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_theme_get_font_subtitle());
    lv_obj_align(lblLinearSpeed, ggLinearSpeed, LV_ALIGN_IN_BOTTOM_MID, 0, -15);

    barAngularSpeed = lv_bar_create(screen, nullptr);
    lv_obj_set_size(barAngularSpeed, 160, 5);
    lv_bar_set_range(barAngularSpeed, -100, 100);
    lv_bar_set_sym(barAngularSpeed, true);
    lv_obj_align(barAngularSpeed, nullptr, LV_ALIGN_IN_BOTTOM_LEFT, 15, -25);

    lblAngularSpeed = lv_label_create(screen, nullptr);
    lv_label_set_text_fmt(lblAngularSpeed, "%.2f рад/сек", 0.0f);
    lv_label_set_align(lblAngularSpeed, LV_LABEL_ALIGN_CENTER);
    lv_obj_set_style_local_text_font(lblAngularSpeed, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_theme_get_font_subtitle());
    lv_obj_align(lblAngularSpeed, barAngularSpeed, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);

    cntPosition = lv_cont_create(screen, nullptr);
    lv_obj_set_size(cntPosition, 100, 60);
    lv_cont_set_layout(cntPosition, LV_LAYOUT_OFF);
    lv_obj_align(cntPosition, nullptr, LV_ALIGN_IN_RIGHT_MID, -25, 0);

    lblX = lv_label_create(cntPosition, nullptr);
    lv_label_set_text_fmt(lblX, "X: ");
    lv_obj_align(lblX, cntPosition, LV_ALIGN_IN_TOP_LEFT, 10, 10);

    lblXValue = lv_label_create(cntPosition, nullptr);
    lv_label_set_text_fmt(lblXValue, "%6.2f м", 0.0);
    lv_obj_align(lblXValue, cntPosition, LV_ALIGN_IN_TOP_RIGHT, -10, 10);

    lblY = lv_label_create(cntPosition, nullptr);
    lv_label_set_text(lblY, "Y: ");
    lv_obj_align(lblY, cntPosition, LV_ALIGN_IN_BOTTOM_LEFT, 10, -10);

    lblYValue = lv_label_create(cntPosition, nullptr);
    lv_label_set_text_fmt(lblYValue, "%6.2f м", 0.0);
    lv_obj_align(lblYValue, cntPosition, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -10);

    cntPlatformStatus = lv_cont_create(screen, nullptr);
    lv_obj_set_size(cntPlatformStatus, 120, 30);
    lv_cont_set_layout(cntPlatformStatus, LV_LAYOUT_CENTER);
    lv_obj_align(cntPlatformStatus, nullptr, LV_ALIGN_IN_BOTTOM_RIGHT, -15, -10);
    lv_obj_set_style_local_bg_color(cntPlatformStatus, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, lv_color_make(0, 128, 0));

    lblPlatformStatus = lv_label_create(cntPlatformStatus, nullptr);
    lv_label_set_text_fmt(lblPlatformStatus, "СТАТУС: %s", "OK");

    timerTaskHandle = lv_task_create(timerTaskHandler, 3000, LV_TASK_PRIO_MID, this);
}

void IndicatorsScreen::deinitializeGui() {
    lv_task_del(timerTaskHandle);

    lv_obj_del(imgBackground);

    lv_obj_del(lblPlatformName);
    lv_obj_del(lblWiFiStatus);
    lv_obj_del(lblBatteryStatus);

    lv_obj_del(ggLinearSpeed);
    lv_obj_del(lblLinearSpeed);

    lv_obj_del(barAngularSpeed);
    lv_obj_del(lblAngularSpeed);

    lv_obj_del(cntPosition);
    lv_obj_del(lblX);
    lv_obj_del(lblXValue);
    lv_obj_del(lblY);
    lv_obj_del(lblYValue);

    lv_obj_del(cntPlatformStatus);
    lv_obj_del(lblPlatformStatus);
}

void IndicatorsScreen::timerTaskHandler(lv_task_t *task) {
    auto *instance = static_cast<IndicatorsScreen *>(task->user_data);
    instance->timerTask();
}

float rand(float min, float max) {
    float scale = (float)rand() / (float) RAND_MAX;
    return min + scale * (max - min);
}

void IndicatorsScreen::timerTask() {
    float linearSpeed = rand(MIN_LINEAR_SPEED, MAX_LINEAR_SPEED);
    updateLinearSpeed(linearSpeed);

    float angularSpeed = rand(MIN_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
    updateAngularSpeed(angularSpeed);

    float x = rand(0, 200);
    float y = rand(0, 200);
    updatePosition(x, y);

    updateWiFiStatus(WiFiStatus(rand() % (int)WiFiStatus::Strange4));

    updateBatteryStatus(BatteryStatus(rand() % (int)BatteryStatus::ChargingPercent100));

    updatePlatformStatus(PlatformStatus(rand() % (int)PlatformStatus::FAULT));
}

void IndicatorsScreen::LinearSpeedFormatterCallback(lv_obj_t *gauge, char *buf, int bufSize, int32_t value) {
    int pos = lv_snprintf(buf, bufSize, "%.2f", mapInt16ToFloat(value, -100, 100, MIN_LINEAR_SPEED, MAX_LINEAR_SPEED));

    // Remove trailing 0 and . (i.e. 1.00 -> 1)
    if (pos > 0) {
        while (buf[pos - 1] == '0')
            pos--;
        while (buf[pos - 1] == '.')
            pos--;
        buf[pos] = '\0';
    }
}

void IndicatorsScreen::updateLinearSpeed(float linearSpeed) {
    int16_t linearSpeedGaugeValue = mapFloatToInt16(linearSpeed, MIN_LINEAR_SPEED, MAX_LINEAR_SPEED, -100, 100);
    setGaugeValueWithAnimation(ggLinearSpeed, linearSpeedGaugeValue);

    lv_label_set_text_fmt(lblLinearSpeed, "%.2f м/сек", abs(linearSpeed));
    lv_obj_realign(lblLinearSpeed);
}

void IndicatorsScreen::updateAngularSpeed(float angularSpeed) {
    int16_t angularSpeedGaugeValue = mapFloatToInt16(angularSpeed, MIN_ANGULAR_SPEED, MAX_ANGULAR_SPEED, -100, 100);
    lv_bar_set_value(barAngularSpeed, angularSpeedGaugeValue, LV_ANIM_ON);

    lv_label_set_text_fmt(lblAngularSpeed, "%.2f рад/сек", abs(angularSpeed));
    lv_obj_realign(lblAngularSpeed);
}

void IndicatorsScreen::updatePosition(float x, float y) {
    lv_label_set_text_fmt(lblXValue, "%6.2f м", x);
    lv_obj_realign(lblXValue);

    lv_label_set_text_fmt(lblYValue, "%6.2f м", y);
    lv_obj_realign(lblYValue);
}

void IndicatorsScreen::updateWiFiStatus(WiFiStatus status) {
    lv_label_set_text(lblWiFiStatus, wiFiStatusSymbol(status));
    lv_obj_realign(lblWiFiStatus);
}

void IndicatorsScreen::updateBatteryStatus(BatteryStatus status) {
    lv_label_set_text(lblBatteryStatus, batteryStatusSymbol(status));
    lv_obj_realign(lblBatteryStatus);
}

void IndicatorsScreen::updatePlatformStatus(PlatformStatus status) {
    lv_obj_set_style_local_bg_color(cntPlatformStatus, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, platformStatusColor(status));

    lv_label_set_text_fmt(lblPlatformStatus, "СТАТУС: %s", platformStatusText(status));
    lv_obj_realign(lblPlatformStatus);
}
