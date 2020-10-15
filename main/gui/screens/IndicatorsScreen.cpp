//
// Created by Maxim Dobryakov on 06/10/2020.
//

#include "IndicatorsScreen.h"


#include <cstdlib>
#include <gui/screens/utils/StringExt.h>
#include "assets/Background.h"
#include "assets/IconsFont.h"
#include "utils/MapValue.h"
#include "utils/GaugeExt.h"

using namespace std;

const std::string IndicatorsScreen::PLATFORM_NOT_CONNECTED = "Платформа не подключена";

const lv_color_t IndicatorsScreen::NEEDLE_COLORS[] = { lv_color_make(255, 0, 0) };

IndicatorsScreen::IndicatorsScreen(SemaphoreHandle_t guiSemaphore) : ScreenBase(guiSemaphore) {
    viewModel = new IndicatorsViewModel();
}

IndicatorsScreen::~IndicatorsScreen() {
    delete viewModel;
}

void IndicatorsScreen::initializeGui() {
    screen = lv_scr_act();

    imgBackground = lv_img_create(screen, nullptr);
    lv_img_set_src(imgBackground, &Background);

    lblPlatformName = lv_label_create(screen, nullptr);
    lv_label_set_long_mode(lblPlatformName, LV_LABEL_LONG_SROLL_CIRC);
    lv_obj_set_size(lblPlatformName, 220, 20);
    lv_obj_align(lblPlatformName, nullptr, LV_ALIGN_IN_TOP_LEFT, 5, 5);

    lblWiFiStatus = lv_label_create(screen, nullptr);
    lv_label_set_align(lblWiFiStatus, LV_LABEL_ALIGN_CENTER);
    lv_obj_set_style_local_text_font(lblWiFiStatus, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &IconsFont);
    lv_obj_set_size(lblWiFiStatus, 20, 20);
    lv_obj_align(lblWiFiStatus, nullptr, LV_ALIGN_IN_TOP_RIGHT, -25, 5);

    lblBatteryStatus = lv_label_create(screen, nullptr);
    lv_label_set_align(lblBatteryStatus, LV_LABEL_ALIGN_CENTER);
    lv_obj_set_style_local_text_font(lblBatteryStatus, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &IconsFont);
    lv_obj_set_size(lblBatteryStatus, 20, 20);
    lv_obj_align(lblBatteryStatus, nullptr, LV_ALIGN_IN_TOP_RIGHT, -5, 5);

    ggLinearSpeed = lv_gauge_create(screen, nullptr);
    lv_obj_set_size(ggLinearSpeed, 160, 160);
    lv_gauge_set_range(ggLinearSpeed, -100, 100);
    lv_gauge_set_scale(ggLinearSpeed, 270, 9, 5);
    lv_gauge_set_needle_count(ggLinearSpeed, 1, NEEDLE_COLORS);
    lv_gauge_set_formatter_cb(ggLinearSpeed, linearSpeedFormatterCallback);
    lv_obj_set_style_local_pad_inner(ggLinearSpeed, LV_GAUGE_PART_MAIN, LV_STATE_DEFAULT, 15);
    lv_obj_align(ggLinearSpeed, nullptr, LV_ALIGN_IN_LEFT_MID, 15, 0);

    lblLinearSpeed = lv_label_create(ggLinearSpeed, nullptr);
    lv_obj_set_style_local_text_font(lblLinearSpeed, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_theme_get_font_subtitle());
    lv_obj_align(lblLinearSpeed, ggLinearSpeed, LV_ALIGN_IN_BOTTOM_MID, 0, -15);

    barAngularSpeed = lv_bar_create(screen, nullptr);
    lv_obj_set_size(barAngularSpeed, 160, 5);
    lv_bar_set_range(barAngularSpeed, -100, 100);
    lv_bar_set_sym(barAngularSpeed, true);
    lv_obj_align(barAngularSpeed, nullptr, LV_ALIGN_IN_BOTTOM_LEFT, 15, -25);

    lblAngularSpeed = lv_label_create(screen, nullptr);
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
    lv_obj_align(lblXValue, cntPosition, LV_ALIGN_IN_TOP_RIGHT, -10, 10);

    lblY = lv_label_create(cntPosition, nullptr);
    lv_label_set_text(lblY, "Y: ");
    lv_obj_align(lblY, cntPosition, LV_ALIGN_IN_BOTTOM_LEFT, 10, -10);

    lblYValue = lv_label_create(cntPosition, nullptr);
    lv_obj_align(lblYValue, cntPosition, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -10);

    cntPlatformStatus = lv_cont_create(screen, nullptr);
    lv_obj_set_size(cntPlatformStatus, 120, 30);
    lv_cont_set_layout(cntPlatformStatus, LV_LAYOUT_CENTER);
    lv_obj_align(cntPlatformStatus, nullptr, LV_ALIGN_IN_BOTTOM_RIGHT, -15, -10);
    lv_obj_set_style_local_bg_color(cntPlatformStatus, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, lv_color_make(0, 128, 0));

    lblPlatformStatus = lv_label_create(cntPlatformStatus, nullptr);

    updateUiTaskHandle = lv_task_create(timerTaskHandler, 100, LV_TASK_PRIO_MID, this);
}

void IndicatorsScreen::deinitializeGui() {
    lv_task_del(updateUiTaskHandle);

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
    auto *indicatorsScreen = static_cast<IndicatorsScreen *>(task->user_data);

    auto *viewModel = indicatorsScreen->viewModel;

    viewModel->takeLock();
    indicatorsScreen->updatePlatformName(viewModel->platformName);
    indicatorsScreen->updateWiFiStatus(viewModel->wiFiStatus);
    indicatorsScreen->updateBatteryStatus(viewModel->batteryStatus);
    indicatorsScreen->updateLinearSpeed(viewModel->linearSpeed);
    indicatorsScreen->updateAngularSpeed(viewModel->angularSpeed);
    indicatorsScreen->updatePosition(viewModel->xPosition, viewModel->yPosition);
    indicatorsScreen->updatePlatformStatus(viewModel->platformStatus);
    viewModel->giveLock();
}

void IndicatorsScreen::linearSpeedFormatterCallback(lv_obj_t *gauge, char *buf, int bufSize, int32_t value) {
    int pos = lv_snprintf(buf, bufSize, "%.2f", mapInt32ToFloat(value, -100, 100, MIN_LINEAR_SPEED, MAX_LINEAR_SPEED));

    // Remove trailing 0 and . (i.e. 1.00 -> 1)
    if (pos > 0) {
        while (buf[pos - 1] == '0')
            pos--;
        while (buf[pos - 1] == '.')
            pos--;
        buf[pos] = '\0';
    }
}

void IndicatorsScreen::updatePlatformName(const std::string& platformName) {
    auto oldText = lv_label_get_text(lblPlatformName);
    if (platformName != oldText) {
        lv_label_set_text(lblPlatformName, platformName.c_str());
        lv_obj_realign(lblPlatformName);
    }
}

void IndicatorsScreen::updateWiFiStatus(WiFiStatus status) {
    const std::string newText = std::string(wiFiStatusSymbol(status));
    auto oldText = lv_label_get_text(lblWiFiStatus);
    if (newText != oldText) {
        lv_label_set_text(lblWiFiStatus, newText.c_str());
        lv_obj_realign(lblWiFiStatus);
    }    
}

void IndicatorsScreen::updateBatteryStatus(BatteryStatus status) {
    const std::string newText = std::string(batteryStatusSymbol(status));
    auto oldText = lv_label_get_text(lblBatteryStatus);
    if (newText != oldText) {
        lv_label_set_text(lblBatteryStatus, newText.c_str());
        lv_obj_realign(lblBatteryStatus);
    }
}

void IndicatorsScreen::updateLinearSpeed(double linearSpeed) {
    int32_t newValue = mapFloatToInt32((float)linearSpeed, MIN_LINEAR_SPEED, MAX_LINEAR_SPEED, -100, 100);
    int32_t oldValue = lv_gauge_get_value(ggLinearSpeed, 0);

    if (newValue != oldValue) {
        setGaugeValueWithAnimation(ggLinearSpeed, newValue); // with animation
//        lv_gauge_set_value(ggLinearSpeed, 0, newValue); // without animation
    }

    const std::string newText = formatString("%.2f м/сек", abs(linearSpeed));
    auto oldText = lv_label_get_text(lblLinearSpeed);
    if (newText != oldText) {
        lv_label_set_text(lblLinearSpeed, newText.c_str());
        lv_obj_realign(lblLinearSpeed);
    }
}

void IndicatorsScreen::updateAngularSpeed(double angularSpeed) {
    int16_t newValue = mapFloatToInt16((float)angularSpeed, MIN_ANGULAR_SPEED, MAX_ANGULAR_SPEED, -100, 100);
    int16_t oldValue = lv_bar_get_value(barAngularSpeed);

    if (newValue != oldValue) {
        lv_bar_set_value(barAngularSpeed, newValue, LV_ANIM_ON); // with/without animation
    }

    const std::string newText = formatString("%.2f рад/сек", abs(angularSpeed));
    auto oldText = lv_label_get_text(lblAngularSpeed);
    if (newText != oldText) {
        lv_label_set_text(lblAngularSpeed, newText.c_str());
        lv_obj_realign(lblAngularSpeed);
    }
}

void IndicatorsScreen::updatePosition(double x, double y) {
    const std::string newXText = formatString("%6.2f м", x);
    auto oldXText = lv_label_get_text(lblXValue);
    if (newXText != oldXText) {
        lv_label_set_text(lblXValue, newXText.c_str());
        lv_obj_realign(lblXValue);
    }

    const std::string newYText = formatString("%6.2f м", y);
    auto oldYText = lv_label_get_text(lblYValue);
    if (newYText != oldYText) {
        lv_label_set_text(lblYValue, newYText.c_str());
        lv_obj_realign(lblYValue);
    }
}

void IndicatorsScreen::updatePlatformStatus(PlatformStatus status) {
    const std::string newText = formatString("СТАТУС: %s", platformStatusText(status));
    auto oldText = lv_label_get_text(lblPlatformStatus);

    if (newText != oldText) {
        lv_obj_set_style_local_bg_color(cntPlatformStatus, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, platformStatusColor(status));

        lv_label_set_text(lblPlatformStatus, newText.c_str());
        lv_obj_realign(lblPlatformStatus);
    }
}
