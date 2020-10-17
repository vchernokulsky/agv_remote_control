//
// Created by Maxim Dobryakov on 06/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_INDICATORSSCREEN_H
#define AGV_REMOTE_CONTROL_INDICATORSSCREEN_H


#include "../ScreenBase.h"

#include <cmath>
#include <string>
#include "utils/WiFiStatusExt.h"
#include "utils/BatteryStatusExt.h"
#include "utils/PlatformStatus.h"
#include "IndicatorsViewModel.h"

class IndicatorsScreen : public ScreenBase {
private:
    static const char *LOG_TAG;

    static const lv_color_t NEEDLE_COLORS[];

    constexpr static const float MIN_LINEAR_SPEED = -2;
    constexpr static const float MAX_LINEAR_SPEED = 2;

    const float MIN_ANGULAR_SPEED = -M_PI_2;
    const float MAX_ANGULAR_SPEED = M_PI_2;

    lv_obj_t *imgBackground = nullptr;

    lv_obj_t *lblPlatformName = nullptr;
    lv_obj_t *lblWiFiStatus = nullptr;
    lv_obj_t *lblBatteryStatus = nullptr;


    lv_obj_t *ggLinearSpeed = nullptr;
    lv_obj_t *lblLinearSpeed = nullptr;

    lv_obj_t *barAngularSpeed = nullptr;
    lv_obj_t *lblAngularSpeed = nullptr;

    lv_obj_t *cntPosition = nullptr;
    lv_obj_t *lblX = nullptr;
    lv_obj_t *lblXValue = nullptr;
    lv_obj_t *lblY = nullptr;
    lv_obj_t *lblYValue = nullptr;

    lv_obj_t *cntPlatformStatus = nullptr;
    lv_obj_t *lblPlatformStatus = nullptr;

    lv_task_t *updateUiTaskHandle = nullptr;
public:
    static const std::string PLATFORM_NOT_CONNECTED;

    IndicatorsViewModel *viewModel = nullptr;

    explicit IndicatorsScreen(SemaphoreHandle_t guiSemaphore);
    virtual ~IndicatorsScreen();

    void initializeGui() override;
    void deinitializeGui() override;

private:
    static void timerTaskHandler(lv_task_t *task);
    static void linearSpeedFormatterCallback(lv_obj_t *gauge, char *buf, int bufSize, int32_t value);

    void updatePlatformName(const std::string& platformName);
    void updateWiFiStatus(WiFiStatus status);
    void updateBatteryStatus(BatteryStatus status);
    void updateLinearSpeed(double linearSpeed);
    void updateAngularSpeed(double angularSpeed);
    void updatePosition(double x, double y);
    void updatePlatformStatus(PlatformStatus status);
};

#endif //AGV_REMOTE_CONTROL_INDICATORSSCREEN_H
