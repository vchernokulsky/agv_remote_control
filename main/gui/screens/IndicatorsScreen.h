//
// Created by Maxim Dobryakov on 06/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_INDICATORSSCREEN_H
#define AGV_REMOTE_CONTROL_INDICATORSSCREEN_H


#include "../ScreenBase.h"

#include <cmath>
#include "utils/WiFiStatus.h"
#include "utils/BatteryStatus.h"
#include "utils/PlatformStatus.h"

class IndicatorsScreen : public ScreenBase {
private:
    constexpr static char const *LOG_TAG = "IndicatorsScreen";

    const lv_color_t NEEDLE_COLORS[1];

    constexpr static const float MIN_LINEAR_SPEED = -2;
    constexpr static const float MAX_LINEAR_SPEED = 2;

    const float MIN_ANGULAR_SPEED = -M_PI_2;
    const float MAX_ANGULAR_SPEED = M_PI_2;

    lv_task_t *timerTaskHandle = nullptr;

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

    static void timerTaskHandler(lv_task_t *task);
    void timerTask();

    static void LinearSpeedFormatterCallback(lv_obj_t *gauge, char *buf, int bufSize, int32_t value);
public:
    lv_obj_t *screen = nullptr;

    IndicatorsScreen();
    virtual ~IndicatorsScreen() = default;

    void initializeGui() override;
    void deinitializeGui() override;

    void updateLinearSpeed(float linearSpeed);
    void updateAngularSpeed(float angularSpeed);
    void updatePosition(float x, float y);
    void updateWiFiStatus(WiFiStatus status);
    void updateBatteryStatus(BatteryStatus status);
    void updatePlatformStatus(PlatformStatus status);
};

#endif //AGV_REMOTE_CONTROL_INDICATORSSCREEN_H
