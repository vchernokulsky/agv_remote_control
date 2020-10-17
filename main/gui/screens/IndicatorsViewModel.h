//
// Created by Maxim Dobryakov on 13/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_INDICATORSVIEWMODEL_H
#define AGV_REMOTE_CONTROL_INDICATORSVIEWMODEL_H

#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <network/WiFiStatus.h>
#include <gui/screens/utils/BatteryStatusExt.h>
#include <gui/screens/utils/PlatformStatusExt.h>

class IndicatorsViewModel {
private:
    static const char *LOG_TAG;

    SemaphoreHandle_t semaphoreHandle = nullptr;

public:
    static const std::string PLATFORM_NOT_CONNECTED;

    std::string platformName = PLATFORM_NOT_CONNECTED;

    WiFiStatus wiFiStatus = WiFiStatus::NotConnected;
    BatteryStatus batteryStatus = BatteryStatus::Level0;

    double linearSpeed = 0.0;
    double angularSpeed = 0.0;

    double xPosition = 0.0;
    double yPosition = 0.0;

    PlatformStatus platformStatus = PlatformStatus::UNDEFINED;

public:
    IndicatorsViewModel();
    virtual ~IndicatorsViewModel();

    void takeLock();
    void giveLock();

};


#endif //AGV_REMOTE_CONTROL_INDICATORSVIEWMODEL_H
