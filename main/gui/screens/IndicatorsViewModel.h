//
// Created by Maxim Dobryakov on 13/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_INDICATORSVIEWMODEL_H
#define AGV_REMOTE_CONTROL_INDICATORSVIEWMODEL_H

#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <network/WiFiStatus.h>
#include <gui/screens/utils/BatteryStatus.h>
#include <gui/screens/utils/PlatformStatus.h>

class IndicatorsViewModel {
private:
    static char const *LOG_TAG;

    SemaphoreHandle_t semaphoreHandle = nullptr;

public:
    std::string platformName;

    WiFiStatus wiFiStatus = WiFiStatus::NotConnected;
    BatteryStatus batteryStatus = BatteryStatus::NotFound;

    double linearSpeed = 0.0;
    double angularSpeed = 0.0;

    double xPosition = 0.0;
    double yPosition = 0.0;

    PlatformStatus platformStatus = PlatformStatus::OK;

public:
    IndicatorsViewModel();
    virtual ~IndicatorsViewModel();

    void takeLock();
    void giveLock();

};


#endif //AGV_REMOTE_CONTROL_INDICATORSVIEWMODEL_H
