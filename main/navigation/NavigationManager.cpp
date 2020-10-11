//
// Created by Maxim Dobryakov on 18/09/2020.
//

#include "NavigationManager.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

NavigationManager::NavigationManager(RosClient *rosClient) :
    rosClient(rosClient)
{
    joystickController.calibrateCenter(xCenter, yCenter);
}

bool NavigationManager::navigationLoop() {
    uint32_t xValue, yValue;
    if (!joystickController.collectPosition(xValue, yValue)) {
        ESP_LOGE(LOG_TAG, "Can't collect joystick position");
        return false;
    }

    double linearSpeed, angularSpeed;
    if (!convertJoystickPosition(xValue, yValue, linearSpeed, angularSpeed)) {
        ESP_LOGE(LOG_TAG, "Can't convert joystick position");
        return false;
    }

    const double tolerance = 0.01;
    if (abs(linearSpeed) > tolerance || abs(angularSpeed) > tolerance) {
        if (!rosClient->sendNavigationMessage(linearSpeed, angularSpeed)) {
            ESP_LOGE(LOG_TAG, "Can't send navigation message");
            return false;
        }
    }

    return true;
}

bool NavigationManager::convertJoystickPosition(uint32_t xValue, uint32_t yValue, double &linearSpeed, double &angularSpeed) {
    double linearSpeedValue = yValue;
    double angularSpeedValue = xValue;

    linearSpeed = rosClient->getMaxLinearSpeed() * ((linearSpeedValue - (double)yCenter) / (double)yCenter);
    angularSpeed = rosClient->getMaxAngularSpeed() * ((angularSpeedValue - (double)xCenter) / (double)xCenter);

    angularSpeed = -angularSpeed; // to make movement direction correspond to joystick direction

    // ignore noise around joystick center
    const double tolerance = 0.08;
    if (abs(linearSpeed) < tolerance)
        linearSpeed = 0;
    if (abs(angularSpeed) < tolerance)
        angularSpeed = 0;

    return true;
}

void NavigationManager::task() {
    ESP_LOGV(LOG_TAG, "Start navigation loop");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(!isCancelled) {
        TickType_t delay = navigationLoop() ? MEASUREMENT_INTERVAL : 5000;

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(delay));
    }

    ESP_LOGV(LOG_TAG, "Finish navigation loop");
}

