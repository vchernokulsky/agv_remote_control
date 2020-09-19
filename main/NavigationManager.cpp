//
// Created by Maxim Dobryakov on 18/09/2020.
//

#include "NavigationManager.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

NavigationManager::NavigationManager(uint32_t rosMasterAddress, uint16_t rosMasterPort)
    : rosMasterAddress(rosMasterAddress),
      rosMasterPort(rosMasterPort),
      publisher(TopicName, &twistMsg) {
    joystickController.calibrateCenter(xCenter, yCenter);
    initializeConnectionToRos();
}

void NavigationManager::initializeConnectionToRos() {
    nodeHandle.initNode(rosMasterAddress, rosMasterPort);
    while (!nodeHandle.connected()) {
        ESP_LOGI(LogTag, "Wait connection to ROS Master...");

        nodeHandle.spinOnce(); //TODO: implement error handling?

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(LogTag, "Connected to ROS Master");

    if (!readParam(MaxLinearSpeedParam, MaxLinearSpeed))
        ESP_LOGW(LogTag, "Use default %s param value: %f", MaxLinearSpeedParam, MaxLinearSpeed);
    if (!readParam(MaxAngularSpeedParam, MaxAngularSpeed))
        ESP_LOGW(LogTag, "Use default %s param value: %f", MaxAngularSpeedParam, MaxAngularSpeed);

    if(!nodeHandle.advertise(publisher)) {
        ESP_LOGE(LogTag, "Can't advertise navigation publisher");
        //TODO: reconnect?
    }
}

bool NavigationManager::joystickFlow() {
    uint32_t xValue, yValue;
    if (!joystickController.collectPosition(xValue, yValue)) {
        ESP_LOGE(LogTag, "Can't collect joystick position");
        return false;
    }

    double linearSpeed, angularSpeed;
    if (!convertJoystickPosition(xValue, yValue, linearSpeed, angularSpeed)) {
        ESP_LOGE(LogTag, "Can't convert joystick position");
        return false;
    }

    const double tolerance = 0.01;
    if (abs(linearSpeed) > tolerance || abs(angularSpeed) > tolerance) {
        if (!sendNavigationMessage(linearSpeed, angularSpeed)) {
            ESP_LOGE(LogTag, "Can't send navigation message");
            return false;
        }
    }

    return true;
}

bool NavigationManager::convertJoystickPosition(uint32_t xValue, uint32_t yValue, double &linearSpeed, double &angularSpeed) {
    double linearSpeedValue = yValue;
    double angularSpeedValue = xValue;

    linearSpeed = MaxLinearSpeed * ((linearSpeedValue - (double)yCenter) / (double)yCenter);
    angularSpeed = MaxAngularSpeed * ((angularSpeedValue - (double)xCenter) / (double)xCenter);

    linearSpeed = -linearSpeed; // to make movement direction correspond to joystick direction

    // ignore noise around joystick center
    const double tolerance = 0.08;
    if (abs(linearSpeed) < tolerance)
        linearSpeed = 0;
    if (abs(angularSpeed) < tolerance)
        angularSpeed = 0;

    return true;
}

bool NavigationManager::sendNavigationMessage(double linearSpeed, double angularSpeed) {
    twistMsg.linear.x = linearSpeed;
    twistMsg.angular.z = angularSpeed;
    return publisher.publish(&twistMsg) > 0;
}

//TODO: Implement cancellation (required to leave resources after delete)
[[noreturn]] void NavigationManager::task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;) {
        TickType_t delay = joystickFlow() ? MEASUREMENT_INTERVAL : 5000;

        nodeHandle.spinOnce(); //TODO: implement error handling?

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(delay));
    }
}

bool NavigationManager::readParam(const char *paramName, double &value) {
    assert(nodeHandle.connected());

    float floatValue;
    if (!nodeHandle.getParam(paramName, &floatValue)) {
        ESP_LOGW(LogTag, "Can't read %s param", paramName);
        return false;
    }

    value = (double) floatValue;

    ESP_LOGD(LogTag, "Read %s param: %f", paramName, value);
    return true;
}
