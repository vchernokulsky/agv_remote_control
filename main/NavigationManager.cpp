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

//TODO: implement error handling
void NavigationManager::initializeConnectionToRos() {
    nodeHandle.initNode(rosMasterAddress, rosMasterPort);
    while (!nodeHandle.connected()) {
        ESP_LOGI(LogTag, "Wait connection to ROS Master...");

        nodeHandle.spinOnce();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(LogTag, "Connected to ROS Master");

    if (!readParam(MaxLinearSpeedParam, MaxLinearSpeed))
        ESP_LOGW(LogTag, "Use default %s param value: %f", MaxLinearSpeedParam, MaxLinearSpeed);
    if (!readParam(MaxAngularSpeedParam, MaxAngularSpeed))
        ESP_LOGW(LogTag, "Use default %s param value: %f", MaxAngularSpeedParam, MaxAngularSpeed);

    nodeHandle.advertise(publisher);
    nodeHandle.spinOnce();
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

    if (!sendNavigationMessage(linearSpeed, angularSpeed)) {
        ESP_LOGE(LogTag, "Can't send navigation message");
        return false;
    }

    return true;
}

bool NavigationManager::convertJoystickPosition(uint32_t xValue, uint32_t yValue, double &linearSpeed, double &angularSpeed) {
    //TODO: implement correction of zero-point (return 0 around middleValue±10)

    double linearSpeedValue = yValue;
    double angularSpeedValue = xValue;

    linearSpeed = MaxLinearSpeed * ((linearSpeedValue - (double)yCenter) / (double)yCenter);
    angularSpeed = MaxAngularSpeed * ((angularSpeedValue - (double)xCenter) / (double)xCenter);

    linearSpeed = -linearSpeed; // to make movement direction correspond to joystick direction

    //TODO: remove magick constants
    // ignore noise around joystick center
    if (abs(linearSpeed) < 0.05)
        linearSpeed = 0;
    if (abs(angularSpeed) < 0.05)
        angularSpeed = 0;

    return true;
}

//TODO: implement error handling
bool NavigationManager::sendNavigationMessage(double linearSpeed, double angularSpeed) {
    if (linearSpeed != 0 || angularSpeed != 0) { //TODO: probably will be problem with != 0 for double
        twistMsg.linear.x = linearSpeed;
        twistMsg.angular.z = angularSpeed;
        publisher.publish(&twistMsg);
    }

    nodeHandle.spinOnce();

    return true;
}

//TODO: Implement cancellation (required to leave resources after delete)
[[noreturn]] void NavigationManager::task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;) {
        TickType_t delay = joystickFlow() ? MEASUREMENT_INTERVAL : 5000;

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
