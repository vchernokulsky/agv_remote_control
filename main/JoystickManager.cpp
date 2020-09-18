//
// Created by Maxim Dobryakov on 18/09/2020.
//

#include "JoystickManager.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

JoystickManager::JoystickManager(uint32_t rosMasterAddress, uint16_t rosMasterPort)
    : rosMasterAddress(rosMasterAddress),
      rosMasterPort(rosMasterPort),
      publisher(TopicName, &twistMsg) {
    joystickController.calibrateCenter(xCenter, yCenter);
    initializeConnectionToRos();
}

//TODO: implement error handling
void JoystickManager::initializeConnectionToRos() {
    nodeHandle.initNode(rosMasterAddress, rosMasterPort);
    while (!nodeHandle.connected()) {
        ESP_LOGI(LogTag, "Wait connection to ROS Master...");

        nodeHandle.spinOnce();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    nodeHandle.advertise(publisher);
    nodeHandle.spinOnce();
}

bool JoystickManager::joystickFlow() {
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

bool JoystickManager::convertJoystickPosition(uint32_t xValue, uint32_t yValue, double &linearSpeed, double &angularSpeed) {
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
bool JoystickManager::sendNavigationMessage(double linearSpeed, double angularSpeed) {
    if (linearSpeed != 0 || angularSpeed != 0) { //TODO: probably will be problem with != 0 for double
        twistMsg.linear.x = linearSpeed;
        twistMsg.angular.z = angularSpeed;
        publisher.publish(&twistMsg);
    }

    nodeHandle.spinOnce();

    return true;
}

//TODO: Implement cancellation (required to leave resources after delete)
[[noreturn]] void JoystickManager::task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;) {
        TickType_t delay = joystickFlow() ? MEASUREMENT_INTERVAL : 5000;

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(delay));
    }
}
