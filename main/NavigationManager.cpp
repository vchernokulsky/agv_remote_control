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
      publisher(TOPIC_NAME.c_str(), &twistMsg) {
    joystickController.calibrateCenter(xCenter, yCenter);
    initializeConnectionToRos();
}

void NavigationManager::initializeConnectionToRos() {
    ESP_LOGV(LOG_TAG, "Initialize connection to ROS");

    nodeHandle.initNode(rosMasterAddress, rosMasterPort);
    while (!nodeHandle.connected()) {
        ESP_LOGD(LOG_TAG, "Wait connection to ROS Master...");

        nodeHandle.spinOnce(); //TODO: implement error handling?

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGD(LOG_TAG, "Connected to ROS Master");

    if (!readParam(MAX_LINEAR_SPEED_PARAM, MaxLinearSpeed))
        ESP_LOGW(LOG_TAG, "Use default %s param value: %f", MAX_LINEAR_SPEED_PARAM.c_str(), MaxLinearSpeed);
    if (!readParam(MAX_ANGULAR_SPEED_PARAM, MaxAngularSpeed))
        ESP_LOGW(LOG_TAG, "Use default %s param value: %f", MAX_ANGULAR_SPEED_PARAM.c_str(), MaxAngularSpeed);

    if(!nodeHandle.advertise(publisher)) {
        ESP_LOGE(LOG_TAG, "Can't advertise navigation publisher. Achieved MAX_PUBLISHERS limit.");
        abort();
    }
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
        if (!sendNavigationMessage(linearSpeed, angularSpeed)) {
            ESP_LOGE(LOG_TAG, "Can't send navigation message");
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

void NavigationManager::task() {
    ESP_LOGV(LOG_TAG, "Start navigation loop");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(!isCancelled) {
        TickType_t delay = navigationLoop() ? MEASUREMENT_INTERVAL : 5000;

        nodeHandle.spinOnce(); //TODO: implement error handling?

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(delay));
    }

    ESP_LOGV(LOG_TAG, "Finish navigation loop");
}

bool NavigationManager::readParam(const std::string &paramName, double &value) {
    assert(nodeHandle.connected());

    float floatValue;
    if (!nodeHandle.getParam(paramName.c_str(), &floatValue)) {
        ESP_LOGW(LOG_TAG, "Can't read %s param", paramName.c_str());
        return false;
    }

    value = (double) floatValue;

    ESP_LOGD(LOG_TAG, "Read %s param: %f", paramName.c_str(), value);
    return true;
}
