//
// Created by Maxim Dobryakov on 11/10/2020.
//

#include "RosClient.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

RosClient::RosClient(uint32_t rosMasterAddress, uint16_t rosMasterPort) :
    rosMasterAddress(rosMasterAddress),
    rosMasterPort(rosMasterPort)
{
}

void RosClient::connect() {
    assert(nodeHandle == nullptr);

    ESP_LOGV(LOG_TAG, "Initialize connection to ROS");

    nodeHandle = new NodeHandle();
    nodeHandle->initNode(rosMasterAddress, rosMasterPort);
    while (!nodeHandle->connected()) {
        ESP_LOGD(LOG_TAG, "Wait connection to ROS Master...");

        nodeHandle->spinOnce(); //FYI: Ignore useless errors from result

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGD(LOG_TAG, "Connected to ROS Master");

    if (!readParam(MAX_LINEAR_SPEED_PARAM, MaxLinearSpeed))
        ESP_LOGW(LOG_TAG, "Use default %s param value: %f", MAX_LINEAR_SPEED_PARAM.c_str(), MaxLinearSpeed);
    if (!readParam(MAX_ANGULAR_SPEED_PARAM, MaxAngularSpeed))
        ESP_LOGW(LOG_TAG, "Use default %s param value: %f", MAX_ANGULAR_SPEED_PARAM.c_str(), MaxAngularSpeed);

    navigationMessagePublisher = new ros::Publisher(NAVIGATION_TOPIC_NAME.c_str(), &navigationMessage);
    if(!nodeHandle->advertise(*navigationMessagePublisher)) {
        ESP_LOGE(LOG_TAG, "Can't advertise navigation publisher. Achieved MAX_PUBLISHERS limit.");
        abort();
    }

    runTask("ros-client-loop", 2, 4 * configMINIMAL_STACK_SIZE);
}

void RosClient::disconnect() {
    assert(nodeHandle != nullptr);

    cancelTask(true);

    delete nodeHandle;
    delete navigationMessagePublisher;

    nodeHandle = nullptr;
    navigationMessagePublisher = nullptr;
}

void RosClient::task() {
    assert(nodeHandle != nullptr);

    ESP_LOGV(LOG_TAG, "Start RosClient loop");

    TickType_t lastWakeTime = xTaskGetTickCount();
    while(!isCancelled) {
        nodeHandle->spinOnce(); //FYI: Ignore useless errors from result

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_INTERVAL));
    }

    ESP_LOGV(LOG_TAG, "Finish RosClient loop");
}

bool RosClient::sendNavigationMessage(double linearSpeed, double angularSpeed) {
    navigationMessage.linear.x = linearSpeed;
    navigationMessage.angular.z = angularSpeed;
    return navigationMessagePublisher->publish(&navigationMessage) > 0;
}

bool RosClient::readParam(const std::string &paramName, double &value) {
    assert(nodeHandle->connected());

    float floatValue;
    if (!nodeHandle->getParam(paramName.c_str(), &floatValue)) {
        ESP_LOGW(LOG_TAG, "Can't read %s param", paramName.c_str());
        return false;
    }

    value = (double) floatValue;

    ESP_LOGD(LOG_TAG, "Read %s param: %f", paramName.c_str(), value);
    return true;
}
