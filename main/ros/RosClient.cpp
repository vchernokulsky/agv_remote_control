//
// Created by Maxim Dobryakov on 11/10/2020.
//

#include "RosClient.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

const char *RosClient::LOG_TAG = "RosClient";

using namespace std::placeholders;

RosClient::RosClient(uint32_t rosMasterAddress, uint16_t rosMasterPort) :
    rosMasterAddress(rosMasterAddress),
    rosMasterPort(rosMasterPort)
{
}

void RosClient::connect() {
    assert(nodeHandle == nullptr);

    ESP_LOGV(LOG_TAG, "Initialize connection to ROS");

    nodeHandle = new NodeHandle();
    nodeHandle->getHardware()->onConnect = std::bind(&RosClient::fireOnConnect, this);
    nodeHandle->getHardware()->onDisconnect = std::bind(&RosClient::fireOnDisconnect, this);
    nodeHandle->initNode(rosMasterAddress, rosMasterPort);

    navigationMessagePublisher = new ros::Publisher(CONFIG_AGV_RC_ROS_NAVIGATION_TOPIC_NAME, &navigationMessage);
    if(!nodeHandle->advertise(*navigationMessagePublisher)) {
        ESP_LOGE(LOG_TAG, "Can't advertise navigation publisher. Achieved MAX_PUBLISHERS limit.");
        abort();
    }

    positionMessageSubscriber = new ros::Subscriber<nav_msgs::Odometry, RosClient>(
            CONFIG_AGV_RC_ROS_POSITION_TOPIC_NAME,
            &RosClient::positionMessageSubscriberHandler,
            this);
    if(!nodeHandle->subscribe(*positionMessageSubscriber)) {
        ESP_LOGE(LOG_TAG, "Can't subscribe position subscriber. Achieved MAX_SUBSCRIBERS limit.");
        abort();
    }

    platformStatusMessageSubscriber = new ros::Subscriber<std_msgs::Int32, RosClient>(
            CONFIG_AGV_RC_ROS_STATUS_TOPIC_NAME,
            &RosClient::platformStatusMessageSubscriberHandler,
            this);
    if(!nodeHandle->subscribe(*platformStatusMessageSubscriber)) {
        ESP_LOGE(LOG_TAG, "Can't subscribe platform status subscriber. Achieved MAX_SUBSCRIBERS limit.");
        abort();
    }

    logMessageSubscriber = new ros::Subscriber<rosserial_msgs::Log, RosClient>(
            CONFIG_AGV_RC_ROS_LOG_TOPIC_NAME,
            &RosClient::logMessageSubscriberHandler,
            this);
    if(!nodeHandle->subscribe(*logMessageSubscriber)) {
        ESP_LOGE(LOG_TAG, "Can't subscribe log subscriber. Achieved MAX_SUBSCRIBERS limit.");
        abort();
    }

    while (!nodeHandle->connected()) {
        ESP_LOGD(LOG_TAG, "Wait connection to ROS Master...");

        nodeHandle->spinOnce(); //FYI: Ignore useless errors from result

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGD(LOG_TAG, "Connected to ROS Master");

    if (!readParam(CONFIG_AGV_RC_ROS_PLATFORM_NAME_PARAM, PlatformName))
        ESP_LOGW(LOG_TAG, "Use default %s param value: %s", CONFIG_AGV_RC_ROS_PLATFORM_NAME_PARAM, PlatformName.c_str());
    if (!readParam(CONFIG_AGV_RC_ROS_MAX_LINEAR_SPEED_PARAM, MaxLinearSpeed))
        ESP_LOGW(LOG_TAG, "Use default %s param value: %f", CONFIG_AGV_RC_ROS_MAX_LINEAR_SPEED_PARAM, MaxLinearSpeed);
    if (!readParam(CONFIG_AGV_RC_ROS_MAX_ANGULAR_SPEED_PARAM, MaxAngularSpeed))
        ESP_LOGW(LOG_TAG, "Use default %s param value: %f", CONFIG_AGV_RC_ROS_MAX_ANGULAR_SPEED_PARAM, MaxAngularSpeed);

    runTask("ros-client-loop", 2, 4 * configMINIMAL_STACK_SIZE);

    fireOnConnect();
}

void RosClient::disconnect() {
    assert(nodeHandle != nullptr);

    cancelTask(true);

    delete nodeHandle;
    nodeHandle = nullptr;

    delete navigationMessagePublisher;
    navigationMessagePublisher = nullptr;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
    delete positionMessageSubscriber;
    positionMessageSubscriber = nullptr;

    delete platformStatusMessageSubscriber;
    platformStatusMessageSubscriber = nullptr;

    delete logMessageSubscriber;
    logMessageSubscriber = nullptr;
#pragma GCC diagnostic pop

    fireOnDisconnect();
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

void RosClient::positionMessageSubscriberHandler(const nav_msgs::Odometry &message) {
    if (onPositionMessage)
        onPositionMessage(message);
}

void RosClient::platformStatusMessageSubscriberHandler(const std_msgs::Int32 &message) {
    if (onPlatformStatusMessage)
        onPlatformStatusMessage((PlatformStatus)message.data);
}

void RosClient::logMessageSubscriberHandler(const rosserial_msgs::Log &message) {
    if (onLogMessage)
        onLogMessage(message);
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

bool RosClient::readParam(const std::string &paramName, std::string &value) {
    assert(nodeHandle->connected());

    char *stringValue = new char[256];
    if (!nodeHandle->getParam(paramName.c_str(), &stringValue)) {
        ESP_LOGW(LOG_TAG, "Can't read %s param", paramName.c_str());
        delete[] stringValue;
        return false;
    }

    value = stringValue;
    delete[] stringValue;

    ESP_LOGD(LOG_TAG, "Read %s param: %s", paramName.c_str(), value.c_str());
    return true;
}

void RosClient::fireOnConnect() {
    if (onConnect)
        onConnect(PlatformName);
}

void RosClient::fireOnDisconnect() const {
    if (onDisconnect)
        onDisconnect();
}
