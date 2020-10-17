//
// Created by Maxim Dobryakov on 11/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_ROSCLIENT_H
#define AGV_REMOTE_CONTROL_ROSCLIENT_H


#include <cstdint>
#include <functional>
#include <string>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <rosserial_msgs/Log.h>
#include <std_msgs/Int32.h>
#include "utils/TaskBase.h"
#include "PlatformStatus.h"

class RosClient : TaskBase {
private:
    static const char *LOG_TAG;

    const uint32_t LOOP_INTERVAL = 10; // ms

    uint32_t rosMasterAddress;
    uint16_t rosMasterPort;

    ros::NodeHandle *nodeHandle = nullptr;

    geometry_msgs::Twist navigationMessage = {};
    ros::Publisher *navigationMessagePublisher = nullptr;
    ros::Subscriber<nav_msgs::Odometry, RosClient> *positionMessageSubscriber = nullptr;
    ros::Subscriber<std_msgs::Int32, RosClient> *platformStatusMessageSubscriber = nullptr;
    ros::Subscriber<rosserial_msgs::Log, RosClient> *logMessageSubscriber = nullptr;

    std::string PlatformName;
    double MaxLinearSpeed = 1; // m/sec (initiated by default value)
    double MaxAngularSpeed = M_PI_2; // rad/sec (initiated by default value)

public:
    RosClient(uint32_t rosMasterAddress, uint16_t rosMasterPort);
    virtual ~RosClient() = default;

    void connect();
    void disconnect();

    bool sendNavigationMessage(double linearSpeed, double angularSpeed);

    double getMaxLinearSpeed() const { return MaxLinearSpeed; }
    double getMaxAngularSpeed() const { return MaxAngularSpeed; }

    std::function<void(const std::string &platformName)> onConnect;
    std::function<void()> onDisconnect;
    std::function<void(const nav_msgs::Odometry &message)> onPositionMessage;
    std::function<void(const PlatformStatus platformStatus)> onPlatformStatusMessage;
    std::function<void(const rosserial_msgs::Log &message)> onLogMessage;

private:
    void task() override;

    void positionMessageSubscriberHandler(const nav_msgs::Odometry &message);
    void platformStatusMessageSubscriberHandler(const std_msgs::Int32 &message);
    void logMessageSubscriberHandler(const rosserial_msgs::Log &message);

    bool readParam(const std::string &paramName, double &value);
    bool readParam(const std::string &paramName, std::string &value);

    void fireOnConnect();
    void fireOnDisconnect() const;
};


#endif //AGV_REMOTE_CONTROL_ROSCLIENT_H
