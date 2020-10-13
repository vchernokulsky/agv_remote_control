//
// Created by Maxim Dobryakov on 11/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_ROSCLIENT_H
#define AGV_REMOTE_CONTROL_ROSCLIENT_H


#include <cstdint>
#include <functional>
#include <string>
#include <ros.h>
#include <ros_lib/ros/publisher.h>
#include <ros_lib/geometry_msgs/Twist.h>
#include <ros_lib/geometry_msgs/Point.h>
#include "utils/TaskBase.h"

class RosClient : TaskBase {
private:
    const char *LOG_TAG = "RosClient";

    const uint32_t LOOP_INTERVAL = 10; // ms

    const std::string PLATFORM_NAME_PARAM = "/agv_remote_control/platform_name";
    const std::string MAX_LINEAR_SPEED_PARAM = "/agv_remote_control/max_linear_speed";
    const std::string MAX_ANGULAR_SPEED_PARAM = "/agv_remote_control/max_angular_speed";

    const std::string NAVIGATION_TOPIC_NAME = "/turtle1/cmd_vel";

    uint32_t rosMasterAddress;
    uint16_t rosMasterPort;

    ros::NodeHandle *nodeHandle = nullptr;

    geometry_msgs::Twist navigationMessage = {};
    ros::Publisher *navigationMessagePublisher = nullptr;
    ros::Subscriber<geometry_msgs::Twist, RosClient> *navigationMessageSubscriber = nullptr;

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

    std::function<void(const std::string &platformName)> onConnect = nullptr;
    std::function<void()> onDisconnect = nullptr;
    std::function<void(const geometry_msgs::Twist &message)> onNavigationMessage = nullptr;

private:
    void task() override;

    void navigationMessageSubscriberHandler(const geometry_msgs::Twist &message);

    bool readParam(const std::string &paramName, double &value);
    bool readParam(const std::string &paramName, std::string &value);

    void fireOnConnect();
    void fireOnDisconnect();
};


#endif //AGV_REMOTE_CONTROL_ROSCLIENT_H
