//
// Created by Maxim Dobryakov on 11/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_ROSCLIENT_H
#define AGV_REMOTE_CONTROL_ROSCLIENT_H


#include <cstdint>
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

    const std::string MAX_LINEAR_SPEED_PARAM = "/agv_remote_control/max_linear_speed";
    const std::string MAX_ANGULAR_SPEED_PARAM = "/agv_remote_control/max_angular_speed";

    const std::string NAVIGATION_TOPIC_NAME = "/turtle1/cmd_vel";

    uint32_t rosMasterAddress;
    uint16_t rosMasterPort;

    ros::NodeHandle *nodeHandle = nullptr;

    geometry_msgs::Twist navigationMessage = {};
    ros::Publisher *navigationMessagePublisher = nullptr;

    double MaxLinearSpeed = 1; // m/sec (initiated by default value)
    double MaxAngularSpeed = M_PI_2; // rad/sec (initiated by default value)

public:
    RosClient(uint32_t rosMasterAddress, uint16_t rosMasterPort);

    void connect();
    void disconnect();

    bool sendNavigationMessage(double linearSpeed, double angularSpeed);

    double getMaxLinearSpeed() const { return MaxLinearSpeed; }
    double getMaxAngularSpeed() const { return MaxAngularSpeed; }

private:
    void task() override;

    bool readParam(const std::string &paramName, double &value);
};


#endif //AGV_REMOTE_CONTROL_ROSCLIENT_H
