//
// Created by Maxim Dobryakov on 18/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_NAVIGATIONMANAGER_H
#define AGV_REMOTE_CONTROL_NAVIGATIONMANAGER_H

#include "TaskBase.h"
#include "JoystickController.h"

#include <cstdint>
#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <ros.h>
#include <ros_lib/ros/publisher.h>
#include <ros_lib/geometry_msgs/Twist.h>
#include <ros_lib/geometry_msgs/Point.h>

class NavigationManager: public TaskBase {
private:
    const char* LogTag = "NavigationManager";

    const uint32_t MEASUREMENT_INTERVAL = 10; // ms

    const char *MaxLinearSpeedParam = "/agv_remote_control/max_linear_speed";
    const char *MaxAngularSpeedParam = "/agv_remote_control/max_angular_speed";

    double MaxLinearSpeed = 1; // m/sec
    double MaxAngularSpeed = M_PI_2; // rad/sec

    const char* TopicName = "/turtle1/cmd_vel";

    uint32_t rosMasterAddress;
    uint16_t rosMasterPort;

    JoystickController joystickController;

    uint32_t xCenter = 0;
    uint32_t yCenter = 0;

    ros::NodeHandle nodeHandle;
    ros::Publisher publisher;
    geometry_msgs::Twist twistMsg;

public:
    NavigationManager(uint32_t rosMasterAddress, uint16_t rosMasterPort);

private:
    void initializeConnectionToRos();

    bool joystickFlow();

    bool convertJoystickPosition(uint32_t xValue, uint32_t yValue, double &linearSpeed, double &angularSpeed);
    bool sendNavigationMessage(double linearSpeed, double angularSpeed);

    [[noreturn]] void task();

    bool readParam(const char *paramName, double &value);
};


#endif //AGV_REMOTE_CONTROL_NAVIGATIONMANAGER_H
