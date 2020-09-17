//
// Created by Maxim Dobryakov on 12/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_JOYSTICKEVENTSSINK_H
#define AGV_REMOTE_CONTROL_JOYSTICKEVENTSSINK_H

#include "TaskBase.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <ros.h>
#include <ros_lib/ros/publisher.h>
#include <ros_lib/geometry_msgs/Twist.h>

class JoystickEventsSink: public TaskBase {
private:
    const char* LogTag = "JoystickEventsSink";

    const char* TopicName = "/turtle1/cmd_vel";

    QueueHandle_t joystickSpeedEventsQueueHandler = nullptr;
    uint32_t rosMasterAddress;
    uint16_t rosMasterPort;

    ros::NodeHandle nodeHandle;
    ros::Publisher publisher;
    geometry_msgs::Twist twistMsg;
public:
    explicit JoystickEventsSink(QueueHandle_t joystickSpeedEventsQueueHandler, uint32_t rosMasterAddress, uint16_t rosMasterPort);

private:
    void sendEvent();

    [[noreturn]] void task() override;
};

#endif //AGV_REMOTE_CONTROL_JOYSTICKEVENTSSINK_H
