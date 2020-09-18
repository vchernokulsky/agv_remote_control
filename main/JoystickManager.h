//
// Created by Maxim Dobryakov on 18/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_JOYSTICKMANAGER_H
#define AGV_REMOTE_CONTROL_JOYSTICKMANAGER_H

#include "TaskBase.h"

#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <driver/adc.h>
#include <ros.h>
#include <ros_lib/ros/publisher.h>
#include <ros_lib/geometry_msgs/Twist.h>

class JoystickManager: public TaskBase {
private:
    const char* LogTag = "JoystickManager";

    const uint32_t MEASUREMENT_INTERVAL = 1000; // ms

    static const adc_bits_width_t ADC_BITS_WIDTH = ADC_WIDTH_BIT_12;

    const adc1_channel_t X_AXIS_ADC_CHANNEL = ADC1_CHANNEL_5;
    const adc1_channel_t Y_AXIS_ADC_CHANNEL = ADC1_CHANNEL_4;

    const double MaxLinearSpeed = 1; // m/sec
    const double MaxAngularSpeed = M_PI_2; // rad/sec

    const char* TopicName = "/turtle1/cmd_vel";

    uint32_t rosMasterAddress;
    uint16_t rosMasterPort;

    ros::NodeHandle nodeHandle;
    ros::Publisher publisher;
    geometry_msgs::Twist twistMsg;

public:
    JoystickManager(uint32_t rosMasterAddress, uint16_t rosMasterPort);

private:
    void initializeADCDriver();
    void initializeConnectionToRos();

    bool joystickFlow();

    bool collectJoystickPosition(uint32_t &xRawValue, uint32_t &yRawValue);
    bool smoothJoystickPosition(uint32_t xRawValue, uint32_t yRawValue, uint32_t &xValue, uint32_t &yValue);
    bool convertJoystickPosition(uint32_t xValue, uint32_t yValue, double &linearSpeed, double &angularSpeed);
    bool sendNavigationMessage(double linearSpeed, double angularSpeed);

    [[noreturn]] void task();
};


#endif //AGV_REMOTE_CONTROL_JOYSTICKMANAGER_H
