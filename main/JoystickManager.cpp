//
// Created by Maxim Dobryakov on 18/09/2020.
//

#include "JoystickManager.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

JoystickManager::JoystickManager(uint32_t rosMasterAddress, uint16_t rosMasterPort)
    : rosMasterAddress(rosMasterAddress),
      rosMasterPort(rosMasterPort),
      publisher(TopicName, &twistMsg) {
    initializeADCDriver();
    initializeConnectionToRos();
}

void JoystickManager::initializeADCDriver() {
    ESP_LOGV(LogTag, "ADC Two Point Calibration: %s",
             esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK ? "Supported" : "Not Supported");
    ESP_LOGV(LogTag, "ADC VRef Calibration: %s",
             esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK ? "Supported" : "Not Supported");

    ESP_ERROR_CHECK(adc1_config_width(ADC_BITS_WIDTH));
    ESP_ERROR_CHECK(adc1_config_channel_atten(X_AXIS_ADC_CHANNEL, ADC_ATTEN_DB_11));
    ESP_ERROR_CHECK(adc1_config_channel_atten(Y_AXIS_ADC_CHANNEL, ADC_ATTEN_DB_11));
}

//TODO: implement error handling
void JoystickManager::initializeConnectionToRos() {
    nodeHandle.initNode(rosMasterAddress, rosMasterPort);
    while (!nodeHandle.connected()) {
        ESP_LOGI(LogTag, "Wait connection of NodeHandle...");

        nodeHandle.spinOnce();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    nodeHandle.advertise(publisher);
    nodeHandle.spinOnce();
}

bool JoystickManager::joystickFlow() {
    uint32_t xRawValue;
    uint32_t yRawValue;

    if (!collectJoystickPosition(xRawValue, yRawValue)) {
        ESP_LOGE(LogTag, "Can't collect joystick position from ADC");
        return false;
    }

    uint32_t xValue;
    uint32_t yValue;
    if (!smoothJoystickPosition(xRawValue, yRawValue, xValue, yValue)) {
        ESP_LOGE(LogTag, "Can't smooth joystick position");
        return false;
    }

    double linearSpeed;
    double angularSpeed;
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

bool JoystickManager::collectJoystickPosition(uint32_t &xRawValue, uint32_t &yRawValue) {
    xRawValue = adc1_get_raw(X_AXIS_ADC_CHANNEL);
    yRawValue = adc1_get_raw(Y_AXIS_ADC_CHANNEL);

    return xRawValue != -1 && yRawValue != -1;
}

bool JoystickManager::smoothJoystickPosition(uint32_t xRawValue, uint32_t yRawValue, uint32_t &xValue, uint32_t &yValue) {
    xValue = xRawValue;
    yValue = yRawValue;
    return true;
}

bool JoystickManager::convertJoystickPosition(uint32_t xValue, uint32_t yValue, double &linearSpeed, double &angularSpeed) {
    //TODO: implement correction of zero-point (return 0 around middleValue±10)

    double linearSpeedRawValue = yValue;
    double angularSpeedRawValue = xValue;
    uint32_t maxPossibleValue = pow(2, 9 + ADC_BITS_WIDTH) - 1;
    double middleValue = (maxPossibleValue - 1) / 2.0;

    linearSpeed = MaxLinearSpeed * ((linearSpeedRawValue - middleValue) / middleValue);
    angularSpeed = MaxAngularSpeed * ((angularSpeedRawValue - middleValue) / middleValue);

    angularSpeed = -angularSpeed; // to make movement direction correspond to joystick direction

    return true;
}

//TODO: implement error handling
bool JoystickManager::sendNavigationMessage(double linearSpeed, double angularSpeed) {
    twistMsg.linear.x = linearSpeed;
    twistMsg.angular.z = angularSpeed;
    publisher.publish(&twistMsg);

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
