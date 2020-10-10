//
// Created by Maxim Dobryakov on 18/09/2020.
//

#include "JoystickController.h"
#include "smoothers/RunningAverageSmoother.h"


#include <esp_log.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

JoystickController::JoystickController() {
    initializeADCDriver();
}

void JoystickController::initializeADCDriver() {
    ESP_LOGV(LOG_TAG, "Initialize ADC driver");

    ESP_LOGV(LOG_TAG, "ADC Two Point Calibration: %s",
             esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK ? "Supported" : "Not Supported");
    ESP_LOGV(LOG_TAG, "ADC VRef Calibration: %s",
             esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK ? "Supported" : "Not Supported");

    ESP_ERROR_CHECK(adc1_config_width(ADC_BITS_WIDTH));
    ESP_ERROR_CHECK(adc1_config_channel_atten(X_AXIS_ADC_CHANNEL, ADC_ATTEN_DB_11));
    ESP_ERROR_CHECK(adc1_config_channel_atten(Y_AXIS_ADC_CHANNEL, ADC_ATTEN_DB_11));
}

bool JoystickController::calibrateCenter(uint32_t &xCenter, uint32_t &yCenter) {
    ESP_LOGV(LOG_TAG, "Calibrate joystick center");

    uint32_t x = 0;
    uint32_t y = 0;

    RunningAverageSmoother xCenterSmoother;
    RunningAverageSmoother yCenterSmoother;

    const uint32_t NumberOfMeasurements = 20;
    for(int measurementNumber = 0; measurementNumber < NumberOfMeasurements; measurementNumber++) {
        uint32_t xRawValue, yRawValue;
        if(!collectRawPosition(xRawValue, yRawValue)) {
            ESP_LOGE(LOG_TAG, "Can't calibrate center.");
            return false;
        }

        x += xCenterSmoother.smooth(xRawValue);
        y += yCenterSmoother.smooth(yRawValue);
    }

    xCenter = x / NumberOfMeasurements;
    yCenter = y / NumberOfMeasurements;

    ESP_LOGD(LOG_TAG, "Calibrated joystick center: (%u, %u)", xCenter, yCenter);

    return true;
}

bool JoystickController::collectPosition(uint32_t &xValue, uint32_t &yValue) {
    uint32_t xRawValue, yRawValue;
    if (!collectRawPosition(xRawValue, yRawValue))
        return false;

    xValue = xSmoother.smooth(xRawValue);
    yValue = ySmoother.smooth(yRawValue);

    return true;
}

bool JoystickController::collectRawPosition(uint32_t &xRawValue, uint32_t &yRawValue) {
    xRawValue = adc1_get_raw(X_AXIS_ADC_CHANNEL);
    yRawValue = adc1_get_raw(Y_AXIS_ADC_CHANNEL);

    return xRawValue != -1 && yRawValue != -1;
}
