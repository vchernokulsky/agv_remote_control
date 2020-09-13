//
// Created by Maxim Dobryakov on 12/09/2020.
//

#include "JoystickEventsDataSource.h"
#include "JoystickEvents.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

JoystickEventsDataSource::JoystickEventsDataSource() {
    initializeADCDriver();
    initializeOutputEventsQueue();
}

void JoystickEventsDataSource::initializeADCDriver() {
    ESP_LOGV(LogTag, "ADC Two Point Calibration: %s",
             esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK ? "Supported" : "Not Supported");
    ESP_LOGV(LogTag, "ADC VRef Calibration: %s",
             esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK ? "Supported" : "Not Supported");

    ESP_ERROR_CHECK(adc1_config_width(ADC_BITS_WIDTH));
    ESP_ERROR_CHECK(adc1_config_channel_atten(X_AXIS_ADC_CHANNEL, ADC_ATTEN_DB_11));
    ESP_ERROR_CHECK(adc1_config_channel_atten(Y_AXIS_ADC_CHANNEL, ADC_ATTEN_DB_11));
}

void JoystickEventsDataSource::initializeOutputEventsQueue() {
    joystickRawEventsQueueHandler = xQueueCreate(1, sizeof(JoystickRawEvent));
    if (joystickRawEventsQueueHandler == nullptr) {
        ESP_LOGE(LogTag, "Can't create joystick raw events queue!");
        abort();
    }
}

void JoystickEventsDataSource::collectEvent() {
    uint32_t xRawValue = adc1_get_raw(X_AXIS_ADC_CHANNEL);
    uint32_t yRawValue = adc1_get_raw(Y_AXIS_ADC_CHANNEL);

    JoystickRawEvent joystickRawEvent = {
            xRawValue,
            yRawValue
    };

    BaseType_t sendResult = xQueueSend(joystickRawEventsQueueHandler, &joystickRawEvent, (TickType_t) 0);
    if (sendResult != pdTRUE) {
        ESP_LOGW(LogTag, "Can't send joystick raw event. Skip it. Error code: %d", sendResult);
        //TODO: remove event from queue if it's full and insert new event
    }
}

[[noreturn]] void JoystickEventsDataSource::task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;) {
        collectEvent();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MEASUREMENT_INTERVAL));
    }
}

uint32_t JoystickEventsDataSource::getMaxPossibleValue() {
    return pow(2, 9 + ADC_BITS_WIDTH) - 1;
}
