//
// Created by Maxim Dobryakov on 12/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_JOYSTICKEVENTSDATASOURCE_H
#define AGV_REMOTE_CONTROL_JOYSTICKEVENTSDATASOURCE_H

#include "TaskBase.h"

#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <driver/adc.h>

class JoystickEventsDataSource: public TaskBase {
private:
    const char* LogTag = "JoystickEventsDataSource";

    const uint32_t MEASUREMENT_INTERVAL = 1000; // ms

    static const adc_bits_width_t ADC_BITS_WIDTH = ADC_WIDTH_BIT_12;

    const adc1_channel_t X_AXIS_ADC_CHANNEL = ADC1_CHANNEL_5;
    const adc1_channel_t Y_AXIS_ADC_CHANNEL = ADC1_CHANNEL_4;

public:
    QueueHandle_t joystickRawEventsQueueHandler = nullptr;

    JoystickEventsDataSource();

private:
    void initializeADCDriver();

    void initializeOutputEventsQueue();

    void collectEvent();

    [[noreturn]] void task() override;

public:
    static uint32_t getMaxPossibleValue();
};

#endif //AGV_REMOTE_CONTROL_JOYSTICKEVENTSDATASOURCE_H
