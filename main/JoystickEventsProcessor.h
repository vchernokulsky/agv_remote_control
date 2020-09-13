//
// Created by Maxim Dobryakov on 12/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_JOYSTICKEVENTSPROCESSOR_H
#define AGV_REMOTE_CONTROL_JOYSTICKEVENTSPROCESSOR_H

#include "TaskBase.h"
#include "JoystickEvents.h"

#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class JoystickEventsProcessor: public TaskBase {
private:
    const char* LogTag = "JoystickEventsProcessor";

    const double MaxLinearSpeed = 1; // m/sec
    const double MaxAngularSpeed = M_PI_2; // rad/sec

    QueueHandle_t joystickRawEventsQueueHandler = nullptr;
    uint32_t maxPossibleValue;

public:
    QueueHandle_t joystickSpeedEventsQueueHandler = nullptr;

    explicit JoystickEventsProcessor(QueueHandle_t joystickRawEventsQueueHandler, uint32_t maxPossibleValue);

private:
    void initializeOutputEventsQueue();

    void processEvent();

    JoystickRawEvent normalizeRawEvent(JoystickRawEvent joystickRawEvent);

    JoystickSpeedEvent convertRawEventToSpeedEvent(JoystickRawEvent event);

    [[noreturn]] void task() override;
};

#endif //AGV_REMOTE_CONTROL_JOYSTICKEVENTSPROCESSOR_H
