//
// Created by Maxim Dobryakov on 12/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_JOYSTICKEVENTSSINK_H
#define AGV_REMOTE_CONTROL_JOYSTICKEVENTSSINK_H

#include "TaskBase.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class JoystickEventsSink: public TaskBase {
private:
    const char* LogTag = "JoystickEventsSink";

    QueueHandle_t joystickSpeedEventsQueueHandler = nullptr;

public:
    explicit JoystickEventsSink(QueueHandle_t joystickSpeedEventsQueueHandler);

private:
    void sendEvent();

    [[noreturn]] void task() override;
};

#endif //AGV_REMOTE_CONTROL_JOYSTICKEVENTSSINK_H
