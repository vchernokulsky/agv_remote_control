//
// Created by Maxim Dobryakov on 12/09/2020.
//

#include "JoystickEventsSink.h"
#include "JoystickEvents.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

JoystickEventsSink::JoystickEventsSink(QueueHandle_t joystickSpeedEventsQueueHandler) {
    assert(joystickSpeedEventsQueueHandler != nullptr);

    this->joystickSpeedEventsQueueHandler = joystickSpeedEventsQueueHandler;
}

void JoystickEventsSink::sendEvent() {
    JoystickSpeedEvent joystickSpeedEvent;

    BaseType_t receiveResult = xQueueReceive(joystickSpeedEventsQueueHandler, &joystickSpeedEvent, (TickType_t) 10);
    if (receiveResult == pdTRUE) {
        ESP_LOGI(LogTag, "Send Event: %f, %f", joystickSpeedEvent.linearSpeed, joystickSpeedEvent.angularSpeed);

        //TODO: implement
    }
}

[[noreturn]] void JoystickEventsSink::task() {
    for(;;) {
        sendEvent();

        taskYIELD();
    }
}
