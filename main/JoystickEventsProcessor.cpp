//
// Created by Maxim Dobryakov on 12/09/2020.
//

#include "JoystickEventsProcessor.h"
#include "JoystickEvents.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

JoystickEventsProcessor::JoystickEventsProcessor(QueueHandle_t joystickRawEventsQueueHandler, uint32_t maxPossibleValue) {
    assert(joystickRawEventsQueueHandler != nullptr);

    this->joystickRawEventsQueueHandler = joystickRawEventsQueueHandler;
    this->maxPossibleValue = maxPossibleValue;

    initializeOutputEventsQueue();
}

void JoystickEventsProcessor::initializeOutputEventsQueue() {
    joystickSpeedEventsQueueHandler = xQueueCreate(1, sizeof(JoystickSpeedEvent));
    if (joystickSpeedEventsQueueHandler == nullptr) {
        ESP_LOGE(LogTag, "Can't create joystick speed events queue!");
        abort();
    }
}

void JoystickEventsProcessor::processEvent() {
    JoystickRawEvent joystickRawEvent;

    BaseType_t receiveResult = xQueueReceive(joystickRawEventsQueueHandler, &joystickRawEvent, (TickType_t) 10);
    if (receiveResult == pdTRUE) {
        JoystickRawEvent normalizedJoystickRawEvent = normalizeRawEvent(joystickRawEvent);
        JoystickSpeedEvent joystickSpeedEvent = convertRawEventToSpeedEvent(normalizedJoystickRawEvent);

        BaseType_t sendResult = xQueueSend(joystickSpeedEventsQueueHandler, &joystickSpeedEvent, (TickType_t) 0);
        if (sendResult != pdTRUE) {
            ESP_LOGW(LogTag, "Can't send joystick speed event. Skip it. Error code: %d", sendResult);
            //TODO: remove event from queue if it's full and insert new event
        }
    }
}

JoystickRawEvent JoystickEventsProcessor::normalizeRawEvent(JoystickRawEvent joystickRawEvent) {
    return joystickRawEvent;
}

JoystickSpeedEvent JoystickEventsProcessor::convertRawEventToSpeedEvent(JoystickRawEvent event) {
    //TODO: implement correction of zero-point (return 0 around middleValue±10)

    double linearSpeedRawValue = event.yRawValue;
    double angularSpeedRawValue = event.xRawValue;
    double middleValue = (maxPossibleValue - 1) / 2.0;

    double linearSpeed = MaxLinearSpeed * ((linearSpeedRawValue - middleValue) / middleValue);
    double angularSpeed = MaxAngularSpeed * ((angularSpeedRawValue - middleValue) / middleValue);

    angularSpeed = -angularSpeed; // to make movement direction correspond to joystick direction

    return {linearSpeed, angularSpeed};
}

[[noreturn]] void JoystickEventsProcessor::task() {
    for(;;) {
        processEvent();

        taskYIELD();
    }
}
