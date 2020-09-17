//
// Created by Maxim Dobryakov on 12/09/2020.
//

#include "JoystickEventsSink.h"
#include "JoystickEvents.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

JoystickEventsSink::JoystickEventsSink(QueueHandle_t joystickSpeedEventsQueueHandler, uint32_t rosMasterAddress, uint16_t rosMasterPort)
    : joystickSpeedEventsQueueHandler(joystickSpeedEventsQueueHandler),
      rosMasterAddress(rosMasterAddress),
      rosMasterPort(rosMasterPort),
      publisher(TopicName, &twistMsg) {
    assert(joystickSpeedEventsQueueHandler != nullptr);
}

void JoystickEventsSink::sendEvent() {
    JoystickSpeedEvent joystickSpeedEvent;

    BaseType_t receiveResult = xQueueReceive(joystickSpeedEventsQueueHandler, &joystickSpeedEvent, (TickType_t) 10);
    if (receiveResult == pdTRUE) {
        ESP_LOGI(LogTag, "Send Event: %f, %f", joystickSpeedEvent.linearSpeed, joystickSpeedEvent.angularSpeed);

        twistMsg.linear.x = joystickSpeedEvent.linearSpeed;
        twistMsg.angular.z = joystickSpeedEvent.angularSpeed;
        publisher.publish(&twistMsg);

        nodeHandle.spinOnce();
    }
}

[[noreturn]] void JoystickEventsSink::task() {
    nodeHandle.initNode(rosMasterAddress, rosMasterPort);
    while (!nodeHandle.connected()) {
        ESP_LOGI(LogTag, "Wait connection of NodeHandle...");

        nodeHandle.spinOnce();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    nodeHandle.advertise(publisher);
    nodeHandle.spinOnce();

    for(;;) {
        sendEvent();

        taskYIELD();
    }
}
