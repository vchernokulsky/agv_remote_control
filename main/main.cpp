#include "JoystickEventsDataSource.h"
#include "JoystickEventsProcessor.h"
#include "JoystickEventsSink.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" void app_main();

void app_main() {
    vTaskDelay(pdMS_TO_TICKS(1000));

    auto *pJoystickEventsDataSource = new JoystickEventsDataSource();
    pJoystickEventsDataSource->runTask("JoystickEventsDataSource", 2, 4 * configMINIMAL_STACK_SIZE);

    auto *pJoystickEventsProcessor = new JoystickEventsProcessor(
            pJoystickEventsDataSource->joystickRawEventsQueueHandler,
            JoystickEventsDataSource::getMaxPossibleValue());
    pJoystickEventsProcessor->runTask("JoystickEventsProcessor", 2, 4 * configMINIMAL_STACK_SIZE);

    auto *pJoystickEventsSink = new JoystickEventsSink(pJoystickEventsProcessor->joystickSpeedEventsQueueHandler);
    pJoystickEventsSink->runTask("JoystickEventsSink", 3, 4 * configMINIMAL_STACK_SIZE);
}
