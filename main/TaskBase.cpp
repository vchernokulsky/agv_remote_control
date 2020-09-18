//
// Created by Maxim Dobryakov on 13/09/2020.
//

#include "TaskBase.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

void TaskBase::runTask(const char *taskName, BaseType_t taskPriority, uint32_t stackDepth) {
    BaseType_t taskResult = xTaskCreate(&taskHandler, taskName, stackDepth, this, taskPriority, &taskHandle);

    if (taskResult != pdPASS) {
        ESP_LOGE(taskName, "Can't create task handler!");
        abort();
    }
}

void TaskBase::taskHandler(void *parm) {
    auto *pTaskClassInstance = static_cast<TaskBase *>(parm);

    pTaskClassInstance->task();
    vTaskDelete(pTaskClassInstance->taskHandle);
}