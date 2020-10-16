//
// Created by Maxim Dobryakov on 13/09/2020.
//

#include "TaskBase.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

void TaskBase::runTask(const std::string &taskName, UBaseType_t taskPriority, uint32_t stackDepth) {
    assert(taskHandle == nullptr);

    ESP_LOGV(taskName.c_str(), "Run task");

    this->taskName = taskName;
    isCancelled = false;

    BaseType_t taskResult = xTaskCreate(&taskHandler, taskName.c_str(), stackDepth, this, taskPriority, &taskHandle);

    if (taskResult != pdPASS) {
        ESP_LOGE(taskName.c_str(), "Can't create task handler!");
        abort();
    }
}

void TaskBase::cancelTask(bool waitCancellation) {
    assert(taskHandle != nullptr);

    ESP_LOGV(taskName.c_str(), "Cancel task");

    isCancelled = true;

    if (waitCancellation) {
        while (eTaskGetState(taskHandle) != eDeleted)
            vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void TaskBase::taskHandler(void *parm) {
    auto *pTaskClassInstance = static_cast<TaskBase *>(parm);

    pTaskClassInstance->task();
    vTaskDelete(pTaskClassInstance->taskHandle);
}
