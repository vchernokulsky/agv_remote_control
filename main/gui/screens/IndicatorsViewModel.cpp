//
// Created by Maxim Dobryakov on 13/10/2020.
//

#include "IndicatorsViewModel.h"


#include <esp_log.h>

const char *IndicatorsViewModel::LOG_TAG = "IndicatorsViewModel";

const std::string IndicatorsViewModel::PLATFORM_NOT_CONNECTED = "Платформа не подключена";

IndicatorsViewModel::IndicatorsViewModel() {
    semaphoreHandle = xSemaphoreCreateMutex();
}

IndicatorsViewModel::~IndicatorsViewModel() {
    vSemaphoreDelete(semaphoreHandle);
}

void IndicatorsViewModel::takeLock() {
    if (xSemaphoreTake(semaphoreHandle, portMAX_DELAY) == pdFALSE) {
        ESP_LOGE(LOG_TAG, "Can't take semaphore.");
        abort();
    }
}

void IndicatorsViewModel::giveLock() {
    if (xSemaphoreGive(semaphoreHandle) == pdFALSE) {
        ESP_LOGE(LOG_TAG, "Can't give semaphore.");
        abort();
    }
}
