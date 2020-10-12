//
// Created by Maxim Dobryakov on 06/10/2020.
//

#include <esp_log.h>
#include "ScreenBase.h"

ScreenBase::ScreenBase(SemaphoreHandle_t guiSemaphore) : guiSemaphore(guiSemaphore) {
    assert(guiSemaphore != nullptr);
}

lv_obj_t *ScreenBase::getScreen() {
    return screen;
}

void ScreenBase::beginUpdate() {
    if (xSemaphoreTake(guiSemaphore, portMAX_DELAY) == pdFALSE) {
        abort();
    }
}

void ScreenBase::endUpdate() {
    xSemaphoreGive(guiSemaphore);
}
