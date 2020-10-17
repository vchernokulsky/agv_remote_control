//
// Created by Maxim Dobryakov on 01/10/2020.
//

#include "LvGlApp.h"


#include <esp_log.h>

const char *LvGlApp::LOG_TAG = "LvGlApp";

LvGlApp::LvGlApp() {
    guiSemaphore = xSemaphoreCreateMutex();
}

LvGlApp::~LvGlApp() {
    vSemaphoreDelete(guiSemaphore);
    guiSemaphore = nullptr;
}

void LvGlApp::runEventLoop() {
    BaseType_t taskResult = xTaskCreatePinnedToCore(eventLoopTaskHandler, "gui-event-loop", 8 * 4096, this, 0, &eventLoopTaskHandle, 1);
    if (taskResult != pdPASS) {
        ESP_LOGW(LOG_TAG, "Can't create `gui-event-loop` task. Error: %d", taskResult);
        abort();
    }

    while (!isRunning)
        vTaskDelay(pdMS_TO_TICKS(10));
}

void LvGlApp::cancelEventLoop() {
    isCancelled = true;
}

void LvGlApp::eventLoopTask() {
    lv_init();

    initializeDriver();

    mainScreen->initializeGui();

    startTickTimer();
    isRunning = true;
    while(!isCancelled) {
        vTaskDelay(pdMS_TO_TICKS(10));

        if (xSemaphoreTake(guiSemaphore, portMAX_DELAY) == pdTRUE) {
            lv_task_handler();
            xSemaphoreGive(guiSemaphore);
        }
    }
    isRunning = false;
    stopTickTimer();

    mainScreen->deinitializeGui();

    lv_deinit();

    vTaskDelete(eventLoopTaskHandle);
    eventLoopTaskHandle = nullptr;
}

void LvGlApp::initializeDriver() {
    lvgl_driver_init();

    lv_disp_buf_init(&displayBuffer, buffer1, buffer2, DISP_BUF_SIZE);

    lv_disp_drv_init(&displayDriver);
    displayDriver.flush_cb = disp_driver_flush;
    displayDriver.monitor_cb = &driverMonitorCallback;
    displayDriver.buffer = &displayBuffer;
    lv_disp_drv_register(&displayDriver);
}

void LvGlApp::startTickTimer() {
    const esp_timer_create_args_t tickTimerArgs = {
            .callback = &tickTaskHandler,
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "gui-tick-timer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&tickTimerArgs, &tickTimer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tickTimer, TickPeriod * 1000));
}

void LvGlApp::stopTickTimer() {
    ESP_ERROR_CHECK(esp_timer_stop(tickTimer));
}

void LvGlApp::eventLoopTaskHandler(void *arg) {
    auto *instance = static_cast<LvGlApp *>(arg);
    instance->eventLoopTask();
}

void LvGlApp::tickTaskHandler(__unused void *arg) {
    lv_tick_inc(TickPeriod);
}

SemaphoreHandle_t LvGlApp::getGuiSemaphore() {
    return guiSemaphore;
}

void LvGlApp::setMainScreen(ScreenBase *mainScreen) {
    this->mainScreen = mainScreen;
}

void LvGlApp::driverMonitorCallback(__unused _disp_drv_t *displayDriver, __unused uint32_t flushTime, uint32_t updatedPixels) {
    ESP_LOGI(LOG_TAG, "Flush time: %6d ms; Updated pixels: %6d", flushTime, updatedPixels);
}
