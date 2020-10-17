//
// Created by Maxim Dobryakov on 01/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_LVGLAPP_H
#define AGV_REMOTE_CONTROL_LVGLAPP_H

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_timer.h>
#include <lvgl/lvgl.h>
#include <lvgl_helpers.h>

#include "ScreenBase.h"

class LvGlApp {
private:
    static const char *LOG_TAG;
    static const uint32_t TickPeriod = 10; // ms

    ScreenBase *mainScreen = nullptr;

    TaskHandle_t eventLoopTaskHandle = nullptr;

    std::atomic_bool isRunning = ATOMIC_VAR_INIT(false);
    std::atomic_bool isCancelled = ATOMIC_VAR_INIT(false);

    lv_color_t buffer1[DISP_BUF_SIZE] = {};
    lv_color_t buffer2[DISP_BUF_SIZE] = {};
    lv_disp_buf_t displayBuffer = {};

    lv_disp_drv_t displayDriver = {};

    esp_timer_handle_t tickTimer = nullptr;

    SemaphoreHandle_t guiSemaphore = nullptr;

public:
    LvGlApp();
    virtual ~LvGlApp();

    void runEventLoop();
    void cancelEventLoop();

    SemaphoreHandle_t getGuiSemaphore();
    void setMainScreen(ScreenBase *mainScreen);

private:
    void eventLoopTask();

    void initializeDriver();

    void startTickTimer();
    void stopTickTimer();

    static void eventLoopTaskHandler(void *arg);
    static void tickTaskHandler(void* arg);

    static void driverMonitorCallback(_disp_drv_t *displayDriver, uint32_t flushTime, uint32_t updatedPixels);
};


#endif //AGV_REMOTE_CONTROL_LVGLAPP_H
