//
// Created by Maxim Dobryakov on 06/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_SCREENBASE_H
#define AGV_REMOTE_CONTROL_SCREENBASE_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <lvgl/lvgl.h>

class ScreenBase {
private:
    SemaphoreHandle_t guiSemaphore = nullptr;

public:
    explicit ScreenBase(SemaphoreHandle_t guiSemaphore);

    virtual void initializeGui() = 0;
    virtual void deinitializeGui() = 0;

    lv_obj_t *getScreen();

protected:
    lv_obj_t *screen = nullptr;

    void beginUpdate();
    void endUpdate();
};


#endif //AGV_REMOTE_CONTROL_SCREENBASE_H
