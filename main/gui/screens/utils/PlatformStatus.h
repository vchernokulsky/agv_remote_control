//
// Created by Maxim Dobryakov on 09/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_PLATFORMSTATUS_H
#define AGV_REMOTE_CONTROL_PLATFORMSTATUS_H

#include <lvgl/lvgl.h>

enum class PlatformStatus {
    UNDEFINED,
    OK,
    WARNING,
    FAULT,
};

const char *platformStatusText(PlatformStatus status);
lv_color_t platformStatusColor(PlatformStatus status);

#endif //AGV_REMOTE_CONTROL_PLATFORMSTATUS_H
