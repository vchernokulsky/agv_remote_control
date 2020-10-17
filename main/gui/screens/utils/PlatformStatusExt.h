//
// Created by Maxim Dobryakov on 09/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_PLATFORMSTATUSEXT_H
#define AGV_REMOTE_CONTROL_PLATFORMSTATUSEXT_H

#include <ros/PlatformStatus.h>
#include <lvgl/lvgl.h>

const char *platformStatusText(PlatformStatus status);
lv_color_t platformStatusColor(PlatformStatus status);

#endif //AGV_REMOTE_CONTROL_PLATFORMSTATUSEXT_H
