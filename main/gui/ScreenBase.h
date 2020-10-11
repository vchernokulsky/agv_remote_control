//
// Created by Maxim Dobryakov on 06/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_SCREENBASE_H
#define AGV_REMOTE_CONTROL_SCREENBASE_H

#include <lvgl/lvgl.h>

class ScreenBase {
public:
    virtual void initializeGui() = 0;
    virtual void deinitializeGui() = 0;
};


#endif //AGV_REMOTE_CONTROL_SCREENBASE_H
