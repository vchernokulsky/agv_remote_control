//
// Created by Maxim Dobryakov on 06/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_LOGSCREEN_H
#define AGV_REMOTE_CONTROL_LOGSCREEN_H


#include "../ScreenBase.h"

class LogScreen : public ScreenBase {
private:
    lv_task_t *timerTaskHandle = nullptr;

    lv_obj_t *imgBackground = nullptr;

    lv_obj_t *lstLog = nullptr;

    static void timerTaskHandler(lv_task_t *task);
    void timerTask();
public:
    lv_obj_t *screen = nullptr;

    LogScreen() = default;
    virtual ~LogScreen() = default;

    void initializeGui() override;
    void deinitializeGui() override;

    void addLine(const char *line);
};


#endif //AGV_REMOTE_CONTROL_LOGSCREEN_H
