//
// Created by Maxim Dobryakov on 13/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_TASKBASE_H
#define AGV_REMOTE_CONTROL_TASKBASE_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class TaskBase {
protected:
    TaskHandle_t taskHandle = nullptr;

public:
    void runTask(const char *taskName, BaseType_t taskPriority, uint32_t stackDepth = configMINIMAL_STACK_SIZE);

private:
    static void taskHandler(void* parm);

protected:
    virtual void task() = 0;
};


#endif //AGV_REMOTE_CONTROL_TASKBASE_H
