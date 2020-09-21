//
// Created by Maxim Dobryakov on 13/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_TASKBASE_H
#define AGV_REMOTE_CONTROL_TASKBASE_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>
#include <string>

class TaskBase {
protected:
    std::atomic_bool isCancelled = ATOMIC_VAR_INIT(false);

    std::string taskName;
    TaskHandle_t taskHandle = nullptr;

public:
    void runTask(const std::string &taskName, BaseType_t taskPriority, uint32_t stackDepth = configMINIMAL_STACK_SIZE);
    void cancelTask(bool waitCancellation = false);

private:
    static void taskHandler(void* parm);

protected:
    virtual void task() = 0;
};


#endif //AGV_REMOTE_CONTROL_TASKBASE_H
