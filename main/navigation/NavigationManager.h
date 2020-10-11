//
// Created by Maxim Dobryakov on 18/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_NAVIGATIONMANAGER_H
#define AGV_REMOTE_CONTROL_NAVIGATIONMANAGER_H

#include "JoystickController.h"

#include <cstdint>
#include <cmath>
#include <string>
#include "utils/TaskBase.h"
#include "ros/RosClient.h"

class NavigationManager: public TaskBase {
private:
    const char *LOG_TAG = "NavigationManager";

    const uint32_t MEASUREMENT_INTERVAL = 10; // ms

    RosClient *rosClient;

    JoystickController joystickController;

    uint32_t xCenter = 0;
    uint32_t yCenter = 0;

public:
    explicit NavigationManager(RosClient *rosClient);
    virtual ~NavigationManager() = default;

    void start();
    void stop(bool waitCancellation = false);

private:
    bool navigationLoop();

    bool convertJoystickPosition(uint32_t xValue, uint32_t yValue, double &linearSpeed, double &angularSpeed);

    void task() override;
};


#endif //AGV_REMOTE_CONTROL_NAVIGATIONMANAGER_H
