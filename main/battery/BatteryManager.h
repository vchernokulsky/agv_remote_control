//
// Created by Maxim Dobryakov on 16/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_BATTERYMANAGER_H
#define AGV_REMOTE_CONTROL_BATTERYMANAGER_H


#include <functional>
#include "BatteryStatus.h"

class BatteryManager {
private:
    static const char *LOG_TAG;

public:
    void start();
    void stop();

    std::function<void(const BatteryStatus batteryStatus)> onBatteryStatusChanged;

private:
    void fireBatteryStatusChanged(BatteryStatus batteryStatus) const;

};


#endif //AGV_REMOTE_CONTROL_BATTERYMANAGER_H
