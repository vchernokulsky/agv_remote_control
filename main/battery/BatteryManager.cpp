//
// Created by Maxim Dobryakov on 16/10/2020.
//

#include "BatteryManager.h"

const char *BatteryManager::LOG_TAG = "BatteryManager";

void BatteryManager::start() {
    //TODO: not implemented yet
}

void BatteryManager::stop() {
    //TODO: not implemented yet
}

void BatteryManager::fireBatteryStatusChanged(const BatteryStatus batteryStatus) const {
    if (onBatteryStatusChanged)
        onBatteryStatusChanged(batteryStatus);
}
