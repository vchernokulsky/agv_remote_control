//
// Created by Maxim Dobryakov on 09/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_BATTERYSTATUS_H
#define AGV_REMOTE_CONTROL_BATTERYSTATUS_H


enum class BatteryStatus {
    NotFound,
    Percent25,
    Percent50,
    Percent75,
    Percent100,
    ChargingPercent25,
    ChargingPercent50,
    ChargingPercent75,
    ChargingPercent100,
};

char const* batteryStatusSymbol(BatteryStatus status);

#endif //AGV_REMOTE_CONTROL_BATTERYSTATUS_H
