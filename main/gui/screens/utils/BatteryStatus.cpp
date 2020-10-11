//
// Created by Maxim Dobryakov on 09/10/2020.
//

#include "BatteryStatus.h"

#include <cstdlib>
#include "../assets/MaterialDesignIconsFont.h"

char const* batteryStatusSymbol(BatteryStatus status) {
    switch (status) {
        case BatteryStatus::NotFound:
            return MDI_SYMBOL_BATTERY_NOT_FOUND;
        case BatteryStatus::Percent25:
            return MDI_SYMBOL_BATTERY_0;
        case BatteryStatus::Percent50:
            return MDI_SYMBOL_BATTERY_1;
        case BatteryStatus::Percent75:
            return MDI_SYMBOL_BATTERY_2;
        case BatteryStatus::Percent100:
            return MDI_SYMBOL_BATTERY_3;
        case BatteryStatus::ChargingPercent25:
            return MDI_SYMBOL_BATTERY_CHARGING_0;
        case BatteryStatus::ChargingPercent50:
            return MDI_SYMBOL_BATTERY_CHARGING_1;
        case BatteryStatus::ChargingPercent75:
            return MDI_SYMBOL_BATTERY_CHARGING_2;
        case BatteryStatus::ChargingPercent100:
            return MDI_SYMBOL_BATTERY_CHARGING_3;
    }

    abort();
}