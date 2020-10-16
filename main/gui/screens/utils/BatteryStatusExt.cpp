//
// Created by Maxim Dobryakov on 09/10/2020.
//

#include "BatteryStatusExt.h"

#include <cstdlib>
#include "../assets/IconsFont.h"

char const* batteryStatusSymbol(BatteryStatus status) {
    switch (status) {
        case BatteryStatus::Level0:
            return ICO_SYMBOL_BATTERY_LEVEL_0;
        case BatteryStatus::Level25:
            return ICO_SYMBOL_BATTERY_LEVEL_25;
        case BatteryStatus::Level50:
            return ICO_SYMBOL_BATTERY_LEVEL_50;
        case BatteryStatus::Level75:
            return ICO_SYMBOL_BATTERY_LEVEL_75;
        case BatteryStatus::Level100:
            return ICO_SYMBOL_BATTERY_LEVEL_100;
        case BatteryStatus::Charging:
            return ICO_SYMBOL_BATTERY_CHARGING;
    }

    abort();
}