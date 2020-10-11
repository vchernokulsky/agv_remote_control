//
// Created by Maxim Dobryakov on 09/10/2020.
//

#include "WiFiStatus.h"


#include <cstdlib>
#include "../assets/MaterialDesignIconsFont.h"

char const* wiFiStatusSymbol(WiFiStatus status) {
    switch (status) {
        case WiFiStatus::NotConnected:
            return MDI_SYMBOL_WIFI_NOT_FOUND;
        case WiFiStatus::Connection:
            return MDI_SYMBOL_WIFI_ALERT;
        case WiFiStatus::Strange0:
            return MDI_SYMBOL_WIFI_0;
        case WiFiStatus::Strange1:
            return MDI_SYMBOL_WIFI_1;
        case WiFiStatus::Strange2:
            return MDI_SYMBOL_WIFI_2;
        case WiFiStatus::Strange3:
            return MDI_SYMBOL_WIFI_3;
        case WiFiStatus::Strange4:
            return MDI_SYMBOL_WIFI_4;
    }

    abort();
}