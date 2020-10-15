//
// Created by Maxim Dobryakov on 09/10/2020.
//

#include "WiFiStatusExt.h"


#include <cstdlib>
#include "../assets/IconsFont.h"

char const* wiFiStatusSymbol(WiFiStatus status) {
    switch (status) {
        case WiFiStatus::NotConnected:
            return ICO_SYMBOL_WIFI_NOT_CONNECTED;
        case WiFiStatus::Connecting:
            return ICO_SYMBOL_WIFI_CONNECTING;
        case WiFiStatus::ConnectionEstablished:
            return ICO_SYMBOL_WIFI_CONNECTED;
        case WiFiStatus::ConnectionFailed:
            return ICO_SYMBOL_WIFI_FAILED;
        case WiFiStatus::ConnectionQuality_25:
            return ICO_SYMBOL_WIFI_QUALITY_25;
        case WiFiStatus::ConnectionQuality_50:
            return ICO_SYMBOL_WIFI_QUALITY_50;
        case WiFiStatus::ConnectionQuality_75:
            return ICO_SYMBOL_WIFI_QUALITY_75;
        case WiFiStatus::ConnectionQuality_100:
            return ICO_SYMBOL_WIFI_QUALITY_100;
    }

    abort();
}