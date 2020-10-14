//
// Created by Maxim Dobryakov on 09/10/2020.
//

#include "WiFiStatusExt.h"


#include <cstdlib>
#include "../assets/MaterialDesignIconsFont.h"

//TODO: fix icons
char const* wiFiStatusSymbol(WiFiStatus status) {
    switch (status) {
        case WiFiStatus::NotConnected:
            return "-";
        case WiFiStatus::Connecting:
            return "~";
        case WiFiStatus::ConnectionEstablished:
            return "+";
        case WiFiStatus::ConnectionStrange_0:
            return "0";
        case WiFiStatus::ConnectionStrange_25:
            return "1";
        case WiFiStatus::ConnectionStrange_50:
            return "2";
        case WiFiStatus::ConnectionStrange_75:
            return "3";
        case WiFiStatus::ConnectionStrange_100:
            return "4";
        case WiFiStatus::ConnectionFailed:
            return "!";
    }

    abort();
}