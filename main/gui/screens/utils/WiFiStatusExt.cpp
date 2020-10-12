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
            return "N";
        case WiFiStatus::Connecting:
            return "C";
        case WiFiStatus::ConnectionEstablished:
            return "+";
        case WiFiStatus::ConnectionFailed:
            return "!";
    }

    abort();
}