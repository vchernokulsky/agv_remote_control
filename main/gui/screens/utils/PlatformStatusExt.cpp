//
// Created by Maxim Dobryakov on 09/10/2020.
//

#include "PlatformStatusExt.h"

#include <cstdlib>

const char *platformStatusText(PlatformStatus status) {
    switch (status) {
        case PlatformStatus::UNDEFINED:
            return "?";
        case PlatformStatus::OK:
            return "OK";
        case PlatformStatus::WARNING:
            return "WARN";
        case PlatformStatus::FAULT:
            return "FAULT";
    }

    abort();
}

lv_color_t platformStatusColor(PlatformStatus status) {
    switch (status) {
        case PlatformStatus::UNDEFINED:
            return lv_color_make(128, 128, 128);
        case PlatformStatus::OK:
            return lv_color_make(0, 128, 0);
        case PlatformStatus::WARNING:
            return lv_color_make(255, 165, 0);
        case PlatformStatus::FAULT:
            return lv_color_make(128, 0, 0);
    }

    abort();
}