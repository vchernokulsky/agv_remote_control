//
// Created by Maxim Dobryakov on 09/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_WIFISTATUS_H
#define AGV_REMOTE_CONTROL_WIFISTATUS_H


enum class WiFiStatus {
    NotConnected,
    Connection,
    Strange0,
    Strange1,
    Strange2,
    Strange3,
    Strange4,
};

char const* wiFiStatusSymbol(WiFiStatus status);

#endif //AGV_REMOTE_CONTROL_WIFISTATUS_H
