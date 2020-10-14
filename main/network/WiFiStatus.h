//
// Created by Maxim Dobryakov on 12/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_WIFISTATUS_H
#define AGV_REMOTE_CONTROL_WIFISTATUS_H

enum class WiFiStatus {
    NotConnected,
    Connecting,
    ConnectionFailed,
    ConnectionEstablished,
    ConnectionStrange_0,
    ConnectionStrange_25,
    ConnectionStrange_50,
    ConnectionStrange_75,
    ConnectionStrange_100,
};

#endif //AGV_REMOTE_CONTROL_WIFISTATUS_H
