//
// Created by Maxim Dobryakov on 12/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_WIFISTATUS_H
#define AGV_REMOTE_CONTROL_WIFISTATUS_H

enum class WiFiStatus {
    NotConnected,
    Connecting,
    ConnectionEstablished,
    ConnectionFailed,
    ConnectionQuality_25,
    ConnectionQuality_50,
    ConnectionQuality_75,
    ConnectionQuality_100,
};

#endif //AGV_REMOTE_CONTROL_WIFISTATUS_H
