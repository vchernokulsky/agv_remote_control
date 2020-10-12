//
// Created by Maxim Dobryakov on 14/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_WIFIMANAGER_H
#define AGV_REMOTE_CONTROL_WIFIMANAGER_H


#include <atomic>
#include <functional>
#include <string>
#include <esp_supplicant/esp_wps.h>
#include <esp_event_base.h>

#include "WiFiStatus.h"

class WiFiManager {
private:
    const char *LOG_TAG = "WiFiManager";

    esp_wps_config_t wpsConfig = {};
public:
    std::atomic_bool isWiFiClientStarted = ATOMIC_VAR_INIT(false);
    std::atomic_bool isWiFiConnectionEstablished = ATOMIC_VAR_INIT(false);

    WiFiManager();

    void start_wifi_client();
    void stop_wifi_client();

    void start_wps();
    void stop_wps();

    void connect();
    void connect(const std::string &ssid, const std::string &password);
    void disconnect();

    std::function<void(WiFiStatus wiFiStatus, std::string reason)> onWiFiEvent = nullptr;

private:
    static void wiFiEventHandlerWrapper(void* eventHandlerArg,
                                 esp_event_base_t eventBase,
                                 int32_t eventId,
                                 void* eventData);

    static void gotIpEventHandlerWrapper(void* eventHandlerArg,
                                               esp_event_base_t eventBase,
                                               int32_t eventId,
                                               void* eventData);

    void wiFiEventHandler(esp_event_base_t eventBase, int32_t eventId, void *eventData);

    void gotIpEventHandler(esp_event_base_t eventBase, int32_t eventId, void *eventData);

    static esp_wps_config_t getWpsConfig();

    void fireWiFiEvent(WiFiStatus wiFiStatus, std::string reason) const;
};


#endif //AGV_REMOTE_CONTROL_WIFIMANAGER_H
