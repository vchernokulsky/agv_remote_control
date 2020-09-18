//
// Created by Maxim Dobryakov on 14/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_WIFIMANAGER_H
#define AGV_REMOTE_CONTROL_WIFIMANAGER_H


#include <esp_supplicant/esp_wps.h>
#include <esp_event_base.h>
#include <atomic>

class WiFiManager {
private:
    const char *LogTag = "WiFiManager";

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
    void connect(const char *ssid, const char *password);
    void disconnect();

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
};


#endif //AGV_REMOTE_CONTROL_WIFIMANAGER_H