#include <climits>
//
// Created by Maxim Dobryakov on 14/09/2020.
//

#include "WiFiManager.h"

#include <cstring>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_wifi_default.h>
#include <esp_wifi.h>
#include <esp_supplicant/esp_wps.h>

WiFiManager::WiFiManager() {
    ESP_LOGV(LOG_TAG, "Initialization of Wi-Fi system");

    ESP_ERROR_CHECK(esp_netif_init());
}

void WiFiManager::start_wifi_client() {
    assert(!isWiFiClientStarted);

    ESP_LOGV(LOG_TAG, "Start STA Wi-Fi client");

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    assert(esp_netif_create_default_wifi_sta());

    wifi_init_config_t wiFiInitConfig = WIFI_INIT_CONFIG_DEFAULT()
    wiFiInitConfig.nvs_enable = 0; //TODO: disable NVS for simplify debug
    ESP_ERROR_CHECK(esp_wifi_init(&wiFiInitConfig));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wiFiEventHandlerWrapper, this));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &gotIpEventHandlerWrapper, this));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void WiFiManager::stop_wifi_client() {
    assert(isWiFiClientStarted);

    ESP_LOGV(LOG_TAG, "Stop STA Wi-Fi client");

    ESP_ERROR_CHECK(esp_wifi_stop());

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &gotIpEventHandlerWrapper));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wiFiEventHandlerWrapper));

    ESP_ERROR_CHECK(esp_wifi_deinit());

    ESP_ERROR_CHECK(esp_event_loop_delete_default());
}

void WiFiManager::start_wps() {
    assert(isWiFiClientStarted);

    ESP_LOGV(LOG_TAG, "Start Wi-Fi WPS");

    wpsConfig = getWpsConfig();
    ESP_ERROR_CHECK(esp_wifi_wps_enable(&wpsConfig));
    ESP_ERROR_CHECK(esp_wifi_wps_start(0));
}

void WiFiManager::stop_wps() {
    assert(isWiFiClientStarted);

    ESP_LOGV(LOG_TAG, "Stop Wi-Fi WPS");

    ESP_ERROR_CHECK(esp_wifi_wps_disable());
}

void WiFiManager::connect() {
    assert(isWiFiClientStarted);
    assert(!isWiFiConnectionEstablished);

    ESP_LOGV(LOG_TAG, "Connect to Wi-Fi (use automatically saved to flash memory credentials)");

    ESP_ERROR_CHECK(esp_wifi_connect());
}

void WiFiManager::connect(const std::string &ssid, const std::string &password) {
    assert(isWiFiClientStarted);
    assert(!isWiFiConnectionEstablished);

    ESP_LOGV(LOG_TAG, "Connect to Wi-Fi with SSID and password");

    wifi_config_t wifiConfig = {};
    strlcpy((char *)wifiConfig.sta.ssid, ssid.c_str(), sizeof(wifiConfig.sta.ssid));
    strlcpy((char *)wifiConfig.sta.password, password.c_str(), sizeof(wifiConfig.sta.password));
    wifiConfig.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifiConfig.sta.pmf_cfg.capable = true;
    wifiConfig.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifiConfig));
    ESP_ERROR_CHECK(esp_wifi_connect());
}

void WiFiManager::disconnect() {
    assert(isWiFiClientStarted);
    assert(isWiFiConnectionEstablished);

    ESP_LOGV(LOG_TAG, "Disconnect from Wi-Fi");

    ESP_ERROR_CHECK(esp_wifi_disconnect());
}

void WiFiManager::wiFiEventHandlerWrapper(void* eventHandlerArg,
                                          esp_event_base_t eventBase,
                                          int32_t eventId,
                                          void* eventData) {
    static_cast<WiFiManager *>(eventHandlerArg)->wiFiEventHandler(eventBase, eventId, eventData);
}

void WiFiManager::gotIpEventHandlerWrapper(void* eventHandlerArg,
                                           esp_event_base_t eventBase,
                                           int32_t eventId,
                                           void* eventData) {
    static_cast<WiFiManager *>(eventHandlerArg)->gotIpEventHandler(eventBase, eventId, eventData);
}

void WiFiManager::wiFiEventHandler(__unused esp_event_base_t eventBase,
                                   int32_t eventId,
                                   __unused void* eventData) {
    switch (eventId) {
        case WIFI_EVENT_STA_START:
            ESP_LOGD(LOG_TAG, "STA start");
            isWiFiClientStarted = true;
            break;
        case WIFI_EVENT_STA_STOP:
            ESP_LOGD(LOG_TAG, "STA stop");
            isWiFiClientStarted = false;
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGD(LOG_TAG, "Connected to AP");
            isWiFiConnectionEstablished = true;
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            {
                auto *event = (wifi_event_sta_disconnected_t *)eventData;
                ESP_LOGW(LOG_TAG, "Disconnected from AP. Reason: %d", event->reason);
                isWiFiConnectionEstablished = false;
                break;
            }
        case WIFI_EVENT_STA_WPS_ER_SUCCESS:
            {
                ESP_LOGD(LOG_TAG, "WPS succeeds in enrollee mode");

                //TODO: ESP-IDF v4.1 doesn't support multiple WPS credentials (fix present in master but not included to release).
                //      Need to implement it later. See: https://github.com/espressif/esp-idf/commit/81f037a2999992fbe184b4b8871a4c670d8fd804

                /*
                 * If only one AP credential is received from WPS, there will be no event data and
                 * esp_wifi_set_config() is already called by WPS modules for backward compatibility
                 * with legacy apps. So directly attempt connection here.
                 */
                ESP_ERROR_CHECK(esp_wifi_wps_disable());
                ESP_ERROR_CHECK(esp_wifi_connect());
            }
            break;
        case WIFI_EVENT_STA_WPS_ER_FAILED:
            {
                ESP_LOGW(LOG_TAG, "WPS fails in enrollee mode");

                wpsConfig = getWpsConfig();

                ESP_ERROR_CHECK(esp_wifi_wps_disable());
                ESP_ERROR_CHECK(esp_wifi_wps_enable(&wpsConfig));
                ESP_ERROR_CHECK(esp_wifi_wps_start(0));
            }
            break;
        case WIFI_EVENT_STA_WPS_ER_TIMEOUT:
            {
                ESP_LOGW(LOG_TAG, "WPS timeout in enrollee mode");

                wpsConfig = getWpsConfig();

                ESP_ERROR_CHECK(esp_wifi_wps_disable());
                ESP_ERROR_CHECK(esp_wifi_wps_enable(&wpsConfig));
                ESP_ERROR_CHECK(esp_wifi_wps_start(0));
            }
            break;
        default:
            ESP_LOGV(LOG_TAG, "Unhandled event id: %d", eventId);
            break;
    }
}

//TODO: Looks like this handler and related code is unusable in our case and should be removed in the future.
void WiFiManager::gotIpEventHandler(__unused esp_event_base_t eventBase,
                                    __unused int32_t eventId,
                                    void* eventData) {
    auto* event = (ip_event_got_ip_t*) eventData;

    ESP_LOGV(LOG_TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
}

esp_wps_config_t WiFiManager::getWpsConfig() {
    esp_wps_config_t espWpsConfig = {};

    espWpsConfig.wps_type = WPS_TYPE_PBC;

    strlcpy(espWpsConfig.factory_info.manufacturer, CONFIG_WPS_MANUFACTURER, sizeof(espWpsConfig.factory_info.manufacturer));
    strlcpy(espWpsConfig.factory_info.model_number, CONFIG_WPS_MODEL_NUMBER, sizeof(espWpsConfig.factory_info.model_number));
    strlcpy(espWpsConfig.factory_info.model_name, CONFIG_WPS_MODEL_NAME, sizeof(espWpsConfig.factory_info.model_name));
    strlcpy(espWpsConfig.factory_info.device_name, CONFIG_WPS_DEVICE_NAME, sizeof(espWpsConfig.factory_info.device_name));

    return espWpsConfig;
}