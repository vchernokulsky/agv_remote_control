#include "WiFiManager.h"
#include "MDnsManager.h"
#include "navigation/NavigationManager.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_wifi_types.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <ros.h>

extern "C" void app_main();

void app_main() {
    vTaskDelay(pdMS_TO_TICKS(1000));

    WiFiManager wiFiManager;

    ESP_LOGI("Main", "Wi-Fi Client Starting...");
    wiFiManager.start_wifi_client();
    while(!wiFiManager.isWiFiClientStarted)
        vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI("Main", "Wi-Fi Client Started...");

//    ESP_LOGI("Main", "Wi-Fi WPS Connection Starting...");
//    wiFiManager.start_wps();
//    while(!wiFiManager.isWiFiConnectionEstablished)
//        vTaskDelay(pdMS_TO_TICKS(500));
//    ESP_LOGI("Main", "Wi-Fi WPS Connection Finished...");

// WPS or Direct Connection

    ESP_LOGI("Main", "Direct Wi-Fi Connection Starting...");
    wiFiManager.connect("???", "???");
    while(!wiFiManager.isWiFiConnectionEstablished)
        vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI("Main", "Direct Wi-Fi Connection Finished...");


    MDnsManager mDnsManager;

    std::string rosManagerHost;
    ip4_addr_t rosManagerIp4 = {};
    uint16_t rosManagerPort = 0;

    while(!mDnsManager.lookupRosMaster(rosManagerHost, rosManagerIp4, rosManagerPort)) {
        ESP_LOGI("Main", "mDNS Lookup failed!");

        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    ESP_LOGI("Main", "ROS Master: " IPSTR ":%d", IP2STR(&rosManagerIp4), rosManagerPort);

    auto *navigationManager = new NavigationManager(rosManagerIp4.addr, rosManagerPort);
    navigationManager->runTask("JoystickManager", 2, 8 * configMINIMAL_STACK_SIZE);

//    ESP_LOGI("Main", "Delay...");
//    vTaskDelay(pdMS_TO_TICKS(20000));
//    ESP_LOGI("Main", "Delay completed...");
//
//    ESP_LOGI("Main", "Start cancellation...");
//    navigationManager->cancelTask(true);
//    ESP_LOGI("Main", "Finish cancellation...");
}
