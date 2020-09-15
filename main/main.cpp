#include "JoystickEventsDataSource.h"
#include "JoystickEventsProcessor.h"
#include "JoystickEventsSink.h"
#include "WiFiManager.h"
#include "MDnsManager.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_wifi_types.h>
#include <esp_wifi.h>
#include <esp_log.h>

extern "C" void app_main();

void app_main() {
    vTaskDelay(pdMS_TO_TICKS(1000));

//    auto *pJoystickEventsDataSource = new JoystickEventsDataSource();
//    pJoystickEventsDataSource->runTask("JoystickEventsDataSource", 2, 4 * configMINIMAL_STACK_SIZE);
//
//    auto *pJoystickEventsProcessor = new JoystickEventsProcessor(
//            pJoystickEventsDataSource->joystickRawEventsQueueHandler,
//            JoystickEventsDataSource::getMaxPossibleValue());
//    pJoystickEventsProcessor->runTask("JoystickEventsProcessor", 2, 4 * configMINIMAL_STACK_SIZE);
//
//    auto *pJoystickEventsSink = new JoystickEventsSink(pJoystickEventsProcessor->joystickSpeedEventsQueueHandler);
//    pJoystickEventsSink->runTask("JoystickEventsSink", 3, 4 * configMINIMAL_STACK_SIZE);

    WiFiManager wiFiManager;

    ESP_LOGI("Main", "Wi-Fi Client Starting...");
    wiFiManager.start_wifi_client();
    while(!wiFiManager.isWiFiClientStarted)
        vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI("Main", "Wi-Fi Client Started...");

    ESP_LOGI("Main", "Wi-Fi WPS Connection Starting...");
    wiFiManager.start_wps();
    while(!wiFiManager.isWiFiConnectionEstablished)
        vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI("Main", "Wi-Fi WPS Connection Finished...");

    MDnsManager mDnsManager;

    std::string rosManagerHost;
    ip4_addr_t rosManagerIp4 = {};
    uint16_t rosManagerPort = 0;

    for(;;) {
        bool lookupResult = mDnsManager.lookupRosMaster(rosManagerHost, rosManagerIp4, rosManagerPort);

        if (lookupResult) {
            ESP_LOGI("", "%s:%d; IP4: " IPSTR, rosManagerHost.c_str(), rosManagerPort, IP2STR(&rosManagerIp4));
        } else {
            ESP_LOGI("", "Lookup failed!");
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
        ESP_LOGI("", "");
    }

//    ESP_LOGI("Main", "Working...");
//    vTaskDelay(pdMS_TO_TICKS(5000));
//    ESP_LOGI("Main", "Worked...");
//
//    ESP_LOGI("Main", "Stoping WPS...");
//    wiFiManager.stop_wps();
//    ESP_LOGI("Main", "Stoped WPS...");
//
//    ESP_LOGI("Main", "Wi-Fi Disconnection Starting...");
//    wiFiManager.disconnect();
//    while(wiFiManager.isWiFiConnectionEstablished)
//        vTaskDelay(pdMS_TO_TICKS(500));
//    ESP_LOGI("Main", "Wi-Fi Disconnection Finished...");

//    ESP_LOGI("Main", "Wi-Fi Connection Starting...");
//    wiFiManager.connect();
//    while(!wiFiManager.isWiFiConnectionEstablished)
//        vTaskDelay(pdMS_TO_TICKS(500));
//    ESP_LOGI("Main", "Wi-Fi Connection Finished...");

}
