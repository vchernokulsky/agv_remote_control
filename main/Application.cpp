//
// Created by Maxim Dobryakov on 11/10/2020.
//

#include "Application.h"


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_netif.h>

#include <utility>

void Application::start() {
    lvGlApp = new LvGlApp();

    mainScreen = new MainScreen(lvGlApp->getGuiSemaphore());

    lvGlApp->setMainScreen(mainScreen);
    lvGlApp->runEventLoop();

    wiFiManager = new WiFiManager();
    wiFiManager->onWiFiEvent = [this](WiFiStatus wiFiStatus, std::string reason) { wiFiEventCallback(wiFiStatus, std::move(reason)); };

    ESP_LOGI("Main", "Wi-Fi Client Starting...");
    wiFiManager->start_wifi_client();
    while(!wiFiManager->isWiFiClientStarted)
        vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI("Main", "Wi-Fi Client Started...");

#ifdef CONFIG_AGV_RC_WIFI_PASSWORD_AUTHENTICATION
    ESP_LOGI("Main", "Direct Wi-Fi Connection Starting...");
    wiFiManager->connect(CONFIG_AGV_RC_WIFI_SSID, CONFIG_AGV_RC_WIFI_PASSWORD);
    while(!wiFiManager->isWiFiConnectionEstablished)
        vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI("Main", "Direct Wi-Fi Connection Finished...");
#elif CONFIG_AGV_RC_WIFI_WPS_AUTHENTICATION
    ESP_LOGI("Main", "Wi-Fi WPS Connection Starting...");
    wiFiManager->start_wps();
    while(!wiFiManager->isWiFiConnectionEstablished)
        vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI("Main", "Wi-Fi WPS Connection Finished...");
#endif

    std::string rosManagerHost;
    ip4_addr_t rosManagerIp4 = {};
    uint16_t rosManagerPort = 0;

    auto *mDnsManager = new MDnsManager();
    while(!mDnsManager->lookupRosMaster(rosManagerHost, rosManagerIp4, rosManagerPort)) {
        ESP_LOGI("Main", "mDNS Lookup failed!");

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    delete mDnsManager;

    ESP_LOGI("Main", "ROS Master: " IPSTR ":%d", IP2STR(&rosManagerIp4), rosManagerPort);

    rosClient = new RosClient(rosManagerIp4.addr, rosManagerPort);
    rosClient->connect();

    navigationManager = new NavigationManager(rosClient);
    navigationManager->start();
}

void Application::stop() {
    navigationManager->stop(true);
    delete navigationManager;
    navigationManager = nullptr;

    rosClient->disconnect();
    delete rosClient;
    rosClient = nullptr;

    wiFiManager->disconnect();
    wiFiManager->stop_wifi_client();
    delete wiFiManager;
    wiFiManager = nullptr;

    lvGlApp->cancelEventLoop(); //TODO: wait
    delete lvGlApp;
    lvGlApp = nullptr;

    delete mainScreen;
    mainScreen = nullptr;
}

void Application::wiFiEventCallback(WiFiStatus wiFiStatus, std::string reason) {
    auto *indicatorScreen = mainScreen->getIndicatorScreen();

    indicatorScreen->updateWiFiStatus(wiFiStatus);
    //TODO: write reason to log
}