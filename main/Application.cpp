//
// Created by Maxim Dobryakov on 11/10/2020.
//

#include "Application.h"


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_netif.h>

using namespace std::placeholders;

void Application::start() {
    lvGlApp = new LvGlApp();

    mainScreen = new MainScreen(lvGlApp->getGuiSemaphore());

    lvGlApp->setMainScreen(mainScreen);
    lvGlApp->runEventLoop();

    buttonsManager = new ButtonsManager();
    buttonsManager->onMenuButton = std::bind(&Application::menuButtonCallback, this);

    wiFiManager = new WiFiManager();
    wiFiManager->onWiFiEvent = std::bind(&Application::wiFiEventCallback, this, _1, _2);

    ESP_LOGI("Main", "Wi-Fi Client Starting...");
    wiFiManager->startWiFiClient();
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
    rosClient->onConnect = [this](const std::string& platformName) { connectToRosCallback(platformName); };
    rosClient->onDisconnect = [this]() { disconnectFromRosCallback(); };
    rosClient->onPositionMessage = [this](const nav_msgs::Odometry &message) { positionMessageCallback(message); };
    rosClient->onPlatformStatusMessage = [this](const PlatformStatus platformStatus) { platformStatusMessageCallback(platformStatus); };
    rosClient->onLogMessage = [this](const rosserial_msgs::Log &message) { logMessageCallback(message); };
    rosClient->connect();

    navigationManager = new NavigationManager(rosClient);
    navigationManager->start();

    batteryManager = new BatteryManager();
    batteryManager->onBatteryStatusChanged = std::bind(&Application::batteryStatusChangedCallback, this, _1);
    batteryManager->start();
}

void Application::stop() {
    batteryManager->stop();
    delete batteryManager;
    batteryManager = nullptr;

    navigationManager->stop(true);
    delete navigationManager;
    navigationManager = nullptr;

    rosClient->disconnect();
    delete rosClient;
    rosClient = nullptr;

    wiFiManager->disconnect();
    wiFiManager->stopWiFiClient();
    delete wiFiManager;
    wiFiManager = nullptr;

    delete buttonsManager;
    buttonsManager = nullptr;

    lvGlApp->cancelEventLoop(); //TODO: wait
    delete lvGlApp;
    lvGlApp = nullptr;

    delete mainScreen;
    mainScreen = nullptr;
}

void Application::wiFiEventCallback(WiFiStatus wiFiStatus, const std::string& reason) {
    auto *indicatorScreen = mainScreen->getIndicatorScreen();

    auto *viewModel = indicatorScreen->viewModel;
    viewModel->takeLock();
    viewModel->wiFiStatus = wiFiStatus;
    viewModel->giveLock();

    if (wiFiStatus == WiFiStatus::ConnectionFailed) {
        auto *logScreen = mainScreen->getLogScreen();
        logScreen->addLine(std::string("Wi-Fi: ") + reason, logLevelColor(rosserial_msgs::Log::FATAL));
    }
}

void Application::connectToRosCallback(const std::string& platformName) {
    auto *indicatorScreen = mainScreen->getIndicatorScreen();

    auto *viewModel = indicatorScreen->viewModel;
    viewModel->takeLock();
    viewModel->platformName = platformName.empty() ? IndicatorsViewModel::PLATFORM_NOT_CONNECTED : platformName;
    viewModel->giveLock();
}

void Application::disconnectFromRosCallback() {
    auto *indicatorScreen = mainScreen->getIndicatorScreen();

    auto *viewModel = indicatorScreen->viewModel;
    viewModel->takeLock();
    viewModel->platformName = IndicatorsViewModel::PLATFORM_NOT_CONNECTED;
    viewModel->giveLock();
}

void Application::positionMessageCallback(const nav_msgs::Odometry &odometry) {
    auto *indicatorScreen = mainScreen->getIndicatorScreen();

    auto *viewModel = indicatorScreen->viewModel;
    viewModel->takeLock();
    viewModel->linearSpeed = odometry.twist.twist.linear.x;
    viewModel->angularSpeed = -odometry.twist.twist.angular.z; //FYI: invert value because turtlesim return invalid sign
    viewModel->xPosition = odometry.pose.pose.position.x;
    viewModel->yPosition = odometry.pose.pose.position.y;
    viewModel->giveLock();
}

void Application::platformStatusMessageCallback(const PlatformStatus platformStatus) {
    auto *indicatorScreen = mainScreen->getIndicatorScreen();

    auto *viewModel = indicatorScreen->viewModel;
    viewModel->takeLock();
    viewModel->platformStatus = platformStatus;
    viewModel->giveLock();
}

void Application::logMessageCallback(const rosserial_msgs::Log &message) {
    auto *logScreen = mainScreen->getLogScreen();
    logScreen->addLine(message.msg, logLevelColor(message.level));
}

void Application::menuButtonCallback() {
    mainScreen->toggleScreen();
}

void Application::batteryStatusChangedCallback(const BatteryStatus batteryStatus) {
    auto *indicatorScreen = mainScreen->getIndicatorScreen();

    auto *viewModel = indicatorScreen->viewModel;
    viewModel->takeLock();
    viewModel->batteryStatus = batteryStatus;
    viewModel->giveLock();
}

lv_color_t Application::logLevelColor(uint8_t level) {
    switch (level) {
        case rosserial_msgs::Log::INFO:
            return lv_color_make(211, 215, 207);
        case rosserial_msgs::Log::WARN:
            return lv_color_make(252, 233, 79);
        case rosserial_msgs::Log::ERROR:
        case rosserial_msgs::Log::FATAL:
            return lv_color_make(239, 41, 41);
        default:
            return lv_color_make(0, 0, 0);
    }
}
