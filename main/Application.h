//
// Created by Maxim Dobryakov on 11/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_APPLICATION_H
#define AGV_REMOTE_CONTROL_APPLICATION_H


#include <gui/screens/MainScreen.h>
#include <gui/LvGlApp.h>
#include <network/WiFiManager.h>
#include <network/MDnsManager.h>
#include <ros/RosClient.h>
#include <navigation/NavigationManager.h>

class Application {
private:
    MainScreen *mainScreen = nullptr;
    LvGlApp *lvGlApp = nullptr;

    WiFiManager *wiFiManager = nullptr;

    RosClient *rosClient = nullptr;

    NavigationManager *navigationManager = nullptr;

public:
    void start();
    void stop();

private:
    void wiFiEventCallback(WiFiStatus wiFiStatus, const std::string& reason);
    void connectToRosCallback(const std::string& platformName);
    void disconnectFromRosCallback();
};


#endif //AGV_REMOTE_CONTROL_APPLICATION_H
