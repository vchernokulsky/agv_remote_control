//
// Created by Maxim Dobryakov on 14/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_BUTTONSMANAGER_H
#define AGV_REMOTE_CONTROL_BUTTONSMANAGER_H

#include <iot_button.h>
#include <functional>

class ButtonsManager {
private:
    const gpio_num_t MENU_BUTTON_GPIO = GPIO_NUM_35;

    CButton *btnMenu = nullptr;

public:
    ButtonsManager();
    virtual ~ButtonsManager();

    std::function<void()> onMenuButton;

private:
    static void menuButtonHandler(void *arg);

};


#endif //AGV_REMOTE_CONTROL_BUTTONSMANAGER_H
