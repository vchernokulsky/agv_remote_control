//
// Created by Maxim Dobryakov on 14/10/2020.
//

#include "ButtonsManager.h"

ButtonsManager::ButtonsManager() {
    btnMenu = new CButton(MENU_BUTTON_GPIO, BUTTON_ACTIVE_HIGH);
    btnMenu->set_evt_cb(BUTTON_CB_RELEASE, menuButtonHandler, this);
}

ButtonsManager::~ButtonsManager() {
    delete btnMenu;
}

void ButtonsManager::menuButtonHandler(void *arg) {
    auto *buttonsManager = static_cast<ButtonsManager *>(arg);
    if (buttonsManager->onMenuButton)
        buttonsManager->onMenuButton();
}
