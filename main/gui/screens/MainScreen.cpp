//
// Created by Maxim Dobryakov on 06/10/2020.
//

#include <esp_log.h>
#include "MainScreen.h"

#include "assets/Montserrat12Font.h"
#include "assets/Montserrat14Font.h"
#include "assets/Montserrat16Font.h"

MainScreen::MainScreen() {
    indicatorScreen = new IndicatorsScreen();
    logScreen = new LogScreen();
}

MainScreen::~MainScreen() {
    delete indicatorScreen;
    delete logScreen;
}

void MainScreen::initializeGui() {
    initializeRussianFonts();

    indicatorScreen->initializeGui();
    logScreen->initializeGui();

    activeScreen = indicatorScreen->screen;
//    activeScreen = logScreen->screen;

    lv_scr_load(activeScreen);

    timerTaskHandle = lv_task_create(timerTaskHandler, 9000, LV_TASK_PRIO_MID, this);
}

void MainScreen::deinitializeGui() {
    indicatorScreen->deinitializeGui();
    logScreen->deinitializeGui();
}

void MainScreen::initializeRussianFonts() {
    auto *theme = lv_theme_get_act();

    theme->font_small = &Montserrat12Font;
    theme->font_normal = &Montserrat14Font;
    theme->font_subtitle = &Montserrat14Font;
    theme->font_title = &Montserrat16Font;
}

void MainScreen::timerTaskHandler(lv_task_t *task) {
    auto *instance = static_cast<MainScreen *>(task->user_data);
    instance->timerTask();
}

void MainScreen::timerTask() {
    if (activeScreen == indicatorScreen->screen) {
        activeScreen = logScreen->screen;
    } else {
        activeScreen = indicatorScreen->screen;
    }

    lv_scr_load(activeScreen);
}
