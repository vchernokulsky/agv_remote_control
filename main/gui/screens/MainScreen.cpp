//
// Created by Maxim Dobryakov on 06/10/2020.
//

#include "MainScreen.h"

#include "assets/Montserrat12Font.h"
#include "assets/Montserrat14Font.h"
#include "assets/Montserrat16Font.h"

MainScreen::MainScreen(SemaphoreHandle_t guiSemaphore) : ScreenBase(guiSemaphore) {
    indicatorScreen = new IndicatorsScreen(guiSemaphore);
    logScreen = new LogScreen(guiSemaphore);
}

MainScreen::~MainScreen() {
    delete indicatorScreen;
    delete logScreen;
}

void MainScreen::initializeGui() {
    initializeRussianFonts();

    indicatorScreen->initializeGui();
    logScreen->initializeGui();

    activeScreen = indicatorScreen->getScreen();
//    activeScreen = logScreen->screen;

    lv_scr_load(activeScreen);
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

LogScreen *MainScreen::getLogScreen() {
    return logScreen;
}

IndicatorsScreen *MainScreen::getIndicatorScreen() {
    return indicatorScreen;
}

