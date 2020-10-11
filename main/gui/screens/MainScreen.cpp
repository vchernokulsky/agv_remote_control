//
// Created by Maxim Dobryakov on 06/10/2020.
//

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
