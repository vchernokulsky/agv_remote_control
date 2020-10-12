//
// Created by Maxim Dobryakov on 06/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_MAINSCREEN_H
#define AGV_REMOTE_CONTROL_MAINSCREEN_H

#include "../ScreenBase.h"

#include "IndicatorsScreen.h"
#include "LogScreen.h"

class MainScreen : public ScreenBase {
private:
    IndicatorsScreen *indicatorScreen;
    LogScreen *logScreen;

    lv_obj_t *activeScreen = nullptr;

public:
    explicit MainScreen(SemaphoreHandle_t guiSemaphore);
    virtual ~MainScreen();

    IndicatorsScreen *getIndicatorScreen();
    LogScreen *getLogScreen();

protected:
    void initializeGui() override;
    void deinitializeGui() override;

    static void initializeRussianFonts() ;
};


#endif //AGV_REMOTE_CONTROL_MAINSCREEN_H
