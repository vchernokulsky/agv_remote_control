//
// Created by Maxim Dobryakov on 06/10/2020.
//

#include "LogScreen.h"


#include <esp_log.h>
#include "assets/Background.h"
#include "assets/Montserrat12Font.h"

LogScreen::LogScreen(SemaphoreHandle_t guiSemaphore) :
    ScreenBase(guiSemaphore) {
}

void LogScreen::initializeGui() {
    screen = lv_obj_create(nullptr, nullptr);

    imgBackground = lv_img_create(screen, nullptr);
    lv_img_set_src(imgBackground, &Background);

    lstLog = lv_list_create(screen, nullptr);
    lv_obj_set_size(lstLog, 310, 231);
    lv_obj_set_style_local_text_font(lstLog, LV_LIST_PART_BG, LV_STATE_DEFAULT, &Montserrat12Font);
    lv_obj_align(lstLog, nullptr, LV_ALIGN_IN_BOTTOM_MID, 0, -5);
}

void LogScreen::deinitializeGui() {
    lv_obj_del(imgBackground);

    lv_obj_del(lstLog);
}

void LogScreen::addLine(const std::string &line) {
    beginUpdate();
    uint16_t listSize = lv_list_get_size(lstLog);
    if (listSize == 7) {
        lv_list_remove(lstLog, 0);
    }

    lv_obj_t *button = lv_list_add_btn(lstLog, nullptr, line.c_str());
    lv_obj_t *label = lv_list_get_btn_label(button);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    endUpdate();
}
