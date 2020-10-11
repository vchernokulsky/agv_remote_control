//
// Created by Maxim Dobryakov on 06/10/2020.
//

#include "LogScreen.h"


#include <esp_log.h>
#include "assets/Background.h"
#include "assets/Montserrat12Font.h"

char const *lines[] {
    "Oct  6 16:47:36 md-dsk com.apple.xpc.launchd[1] (com.apple.quicklook[31122]): Endpoint has been activated through legacy launch(3) APIs. Please switch to XPC or bootstrap_check_in(): com.apple.quicklook",
    "Oct  6 16:52:35 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 17:07:04 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 17:07:15 md-dsk com.apple.xpc.launchd[1] (com.apple.imfoundation.IMRemoteURLConnectionAgent): Unknown key for integer: _DirtyJetsamMemoryLimit",
    "Oct  6 17:07:17 md-dsk com.apple.xpc.launchd[1] (com.apple.quicklook[31190]): Endpoint has been activated through legacy launch(3) APIs. Please switch to XPC or bootstrap_check_in(): com.apple.quicklook",
    "Oct  6 17:13:26 md-dsk com.apple.xpc.launchd[1] (com.apple.imfoundation.IMRemoteURLConnectionAgent): Unknown key for integer: _DirtyJetsamMemoryLimit",
    "Oct  6 17:14:54 md-dsk com.apple.xpc.launchd[1] (com.apple.quicklook[31206]): Endpoint has been activated through legacy launch(3) APIs. Please switch to XPC or bootstrap_check_in(): com.apple.quicklook",
    "Oct  6 17:17:22 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 17:26:00 md-dsk com.apple.xpc.launchd[1] (com.apple.imfoundation.IMRemoteURLConnectionAgent): Unknown key for integer: _DirtyJetsamMemoryLimit",
    "Oct  6 17:27:31 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 17:37:42 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 17:40:48 md-dsk com.apple.xpc.launchd[1] (com.apple.quicklook[32707]): Endpoint has been activated through legacy launch(3) APIs. Please switch to XPC or bootstrap_check_in(): com.apple.quicklook",
    "Oct  6 17:42:32 md-dsk com.apple.xpc.launchd[1] (com.apple.imfoundation.IMRemoteURLConnectionAgent): Unknown key for integer: _DirtyJetsamMemoryLimit",
    "Oct  6 17:44:36 md-dsk Dock[308]: DEPRECATED USE in libdispatch client: Setting timer interval to 0 requests a 1ns timer, did you mean FOREVER (a one-shot timer)?",
    "Oct  6 17:44:36 md-dsk Dock[308]: BUG in libdispatch client: kevent[EVFILT_MACHPORT] add: \"No such file or directory\" - 0x2",
    "Oct  6 17:49:46 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 17:56:41 md-dsk com.apple.xpc.launchd[1] (com.apple.imfoundation.IMRemoteURLConnectionAgent): Unknown key for integer: _DirtyJetsamMemoryLimit",
    "Oct  6 17:58:38 md-dsk login[37744]: USER_PROCESS: 37744 ttys002",
    "Oct  6 18:01:15 md-dsk login[37744]: DEAD_PROCESS: 37744 ttys002",
    "Oct  6 18:01:15 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 18:04:12 md-dsk com.apple.xpc.launchd[1] (com.apple.quicklook[38157]): Endpoint has been activated through legacy launch(3) APIs. Please switch to XPC or bootstrap_check_in(): com.apple.quicklook",
    "Oct  6 18:08:22 md-dsk com.apple.xpc.launchd[1] (com.apple.imfoundation.IMRemoteURLConnectionAgent): Unknown key for integer: _DirtyJetsamMemoryLimit",
    "Oct  6 18:11:11 md-dsk com.apple.xpc.launchd[1] (com.apple.quicklook[38222]): Endpoint has been activated through legacy launch(3) APIs. Please switch to XPC or bootstrap_check_in(): com.apple.quicklook",
    "Oct  6 18:11:51 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 18:15:39 md-dsk login[38379]: USER_PROCESS: 38379 ttys002",
    "Oct  6 18:25:08 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 18:36:31 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 18:38:17 md-dsk com.apple.xpc.launchd[1] (com.apple.imfoundation.IMRemoteURLConnectionAgent): Unknown key for integer: _DirtyJetsamMemoryLimit",
    "Oct  6 18:47:13 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 18:57:47 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 18:58:32 md-dsk com.apple.xpc.launchd[1] (com.apple.imfoundation.IMRemoteURLConnectionAgent): Unknown key for integer: _DirtyJetsamMemoryLimit",
    "Oct  6 19:10:31 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 19:21:08 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 19:31:21 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 19:35:15 md-dsk com.apple.xpc.launchd[1] (com.apple.imfoundation.IMRemoteURLConnectionAgent): Unknown key for integer: _DirtyJetsamMemoryLimit",
    "Oct  6 19:41:36 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 19:42:54 md-dsk systemstats[53]: assertion failed: 17G8030: systemstats + 914800 [D1E75C38-62CE-3D77-9ED3-5F6D38EF0676]: 0x40",
    "Oct  6 19:48:44 md-dsk com.apple.xpc.launchd[1] (com.apple.imfoundation.IMRemoteURLConnectionAgent): Unknown key for integer: _DirtyJetsamMemoryLimit",
    "Oct  6 19:51:21 md-dsk com.apple.xpc.launchd[1] (com.adobe.ARMDCHelper.cc24aef4a1b90ed56a725c38014c95072f92651fb65e1bf9c8e43c37a23d420d[42603]): Service exited with abnormal code: 111",
    "Oct  6 19:56:02 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 20:00:48 md-dsk com.apple.xpc.launchd[1] (com.apple.quicklook[42860]): Endpoint has been activated through legacy launch(3) APIs. Please switch to XPC or bootstrap_check_in(): com.apple.quicklook",
    "Oct  6 20:01:04 md-dsk com.apple.xpc.launchd[1] (com.apple.appkit.xpc.openAndSavePanelService[24683]): Service did not exit 5 seconds after SIGTERM. Sending SIGKILL.",
    "Oct  6 20:01:04 md-dsk systemstats[53]: assertion failed: 17G8030: systemstats + 914800 [D1E75C38-62CE-3D77-9ED3-5F6D38EF0676]: 0x40",
    "Oct  6 20:06:24 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 20:16:27 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 20:25:44 md-dsk com.apple.xpc.launchd[1] (com.apple.imfoundation.IMRemoteURLConnectionAgent): Unknown key for integer: _DirtyJetsamMemoryLimit",
    "Oct  6 20:26:44 md-dsk syslogd[40]: ASL Sender Statistics",
    "Oct  6 20:32:58 md-dsk login[43775]: USER_PROCESS: 43775 ttys003",
    "Oct  6 20:33:20 md-dsk diagnosticd[43858]: no EOS device present",
    "Oct  6 20:33:20 md-dsk diagnosticd[43858]: System mode client started - Console (43856) - mode: 0xb",
    "Oct  6 20:33:27 md-dsk Console[43856]: BUG in libdispatch client: kevent[vnode] monitored resource vanished before the source cancel handler was invoked",
    "Oct  6 20:33:28 md-dsk com.apple.xpc.launchd[1] (com.apple.imfoundation.IMRemoteURLConnectionAgent): Unknown key for integer: _DirtyJetsamMemoryLimit"
};

void LogScreen::initializeGui() {
    screen = lv_obj_create(nullptr, nullptr);

    imgBackground = lv_img_create(screen, nullptr);
    lv_img_set_src(imgBackground, &Background);

    lstLog = lv_list_create(screen, nullptr);
    lv_obj_set_size(lstLog, 310, 231);
    lv_obj_set_style_local_text_font(lstLog, LV_LIST_PART_BG, LV_STATE_DEFAULT, &Montserrat12Font);
    lv_obj_align(lstLog, nullptr, LV_ALIGN_IN_BOTTOM_MID, 0, -5);

    timerTaskHandle = lv_task_create(timerTaskHandler, 3000, LV_TASK_PRIO_MID, this);
}

void LogScreen::deinitializeGui() {
    lv_task_del(timerTaskHandle);

    lv_obj_del(imgBackground);

    lv_obj_del(lstLog);
}

void LogScreen::timerTaskHandler(lv_task_t *task) {
    auto *instance = static_cast<LogScreen *>(task->user_data);
    instance->timerTask();
}

void LogScreen::timerTask() {
    static uint32_t lineIndex = 0;
    static uint32_t linesCount = sizeof(lines) / sizeof(lines[0]);

    uint32_t linePosition = (lineIndex++) % (linesCount - 1);
    addLine(lines[linePosition]);
}

void LogScreen::addLine(const char *line) {
    uint16_t listSize = lv_list_get_size(lstLog);
    if (listSize == 7) {
        lv_list_remove(lstLog, 0);
    }

    lv_obj_t *button = lv_list_add_btn(lstLog, nullptr, line);
    lv_obj_t *label = lv_list_get_btn_label(button);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
}
