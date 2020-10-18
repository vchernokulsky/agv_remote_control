FILE(GLOB_RECURSE app_sources ${COMPONENT_DIR}/*.*)

idf_component_register(SRCS ${app_sources}
        INCLUDE_DIRS .
        PRIV_REQUIRES esp_adc_cal mdns wpa_supplicant
        REQUIRES rosserial_esp32 lvgl lvgl_tft lvgl_esp32_drivers button)

set_source_files_properties(buttons/ButtonsManager.cpp PROPERTIES COMPILE_FLAGS -Wno-unused-function)