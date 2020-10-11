cmake_minimum_required(VERSION 3.17)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(EspIdf)
find_package(Espressif)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
        components/lv_port_esp32/components/lvgl
        components/lv_port_esp32/components/lvgl_esp32_drivers/lvgl_tft
        components/lv_port_esp32/components/lvgl_esp32_drivers/lvgl_touch
        components/lv_port_esp32/components/lvgl_esp32_drivers)

project(agv_remote_control)
