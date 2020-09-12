cmake_minimum_required(VERSION 3.17)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(EspIdf)
find_package(Espressif)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(agv_remote_control)
