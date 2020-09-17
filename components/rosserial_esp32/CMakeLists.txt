FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/components/rosserial_esp32/*.*)

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS . ros_lib)
