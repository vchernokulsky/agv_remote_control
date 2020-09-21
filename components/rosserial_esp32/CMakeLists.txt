FILE(GLOB_RECURSE app_sources ${COMPONENT_DIR}/*.*)

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS . ros_lib)
