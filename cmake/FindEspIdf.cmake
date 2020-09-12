IF (NOT DEFINED ENV{IDF_PATH})
    SET(ESP_IDF_PATH $ENV{HOME}/.esp-idf)

    IF (NOT EXISTS ${ESP_IDF_PATH})
        message(FATAL_ERROR "'${ESP_IDF_PATH}' directory with ESP-IDF framework not found. "
                "See 'Step 2. Get ESP-IDF' by link https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#step-2-get-esp-idf")
    ENDIF()

    # Set env variable with ESP IDF path
    SET(ENV{IDF_PATH} ${ESP_IDF_PATH})

    # Set additional look-up paths (i.e. with ESP-IDF tools)
    LIST(APPEND CMAKE_SYSTEM_PROGRAM_PATH
            "${ESP_IDF_PATH}/components/esptool_py/esptool"
            "${ESP_IDF_PATH}/components/espcoredump"
            "${ESP_IDF_PATH}/components/partition_table/"
            "${ESP_IDF_PATH}/components/esptool_py/esptool"
            "${ESP_IDF_PATH}/components/espcoredump"
            "${ESP_IDF_PATH}/components/partition_table/"
            "${ESP_IDF_PATH}/tools")
ENDIF()
