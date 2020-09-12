IF (NOT DEFINED ENV{IDF_PYTHON_ENV_PATH})
    SET(ESPRESSIF_PATH $ENV{HOME}/.espressif)

    IF (NOT EXISTS ${ESPRESSIF_PATH})
        message(FATAL_ERROR "'${ESPRESSIF_PATH}' directory with Espressif tools not found. "
                "See 'Step 3. Set up the tools' by link https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#id3")
    ENDIF()

    # Find Python environment directory
    file(GLOB PYTHON_ENV_DIR "${ESPRESSIF_PATH}/python_env/*")

    file(GLOB XTENSA_ESP32_ELF_DIR "${ESPRESSIF_PATH}/tools/xtensa-esp32-elf/*/xtensa-esp32-elf")
    file(GLOB XTENSA_ESP32S2_ELF_DIR "${ESPRESSIF_PATH}/tools/xtensa-esp32s2-elf/*/xtensa-esp32s2-elf")
    file(GLOB XESP32ULP_ELF_DIR "${ESPRESSIF_PATH}/tools/esp32ulp-elf/*/esp32ulp-elf-binutils")
    file(GLOB XESP32S2ULP_ELF_DIR "${ESPRESSIF_PATH}/tools/esp32s2ulp-elf/*/esp32s2ulp-elf-binutils")
    file(GLOB OPENOCD_ESP32_DIR "${ESPRESSIF_PATH}/tools/openocd-esp32/*/openocd-esp32")

    # Set env variables with Python environment directory and executable Python file
    SET(ENV{IDF_PYTHON_ENV_PATH} ${PYTHON_ENV_DIR})
    SET(ENV{PYTHON} ${PYTHON_ENV_DIR}/bin/python)

    # Set additional look-up paths (i.e. with C/CPP compilers, etc.)
    LIST(APPEND CMAKE_SYSTEM_PROGRAM_PATH
            "${XTENSA_ESP32_ELF_DIR}/bin"
            "${XTENSA_ESP32S2_ELF_DIR}/bin"
            "${XESP32ULP_ELF_DIR}/bin"
            "${XESP32S2ULP_ELF_DIR}/bin"
            "${OPENOCD_ESP32_DIR}/bin"
            "${PYTHON_ENV_DIR}/bin")
ENDIF()