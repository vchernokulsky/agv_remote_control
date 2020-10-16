IF (NOT DEFINED ENV{IDF_PYTHON_ENV_PATH})
    EXECUTE_PROCESS(
            COMMAND ${ESP_IDF_PATH}/tools/idf_tools.py export --format key-value
            WORKING_DIRECTORY ${ESP_IDF_PATH}/tools
            OUTPUT_VARIABLE IDF_TOOLS_EXPORT)

    STRING(REGEX REPLACE "\n" ";" IDF_TOOLS_EXPORT "${IDF_TOOLS_EXPORT}")

    FOREACH (IDF_TOOLS_EXPORT_LINE IN ITEMS ${IDF_TOOLS_EXPORT})
        IF(IDF_TOOLS_EXPORT_LINE MATCHES "(.*)=(.*)")
            SET(KEY ${CMAKE_MATCH_1})
            SET(VALUE ${CMAKE_MATCH_2})

            IF (${KEY} STREQUAL "IDF_PYTHON_ENV_PATH")
                SET(ENV{IDF_PYTHON_ENV_PATH} ${VALUE})
                SET(ENV{PYTHON} ${VALUE}/bin/python)
            ELSEIF (${KEY} STREQUAL "OPENOCD_SCRIPTS")
                SET(ENV{OPENOCD_SCRIPTS} ${VALUE})
            ELSEIF (${KEY} STREQUAL "PATH")
                STRING(REGEX REPLACE ":" ";" PATH_ITEMS "${VALUE}")

                FOREACH(PATH_ITEM IN ITEMS ${PATH_ITEMS})
                    IF (NOT ${PATH_ITEM} STREQUAL "$PATH")
                        LIST(APPEND CMAKE_SYSTEM_PROGRAM_PATH ${PATH_ITEM})
                    ENDIF()
                ENDFOREACH(PATH_ITEM)
            ENDIF()

        ENDIF()
    ENDFOREACH(IDF_TOOLS_EXPORT_LINE)
ENDIF()