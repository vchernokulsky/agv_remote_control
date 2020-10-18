idf_component_register(
        SRCS src/button.c src/button_obj.cpp
        INCLUDE_DIRS src/include)

set_source_files_properties(src/button.c PROPERTIES COMPILE_FLAGS -Wno-incompatible-pointer-types)
