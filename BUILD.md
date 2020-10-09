# Build process of AGV Remote Control project

## Prerequisites

1. `macOS` or `Linux` OS required now. `Windows` can be added later (need to write additional scripts).
1. Latest version of [Docker](https://www.docker.com/products/docker-desktop) must be installed.
1. `ESP-IDF` framework should be installed to `~/.esp-idf` directory to auto-detect
   `OR` `IDF_PATH` env variable should be specified (see `cmake/FindEspIdf.cmake` for more details).
   
   See [Step 2. Get ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#step-2-get-esp-idf) for more details.  
1. `Espressif` toolchain should be installed to `~/.espressif` directory to auto-detect 
   `OR` `Espressif` env variables should be initialized manually  (see `cmake/FindEspressif.cmake` for more details)
   
   See [Step 3. Set up the tools](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#id3) for more details.

## Build from CLion IDE 

1. Run `bin/generate_ros_lib.sh` script to generate content of `components/rosserial_esp32/ros_lib` directory.
1. Import CMake project use `Open or import` menu item.
1. Select `app | Debug` configuration to build project or `app-flash | Debug` to build and flash.

## Build from console

1. Import `Espressif` environment variables to console (see [Step 4. Set up the environment variables](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#step-4-set-up-the-environment-variables) for more details)
1. Run `idf.py build` to build or `idf.py flash monitor` to build and flash (UART port can be additionally specified through `--port` command line argument).
